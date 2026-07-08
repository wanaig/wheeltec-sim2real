/**
 * MCPTools.js — MCP (Model Context Protocol) 工具集
 *
 * 暴露机器人控制工具供大模型 (LLM) 通过 function calling 调用:
 *   1. perceive          — 感知场景中的工具/零件
 *   2. move_base         — 底盘导航到指定世界坐标
 *   3. plan_arm_motion   — MoveIt2 运动规划 (IK + 碰撞检测 + 轨迹生成)
 *   4. execute_arm_motion— 执行规划好的运动轨迹
 *   5. grasp             — 闭合夹爪抓取
 *   6. release           — 张开夹爪放置
 *   7. verify            — 验证抓取/放置结果
 *   8. get_robot_state   — 获取机器人当前状态
 *   9. get_scene_info    — 获取场景信息 (料箱格子/工作台位置)
 *
 * 工具定义符合 OpenAI function calling 格式 (JSON Schema)。
 * 执行器 MCPToolExecutor 把工具调用路由到 MockAgent/RobotModel/IKSolver。
 */
import * as THREE from 'three';
import { PRESETS, GRIPPER_POSES, ARM_JOINT_NAMES, GRIPPER_JOINT_NAMES } from './RobotModel.js';

// ─────────────── 环境碰撞模型 (MoveIt2 Planning Scene 仿真) ───────────────

// 工作台碰撞盒 (AABB): 臂连杆不能穿入这些区域
// 台面实心: z[0.040, 0.045] (碰撞盒比视觉台面 z=0.060 低 15mm, 给夹爪手指下沉余量);
// 料箱壁: z[0.060, 0.113] (薄壁, 不覆盖格子内部空间)
const COLLISION_BOXES = [
  // 左台 (料箱) 台面
  { name: 'bin_bench', x: [0.225, 0.375], y: [-0.49, -0.11], z: [0.040, 0.045] },
  // 右台 (工具) 台面
  { name: 'tool_bench', x: [0.225, 0.375], y: [0.11, 0.49], z: [0.040, 0.045] },
  // 料箱隔板 (4条, 薄壁 y 方向)
  { name: 'bin_wall_p1', x: [0.25, 0.35], y: [-0.482, -0.478], z: [0.060, 0.113] },
  { name: 'bin_wall_p2', x: [0.25, 0.35], y: [-0.362, -0.358], z: [0.060, 0.113] },
  { name: 'bin_wall_p3', x: [0.25, 0.35], y: [-0.242, -0.238], z: [0.060, 0.113] },
  { name: 'bin_wall_p4', x: [0.25, 0.35], y: [-0.122, -0.118], z: [0.060, 0.113] },
  // 料箱侧壁 (左右, 薄壁 x 方向)
  { name: 'bin_wall_left',  x: [0.248, 0.252], y: [-0.48, -0.12], z: [0.060, 0.113] },
  { name: 'bin_wall_right', x: [0.348, 0.352], y: [-0.48, -0.12], z: [0.060, 0.113] },
];

const ARM_BASE_OFFSET = { x: 0.054476, y: 0.00070272, z: 0.156 };
const ARM_REACH = 0.40;
const ARM_COMFORT_DIST = 0.28;
const MOBILE_AREA = { xMin: -0.25, xMax: 0.70, yMin: -0.62, yMax: 0.62 };
const CHASSIS_RADIUS = 0.13;
const CHASSIS_CLEARANCE = 0.03;
const WORKBENCH_FRONT_STANDOFF = 0.12;
const MAX_FRONT_APPROACH_ANGLE = 1.25; // 约72°, 禁止侧后方/背向工作台操作
const MAX_WORKBENCH_LATERAL_OFFSET = 0.16;

/** 检查点是否在碰撞盒内 */
function _pointInBox(x, y, z, box) {
  return x >= box.x[0] && x <= box.x[1] &&
         y >= box.y[0] && y <= box.y[1] &&
         z >= box.z[0] && z <= box.z[1];
}

/** 检查点是否与任何工作台碰撞 */
function _pointInCollision(x, y, z) {
  for (const box of COLLISION_BOXES) {
    if (_pointInBox(x, y, z, box)) return box.name;
  }
  return null;
}

/**
 * 检查机械臂所有连杆是否与环境碰撞
 * @returns {string|null} 碰撞的连杆名, null=无碰撞
 */
function _checkArmEnvCollision(robot) {
  robot.root.updateMatrixWorld(true);
  const links = ['link1', 'link2', 'link3', 'link4', 'link5', 'link6', 'link7', 'link8', 'link9', 'link10', 'link11'];
  const corner = new THREE.Vector3();
  for (const name of links) {
    const mesh = robot.linkMeshes?.[name];
    const bb = robot.boundingBoxes?.[name];
    if (!mesh || !bb) continue;
    const mins = [bb.min.x, bb.min.y, bb.min.z];
    const maxs = [bb.max.x, bb.max.y, bb.max.z];
    for (let i = 0; i < 8; i++) {
      corner.set(
        (i & 1) ? maxs[0] : mins[0],
        (i & 2) ? maxs[1] : mins[1],
        (i & 4) ? maxs[2] : mins[2],
      ).applyMatrix4(mesh.matrixWorld);
      const hit = _pointInCollision(corner.x, corner.y, corner.z);
      if (hit) return `${name}↔${hit}`;
    }
  }
  return null;
}

function _jointsWithinLimits(robot, joints) {
  for (const n of ARM_JOINT_NAMES) {
    const lim = robot.jointLimits?.[n];
    if (!lim) continue;
    const v = joints[n] ?? 0;
    if (v < lim[0] - 1e-6 || v > lim[1] + 1e-6) return false;
  }
  return true;
}

/**
 * 在关节插值轨迹上采样检查碰撞
 * @param {object} robot - RobotModel
 * @param {object} fromJoints - 起始关节角
 * @param {object} toJoints - 目标关节角
 * @param {number} samples - 采样数
 * @returns {{collision: string|null, safeHeight: number}}
 */
function _checkTrajectoryCollision(robot, fromJoints, toJoints, samples = 20) {
  let maxZ = 0;
  let collisionFound = null;
  const names = ARM_JOINT_NAMES;
  const from = names.map(n => fromJoints[n] ?? 0);
  const to = names.map(n => toJoints[n] ?? 0);
  // 保存当前关节 (检查后恢复, 避免视觉抽搐)
  const saved = names.map(n => robot.jointValues[n] ?? 0);
  // 临时关闭 RobotModel 自碰撞 (只检查环境碰撞)
  const wasEnabled = robot.collisionEnabled;
  robot.collisionEnabled = false;

  for (let i = 0; i <= samples; i++) {
    const t = i / samples;
    const vals = names.map((n, idx) => from[idx] + (to[idx] - from[idx]) * t);
    robot.applyJointState(names, vals);
    // 检查环境碰撞
    const hit = _checkArmEnvCollision(robot);
    if (hit) {
      collisionFound = hit;
      break;
    }
    const ground = robot._checkGroundCollision?.();
    if (ground) {
      collisionFound = `ground↔${ground}`;
      break;
    }
    // 记录最高 TCP Z (用于安全高度)
    const tcp = robot.getGripperTCP();
    if (tcp.z > maxZ) maxZ = tcp.z;
  }
  // 恢复关节状态和碰撞设置
  robot.applyJointState(names, saved);
  robot.collisionEnabled = wasEnabled;
  return { collision: collisionFound, safeHeight: maxZ };
}

function _wrapAngle(rad) {
  return Math.atan2(Math.sin(rad), Math.cos(rad));
}

function _jointDistance(a, b) {
  let sum = 0;
  for (const n of ARM_JOINT_NAMES) {
    const d = _wrapAngle((a[n] ?? 0) - (b[n] ?? 0));
    sum += d * d;
  }
  return Math.sqrt(sum);
}

function _inMobileArea(x, y) {
  const r = CHASSIS_RADIUS;
  return y >= MOBILE_AREA.yMin + r && y <= MOBILE_AREA.yMax - r &&
         x >= MOBILE_AREA.xMin + r && x <= MOBILE_AREA.xMax - r;
}

function _inBenchFootprint(x, y, margin = CHASSIS_RADIUS + CHASSIS_CLEARANCE) {
  for (const box of COLLISION_BOXES) {
    if (x >= box.x[0] - margin && x <= box.x[1] + margin &&
        y >= box.y[0] - margin && y <= box.y[1] + margin) return true;
  }
  return false;
}

function _isWorkbenchTarget(target) {
  return target.x >= 0.20 && target.x <= 0.40 &&
         ((target.y >= 0.11 && target.y <= 0.49) || (target.y >= -0.49 && target.y <= -0.11)) &&
         target.z >= 0.04 && target.z <= 0.16;
}

function _frontApproachAngles(target) {
  if (!_isWorkbenchTarget(target)) return null;
  // 工作台只能从正面(x较小的一侧)接近, 允许少量斜向误差, 禁止绕到侧后方/背面。
  return [Math.PI, Math.PI - Math.PI / 10, Math.PI + Math.PI / 10];
}

// ─────────────── MCP 工具定义 (OpenAI function calling 格式) ───────────────

export const MCP_TOOLS = [
  {
    type: 'function',
    function: {
      name: 'perceive',
      description: '感知当前场景中所有可见的工具和零件, 返回类别、世界坐标(x,y,z米)、置信度。仅在工具未被抓取时可见。',
      parameters: { type: 'object', properties: {}, required: [] },
    },
  },
  {
    type: 'function',
    function: {
      name: 'move_base',
      description: '驱动底盘导航到指定世界坐标 (x, y) 米, 朝向 yaw 弧度。底盘移动后臂的可达范围改变。用于目标超出臂展时靠近目标。',
      parameters: {
        type: 'object',
        properties: {
          x: { type: 'number', description: '目标X坐标 (米), 前方为正' },
          y: { type: 'number', description: '目标Y坐标 (米), 左为正' },
          yaw: { type: 'number', description: '目标朝向 (弧度), 默认0', default: 0 },
        },
        required: ['x', 'y'],
      },
    },
  },
  {
    type: 'function',
    function: {
      name: 'plan_arm_motion',
      description: 'MoveIt2规划并执行臂运动到目标坐标(x,y,z)。多起始点IK求解+碰撞检测+经由点轨迹, 自动避穿模。一次调用完成规划+执行, 无需单独execute。抓取时直接传目标高度即可, 规划器自动经安全高度绕行。',
      parameters: {
        type: 'object',
        properties: {
          x: { type: 'number', description: '目标TCP X坐标 (米)' },
          y: { type: 'number', description: '目标TCP Y坐标 (米)' },
          z: { type: 'number', description: '目标TCP Z坐标 (米)' },
        },
        required: ['x', 'y', 'z'],
      },
    },
  },
  {
    type: 'function',
    function: {
      name: 'grasp',
      description: '闭合夹爪, 抓取距离TCP最近且已贴近的工具(3.5cm内)。返回是否抓取成功。',
      parameters: { type: 'object', properties: {}, required: [] },
    },
  },
  {
    type: 'function',
    function: {
      name: 'release',
      description: '张开夹爪, 松开当前抓取的工具, 在TCP位置放置。',
      parameters: { type: 'object', properties: {}, required: [] },
    },
  },
  {
    type: 'function',
    function: {
      name: 'retract',
      description: '收回机械臂: 多策略垂直抬升(逆序/多解IK/后退脱离)到安全高度, 再收到安全姿态, 保持当前方向。抓取/放置后必须调用。碰撞时系统自动切换策略, 无需手动处理。',
      parameters: {
        type: 'object',
        properties: {
          lift: { type: 'number', description: '额外抬升高度 (米), 默认0.08', default: 0.08 },
        },
        required: [],
      },
    },
  },
  {
    type: 'function',
    function: {
      name: 'verify',
      description: '验证操作结果。type=grasp检查工具是否被抓取(离开原位置), type=place检查工具是否在指定格子内。',
      parameters: {
        type: 'object',
        properties: {
          type: { type: 'string', enum: ['grasp', 'place'], description: '验证类型' },
          slot: { type: 'integer', description: '格子编号(1-3), 仅place时需要', minimum: 1, maximum: 3 },
        },
        required: ['type'],
      },
    },
  },
  {
    type: 'function',
    function: {
      name: 'get_robot_state',
      description: '获取机器人当前状态: 底盘位置(x,y,yaw)、臂关节角(joint1-5)、夹爪状态(open/close)、TCP世界坐标。',
      parameters: { type: 'object', properties: {}, required: [] },
    },
  },
  {
    type: 'function',
    function: {
      name: 'get_scene_info',
      description: '获取场景信息: 双工作台位置、料箱格子坐标、臂展范围、工具台区域。',
      parameters: { type: 'object', properties: {}, required: [] },
    },
  },
];

// ─────────────── MCP 工具执行器 ───────────────

export class MCPToolExecutor {
  /**
   * @param {object} ctx — { mockAgent, robot, ik, chassis, ros, binSlots }
   */
  constructor(ctx) {
    this.agent = ctx.mockAgent;
    this.robot = ctx.robot;
    this.ik = ctx.ik;
    this.chassis = ctx.chassis;
    this.ros = ctx.ros;
    this.binSlots = ctx.binSlots || {};
    this._plannedTrajectory = null;  // plan_arm_motion → execute_arm_motion
    this._lastAboveJoints = null;   // 最近一次 plan_arm_motion 下降段"目标正上方"关节角, 供 retract 逆序垂直抬升
    this._logCb = null;
  }

  onLog(cb) { this._logCb = cb; }
  _log(msg) { if (this._logCb) this._logCb(msg); }

  _sendArmCommand(targetMap) {
    if (!this.ros?.publishEnabled || !this.ros?.connected) return;
    this.ros.sendArmCommand(targetMap);
  }

  _tweenArmTo(targetMap, duration) {
    this._sendArmCommand(targetMap);
    return new Promise(resolve => {
      this.robot.tweenTo(targetMap, duration, () => {
        this._sendArmCommand(targetMap);
        resolve();
      });
    });
  }

  /** 获取当前夹持的工具 (null=未持物) */
  _getHeldObject() {
    if (!this.agent?.tools) return null;
    const held = this.agent.tools.find(t => t.grasped);
    return held ? { class: held.class, xyz: [...held.currentXyz] } : null;
  }

  /** 获取各料箱格子中的物体列表 (用于进度跟踪) */
  _getBinOccupancy() {
    const result = {};
    for (const [slotNum, slotXyz] of Object.entries(this.binSlots)) {
      const objects = [];
      if (this.agent?.tools) {
        for (const t of this.agent.tools) {
          if (t.grasped) continue;
          const d = Math.hypot(t.currentXyz[0] - slotXyz[0], t.currentXyz[1] - slotXyz[1]);
          if (d < 0.06) objects.push(t.class);
        }
      }
      result[slotNum] = objects;
    }
    return result;
  }

  /** 给工具结果追加机器人状态 (夹爪/持物), 让 LLM 每轮都能看到当前状态 */
  _enrichState(r) {
    if (r.gripper !== undefined) return r; // get_robot_state 已自带, 不覆盖
    const held = this._getHeldObject();
    r.gripper = held ? 'closed' : 'open';
    r.holding = held ? held.class : null;
    return r;
  }

  _suggestChassisFor(target) {
    const cur = this.robot.root.position;
    const curYaw = this.robot.root.rotation.z;
    const armBase = new THREE.Vector3();
    this.robot.jointGroups['joint1'].getWorldPosition(armBase);
    const dz = target.z - ARM_BASE_OFFSET.z;
    const planarDist = Math.max(0.18, Math.min(0.34,
      Math.sqrt(Math.max(0, ARM_COMFORT_DIST * ARM_COMFORT_DIST - dz * dz))));
    const fixedFrontAngles = _frontApproachAngles(target);
    const angles = [];
    const currentSide = Math.atan2(armBase.y - target.y, armBase.x - target.x);
    if (fixedFrontAngles) {
      angles.push(...fixedFrontAngles);
    } else {
      angles.push(currentSide);
      for (let i = 0; i < 16; i++) angles.push(i * Math.PI / 8);
    }

    // 保存机器人状态 (碰撞验证会修改底盘位姿和关节)
    const svPos = { x: cur.x, y: cur.y, z: this.robot.root.position.z };
    const svYaw = curYaw;
    const svArm = {}; ARM_JOINT_NAMES.forEach(j => svArm[j] = this.robot.getJoint(j));
    const svGrip = {}; GRIPPER_JOINT_NAMES.forEach(j => svGrip[j] = this.robot.getJoint(j));

    let bestValid = null;  // 碰撞-free 候选
    let bestAny = null;    // 任意候选 (fallback)

    for (const a of angles) {
      const standoff = fixedFrontAngles ? Math.max(planarDist, WORKBENCH_FRONT_STANDOFF) : planarDist;
      const armGoalX = target.x + Math.cos(a) * standoff;
      const armGoalY = target.y + Math.sin(a) * standoff;
      const yaw = Math.atan2(target.y - armGoalY, target.x - armGoalX);
      const cos = Math.cos(yaw), sin = Math.sin(yaw);
      const x = armGoalX - (cos * ARM_BASE_OFFSET.x - sin * ARM_BASE_OFFSET.y);
      const y = armGoalY - (sin * ARM_BASE_OFFSET.x + cos * ARM_BASE_OFFSET.y);
      if (!_inMobileArea(x, y) || _inBenchFootprint(x, y)) continue;
      const armDistance = Math.hypot(target.x - armGoalX, target.y - armGoalY, target.z - ARM_BASE_OFFSET.z);
      if (armDistance > ARM_REACH) continue;
      const travel = Math.hypot(x - cur.x, y - cur.y);
      const turn = Math.abs(_wrapAngle(yaw - curYaw)) * 0.05;
      const score = travel + turn;
      if (!bestAny || score < bestAny.score) bestAny = { x, y, yaw, score };

      // 碰撞验证: 模拟底盘到位, IK求解 + 下降段碰撞检测 (与 plan_arm_motion 一致)
      this.robot.root.position.set(x, y, 0);
      this.robot.root.rotation.z = yaw;
      this.robot.root.updateMatrixWorld(true);

      // 恢复保存的臂状态 (与 plan_arm_motion 的 IK 起始点一致, 避免解分支不一致)
      this.robot.applyJointState(ARM_JOINT_NAMES,
        ARM_JOINT_NAMES.map(j => svArm[j]));

      // Via2: 目标正上方安全高度 (先求解, 用作 target IK 的 preferred → 同分支解)
      const safeZ = Math.max(target.z + 0.06, 0.14);
      const r2 = this._multiStartIK(new THREE.Vector3(target.x, target.y, safeZ), svArm);
      if (!r2.solved) continue;
      const v2J = {};
      ARM_JOINT_NAMES.forEach(j => v2J[j] = Math.atan2(
        Math.sin(r2.joints[j]), Math.cos(r2.joints[j])));
      if (fixedFrontAngles && Math.abs(v2J.joint1 ?? 0) > MAX_FRONT_APPROACH_ANGLE) continue;

      // Target IK (preferred: via2 → 同分支, 下降更垂直, 减少料箱壁碰撞)
      const ikRes = this._multiStartIK(target, v2J);
      if (!ikRes.solved) continue;
      const ikJ = {};
      ARM_JOINT_NAMES.forEach(j => ikJ[j] = Math.atan2(
        Math.sin(ikRes.joints[j]), Math.cos(ikRes.joints[j])));
      if (fixedFrontAngles && Math.abs(ikJ.joint1 ?? 0) > MAX_FRONT_APPROACH_ANGLE) continue;

      // 目标姿态碰撞 (两种夹爪)
      let goalHit = null;
      for (const grip of [GRIPPER_POSES.close, GRIPPER_POSES.open]) {
        this.robot.applyJointState(GRIPPER_JOINT_NAMES,
          GRIPPER_JOINT_NAMES.map(j => grip[j] ?? 0));
        this.robot.applyJointState(ARM_JOINT_NAMES, ARM_JOINT_NAMES.map(j => ikJ[j]));
        const hit = _checkArmEnvCollision(this.robot) || this.robot._checkGroundCollision?.();
        if (hit) { goalHit = hit; break; }
      }
      if (goalHit) continue;

      // C3 下降轨迹碰撞 (两种夹爪, 与 plan_arm_motion 完全一致)
      let c3Hit = null;
      for (const grip of [GRIPPER_POSES.close, GRIPPER_POSES.open]) {
        this.robot.applyJointState(GRIPPER_JOINT_NAMES,
          GRIPPER_JOINT_NAMES.map(j => grip[j] ?? 0));
        const c = _checkTrajectoryCollision(this.robot, v2J, ikJ, 20);
        if (c.collision) { c3Hit = c.collision; break; }
      }
      if (c3Hit) continue;

      if (!bestValid || score < bestValid.score) bestValid = { x, y, yaw, score };
    }

    // 恢复机器人状态
    this.robot.root.position.set(svPos.x, svPos.y, svPos.z);
    this.robot.root.rotation.z = svYaw;
    this.robot.applyJointState(ARM_JOINT_NAMES, ARM_JOINT_NAMES.map(j => svArm[j]));
    this.robot.applyJointState(GRIPPER_JOINT_NAMES, GRIPPER_JOINT_NAMES.map(j => svGrip[j]));
    this.robot.root.updateMatrixWorld(true);

    const best = bestValid || bestAny;
    if (best) {
      return {
        x: +best.x.toFixed(3), y: +best.y.toFixed(3), yaw: +best.yaw.toFixed(3),
        no_collision_free: !bestValid,
      };
    }

    if (fixedFrontAngles) {
      const standoff = Math.max(planarDist, WORKBENCH_FRONT_STANDOFF);
      const yaw = 0;
      const cos = Math.cos(yaw), sin = Math.sin(yaw);
      const armGoalX = target.x - standoff;
      const armGoalY = target.y;
      const x = armGoalX - (cos * ARM_BASE_OFFSET.x - sin * ARM_BASE_OFFSET.y);
      const y = armGoalY - (sin * ARM_BASE_OFFSET.x + cos * ARM_BASE_OFFSET.y);
      return { x: +x.toFixed(3), y: +y.toFixed(3), yaw: 0, no_collision_free: true };
    }

    // Fallback: 环形搜索安全站位 (不进入工作台占地)
    for (let r = 0.10; r <= 0.80; r += 0.04) {
      const n = Math.max(16, Math.ceil(2 * Math.PI * r / 0.04));
      for (let i = 0; i < n; i++) {
        const a = i * 2 * Math.PI / n;
        const fx = target.x - ARM_BASE_OFFSET.x + Math.cos(a) * r;
        const fy = target.y - ARM_BASE_OFFSET.y + Math.sin(a) * r;
        if (!_inMobileArea(fx, fy) || _inBenchFootprint(fx, fy)) continue;
        const armGoalX = fx + ARM_BASE_OFFSET.x, armGoalY = fy + ARM_BASE_OFFSET.y;
        const armDist = Math.hypot(target.x - armGoalX, target.y - armGoalY, target.z - ARM_BASE_OFFSET.z);
        if (armDist > ARM_REACH) continue;
        return { x: +fx.toFixed(3), y: +fy.toFixed(3), yaw: 0, no_collision_free: true };
      }
    }
    // 最终兜底: 取工作台侧方的安全区域
    return {
      x: +Math.max(MOBILE_AREA.xMin + CHASSIS_RADIUS, Math.min(0, target.x - ARM_BASE_OFFSET.x)).toFixed(3),
      y: +(target.y >= 0 ? Math.max(MOBILE_AREA.yMin + CHASSIS_RADIUS, -0.05) : Math.min(MOBILE_AREA.yMax - CHASSIS_RADIUS, 0.05)).toFixed(3),
      yaw: 0,
      no_collision_free: true,
    };
  }

  _validateWorkbenchApproach(target) {
    if (!_isWorkbenchTarget(target)) return null;
    const armBase = new THREE.Vector3();
    this.robot.jointGroups['joint1'].getWorldPosition(armBase);
    const baseToTargetYaw = Math.atan2(target.y - armBase.y, target.x - armBase.x);
    const relYaw = _wrapAngle(baseToTargetYaw - this.robot.root.rotation.z);
    const frontClearance = target.x - armBase.x;
    const lateralOffset = Math.abs(target.y - armBase.y);
    if (frontClearance < WORKBENCH_FRONT_STANDOFF ||
        lateralOffset > MAX_WORKBENCH_LATERAL_OFFSET ||
        Math.abs(relYaw) > MAX_FRONT_APPROACH_ANGLE) {
      const suggested = this._suggestChassisFor(target);
      return {
        ok: false,
        reason: 'unsafe_approach',
        detail: `禁止从侧后方/背向/跨工作台操作: arm_base=(${armBase.x.toFixed(3)}, ${armBase.y.toFixed(3)}), target=(${target.x.toFixed(3)}, ${target.y.toFixed(3)}), lateral=${lateralOffset.toFixed(3)}m, relative_yaw=${relYaw.toFixed(2)}rad`,
        suggested_chassis: suggested,
        hint: `请先move_base到(${suggested.x.toFixed(2)}, ${suggested.y.toFixed(2)}, yaw=${suggested.yaw.toFixed(2)})，使车头正对工作台后再规划机械臂`,
      };
    }
    return null;
  }

  /**
   * 执行工具调用
   * @param {string} name — 工具名
   * @param {object} params — 参数
   * @returns {Promise<object>} 工具返回值 (JSON 可序列化, 会传回 LLM)
   */
  async execute(name, params = {}) {
    this._log(`[MCP] 调用工具: ${name}(${JSON.stringify(params)})`);
    try {
      const r = await this._dispatch(name, params);
      this._enrichState(r);  // 每个工具结果都带夹爪/持物状态, 防止 LLM 状态混淆
      this._log(`[MCP] ${name} → ${r.ok ? 'ok' : 'fail'}${r.reason ? ': ' + r.reason : ''}${r.holding ? ' (holding:' + r.holding + ')' : ''}`);
      // perceive 成功时回调通知面板 (供 UI 更新物体检测)
      if (name === 'perceive' && r.ok && this.onToolResult) {
        try { this.onToolResult(name, r); } catch (e) { /* 忽略回调异常 */ }
      }
      return r;
    } catch (e) {
      this._log(`[MCP] ${name} 异常: ${e.message}`);
      return { ok: false, reason: e.message };
    }
  }

  async _dispatch(name, p) {
    switch (name) {
      case 'perceive':         return this._perceive();
      case 'move_base':        return this._moveBase(p);
      case 'plan_arm_motion':  return this._planArmMotion(p);
      case 'grasp':            return this._grasp();
      case 'release':          return this._release();
      case 'retract':          return this._retract(p);
      case 'verify':           return this._verify(p);
      case 'get_robot_state':  return this._getRobotState();
      case 'get_scene_info':   return this._getSceneInfo();
      default:                 return { ok: false, reason: `unknown_tool: ${name}` };
    }
  }

  // ── perceive ──
  _perceive() {
    const objs = this.agent._perceive();
    const armBase = new THREE.Vector3();
    this.robot.jointGroups['joint1'].getWorldPosition(armBase);
    const armX = armBase.x;
    const armY = armBase.y;
    const armZ = armBase.z;
    const held = this._getHeldObject();
    return {
      ok: true,
      objects: objs.map(o => {
        const dist = Math.hypot(o.xyz[0] - armX, o.xyz[1] - armY, o.xyz[2] - armZ);
        const reachable = dist <= ARM_REACH;
        const obj = {
          class: o.class,
          position: { x: +o.xyz[0].toFixed(3), y: +o.xyz[1].toFixed(3), z: +o.xyz[2].toFixed(3) },
          confidence: o.conf,
          distance_m: +dist.toFixed(3),
          reachable,
          real: !!o.real,
        };
        if (!reachable) {
          obj.suggested_chassis = this._suggestChassisFor(new THREE.Vector3(o.xyz[0], o.xyz[1], o.xyz[2]));
        }
        return obj;
      }),
      count: objs.length,
      arm_base: { x: +armX.toFixed(3), y: +armY.toFixed(3), z: +armZ.toFixed(3) },
      arm_reach_m: 0.40,
      gripper: held ? 'closed' : 'open',
      holding: held ? held.class : null,
      bin_occupancy: this._getBinOccupancy(),
    };
  }

  // ── move_base ──
  async _moveBase(p) {
    // 先用 MCP retract 把臂抬到安全高度 (多策略抬升, 优于 _moveChassisTo 的直接插值)
    // 避免"臂未收回→先抬→碰撞"死循环
    const curJoints = {};
    ARM_JOINT_NAMES.forEach(j => curJoints[j] = this.robot.getJoint(j));
    const isUplift = curJoints['joint2'] > 0.3 && curJoints['joint3'] > 1.0;
    if (!isUplift) {
      const r = await this._retract({});
      if (!r.ok) {
        return { ok: false, reason: 'arm_stuck', detail: `收回机械臂失败, move_base中止: ${r.detail || r.reason}` };
      }
    }
    const result = await this.agent._moveChassisTo(p.x, p.y, p.yaw || 0, 2.0);
    if (!result.ok) return result;
    const pos = this.robot.root.position;
    const dist = Math.hypot(pos.x - p.x, pos.y - p.y);
    return {
      ok: true,
      chassis_position: { x: +pos.x.toFixed(3), y: +pos.y.toFixed(3), yaw: +this.robot.root.rotation.z.toFixed(3) },
      reached_target: dist < 0.05,
      target_distance_m: +dist.toFixed(3),
    };
  }

  // ── 多起始点IK求解 (预旋转joint1面朝目标 + 多planar起始姿态) ──
  _multiStartIK(target, preferred = null) {
    const cur = {};
    ARM_JOINT_NAMES.forEach(j => cur[j] = this.robot.getJoint(j));

    // 计算臂基座世界坐标 → 目标方向的yaw角
    const armBase = new THREE.Vector3();
    this.robot.jointGroups['joint1'].getWorldPosition(armBase);
    const targetYaw = Math.atan2(target.y - armBase.y, target.x - armBase.x);

    // 起始姿态: joint1预旋转到目标方向 + 不同planar初始姿态
    // arm_place (joint2=0.14) 优先: 更竖直, 避免连杆与料箱壁/工作台碰撞
    const startPoints = [
      { name: 'current', joints: cur },
      { name: 'pre_rot_place',  joints: { joint1: targetYaw, joint2: 0.14, joint3: 1.57, joint4: 1.57, joint5: 0 } },
      { name: 'pre_rot_up',    joints: { joint1: targetYaw, joint2: 0.54, joint3: 1.57, joint4: 1.57, joint5: 0 } },
      { name: 'pre_rot_reach', joints: { joint1: targetYaw, joint2: -0.8,  joint3: 1.2,  joint4: 1.0,  joint5: 0 } },
      { name: 'pre_rot_home',  joints: { joint1: targetYaw, joint2: 0,    joint3: 0,    joint4: 0,    joint5: 0 } },
      { name: 'arm_uplift',    joints: PRESETS.arm_uplift },
      { name: 'arm_home',      joints: PRESETS.arm_home },
    ];

    let best = null;
    let bestSolved = null;
    for (const sp of startPoints) {
      this.robot.applyJointState(ARM_JOINT_NAMES,
        ARM_JOINT_NAMES.map(j => sp.joints[j] ?? 0));
      const result = this.ik.solve(target);
      const joints = {};
      ARM_JOINT_NAMES.forEach(j => joints[j] = this.robot.getJoint(j));

      if (result.solved) {
        const solved = { solved: true, error: result.error, joints, start: sp.name };
        if (!preferred) {
          this.robot.applyJointState(ARM_JOINT_NAMES,
            ARM_JOINT_NAMES.map(j => cur[j]));
          return solved;
        }
        solved.distance = _jointDistance(joints, preferred);
        if (!bestSolved || solved.distance < bestSolved.distance) bestSolved = solved;
        continue;
      }
      if (!best || result.error < best.error) {
        best = { solved: false, error: result.error, joints, start: sp.name };
      }
    }
    this.robot.applyJointState(ARM_JOINT_NAMES,
      ARM_JOINT_NAMES.map(j => cur[j]));
    if (bestSolved) return bestSolved;
    return best;
  }

  /** 返回所有IK解 (不同分支), 供 retract 逐个尝试碰撞检测 */
  _multiIKSolutions(target) {
    const cur = {};
    ARM_JOINT_NAMES.forEach(j => cur[j] = this.robot.getJoint(j));
    const armBase = new THREE.Vector3();
    this.robot.jointGroups['joint1'].getWorldPosition(armBase);
    const targetYaw = Math.atan2(target.y - armBase.y, target.x - armBase.x);
    const startPoints = [
      { joints: cur },
      { joints: { joint1: targetYaw, joint2: 0.14, joint3: 1.57, joint4: 1.57, joint5: 0 } },
      { joints: { joint1: targetYaw, joint2: 0.54, joint3: 1.57, joint4: 1.57, joint5: 0 } },
      { joints: { joint1: targetYaw, joint2: -0.8,  joint3: 1.2,  joint4: 1.0,  joint5: 0 } },
      { joints: { joint1: targetYaw, joint2: 0,    joint3: 0,    joint4: 0,    joint5: 0 } },
      { joints: PRESETS.arm_uplift },
      { joints: PRESETS.arm_home },
    ];
    const solutions = [];
    for (const sp of startPoints) {
      this.robot.applyJointState(ARM_JOINT_NAMES,
        ARM_JOINT_NAMES.map(j => sp.joints[j] ?? 0));
      const result = this.ik.solve(target);
      if (result.solved) {
        const joints = {};
        ARM_JOINT_NAMES.forEach(j => joints[j] = this.robot.getJoint(j));
        solutions.push({ error: result.error, joints });
      }
    }
    this.robot.applyJointState(ARM_JOINT_NAMES,
      ARM_JOINT_NAMES.map(j => cur[j]));
    return solutions;
  }

  // ── plan_arm_motion (MoveIt2: 多起始IK + 碰撞检测 + 经由点轨迹 + 自动执行) ──
  async _planArmMotion(p) {
    const target = new THREE.Vector3(p.x, p.y, p.z);
    const approachError = this._validateWorkbenchApproach(target);
    if (approachError) return approachError;
    const cur = {};
    ARM_JOINT_NAMES.forEach(j => cur[j] = this.robot.getJoint(j));
    const tcpNow = this.robot.getGripperTCP();

    // 1. 安全高度 (高于工作台和料箱壁)
    const safeZ = Math.max(tcpNow.z, target.z + 0.06, 0.14);

    // 2. Via2: 目标正上方安全高度 (先求解, 用作 target IK 的 preferred → 同分支解, 下降更垂直)
    const r2 = this._multiStartIK(new THREE.Vector3(target.x, target.y, safeZ), cur);
    const via2Joints = r2.solved ? {} : null;
    if (via2Joints) ARM_JOINT_NAMES.forEach(j => via2Joints[j] = Math.atan2(
      Math.sin(r2.joints[j]), Math.cos(r2.joints[j])));

    // 3. 多起始点 IK 求解 (preferred: via2 → 同分支, 确保下降段垂直, 减少料箱壁碰撞)
    const ikResult = via2Joints
      ? this._multiStartIK(target, via2Joints)
      : this._multiStartIK(target);

    if (!ikResult.solved) {
      this._log(`[MCP] IK失败: 误差${(ikResult.error * 1000).toFixed(1)}mm (from ${ikResult.start})`);
      const suggested = this._suggestChassisFor(target);
      return {
        ok: false, reason: 'ik_unreachable',
        detail: `IK求解失败(多起始点), 最小误差${(ikResult.error * 1000).toFixed(1)}mm (from ${ikResult.start})`,
        error_mm: +(ikResult.error * 1000).toFixed(1),
        suggested_chassis: suggested,
        hint: suggested.no_collision_free
          ? '⚠ 所有站位均无法避免碰撞或不可达, 请更换目标或放弃'
          : `建议先move_base到(${suggested.x.toFixed(2)}, ${suggested.y.toFixed(2)}, yaw=${suggested.yaw.toFixed(2)})再重试`,
      };
    }

    const ikJoints = {};
    ARM_JOINT_NAMES.forEach(j => ikJoints[j] = Math.atan2(
      Math.sin(ikResult.joints[j]), Math.cos(ikResult.joints[j])));
    if (!_jointsWithinLimits(this.robot, ikJoints)) {
      return { ok: false, reason: 'joint_limit', detail: '目标姿态超出URDF关节角度限制' };
    }
    if (_isWorkbenchTarget(target) && Math.abs(ikJoints.joint1 ?? 0) > MAX_FRONT_APPROACH_ANGLE) {
      const suggested = this._suggestChassisFor(target);
      return {
        ok: false,
        reason: 'unsafe_arm_orientation',
        detail: `禁止机械臂底座大角度横摆操作工作台目标: joint1=${ikJoints.joint1.toFixed(2)}rad`,
        suggested_chassis: suggested,
        hint: `请先move_base到(${suggested.x.toFixed(2)}, ${suggested.y.toFixed(2)}, yaw=${suggested.yaw.toFixed(2)})，再重新规划`,
      };
    }

    // 4. 检查目标姿态碰撞
    this.robot.applyJointState(ARM_JOINT_NAMES, ARM_JOINT_NAMES.map(j => ikJoints[j]));
    const goalCollision = _checkArmEnvCollision(this.robot) || this.robot._checkGroundCollision?.();
    this.robot.applyJointState(ARM_JOINT_NAMES, ARM_JOINT_NAMES.map(j => cur[j]));
    if (goalCollision) {
      const suggested = this._suggestChassisFor(target);
      return {
        ok: false, reason: 'goal_collision',
        detail: `目标姿态碰撞: ${goalCollision}`,
        suggested_chassis: suggested,
        hint: suggested.no_collision_free
          ? '⚠ 所有站位均无法避免碰撞, 目标可能与工作台表面过近。请勿继续重试同一目标, 更换目标或放弃'
          : `建议先move_base到(${suggested.x.toFixed(2)}, ${suggested.y.toFixed(2)}, yaw=${suggested.yaw.toFixed(2)})再重试`,
      };
    }

    // 5. Via1: 当前位置抬升到安全高度
    const r1 = this._multiStartIK(new THREE.Vector3(tcpNow.x, tcpNow.y, safeZ), cur);
    const via1Joints = r1.solved ? {} : null;
    if (via1Joints) ARM_JOINT_NAMES.forEach(j => via1Joints[j] = Math.atan2(
      Math.sin(r1.joints[j]), Math.cos(r1.joints[j])));

    const segments = [];
    const xyDist = Math.hypot(target.x - tcpNow.x, target.y - tcpNow.y);
    let segmentStart = cur;

    if (tcpNow.z < safeZ - 0.01) {
      if (!via1Joints) return { ok: false, reason: 'via_ik_failed', detail: '安全抬升点不可达' };
      const c1 = _checkTrajectoryCollision(this.robot, segmentStart, via1Joints, 20);
      if (c1.collision) return { ok: false, reason: 'trajectory_collision', detail: `抬升段碰撞: ${c1.collision}` };
      segments.push({ joints: via1Joints, duration: 1.0, desc: '抬升到安全高度' });
      segmentStart = via1Joints;
    }

    if (xyDist > 0.02) {
      if (!via2Joints) return { ok: false, reason: 'via_ik_failed', detail: '目标上方安全点不可达' };
      const c2 = _checkTrajectoryCollision(this.robot, segmentStart, via2Joints, 24);
      if (c2.collision) return { ok: false, reason: 'trajectory_collision', detail: `水平移动段碰撞: ${c2.collision}` };
      segments.push({ joints: via2Joints, duration: 1.5, desc: '水平移动到目标上方' });
      segmentStart = via2Joints;
    }

    // 下降段碰撞检测: 同时检查夹爪闭合和张开两种状态
    // 抓取后 retract 用闭合, 放置后 retract 用张开 — 两种都必须无碰撞
    // 否则只查一种 → 另一种位姿下 link7 等连杆位置不同可能碰撞 → retract 卡死
    const _savedGrip = {};
    GRIPPER_JOINT_NAMES.forEach(j => _savedGrip[j] = this.robot.getJoint(j));
    let c3Hit = null;
    for (const [label, grip] of [['闭合', GRIPPER_POSES.close], ['张开', GRIPPER_POSES.open]]) {
      this.robot.applyJointState(GRIPPER_JOINT_NAMES,
        GRIPPER_JOINT_NAMES.map(j => grip[j] ?? 0));
      const c = _checkTrajectoryCollision(this.robot, segmentStart, ikJoints, 20);
      if (c.collision) { c3Hit = `夹爪${label}:${c.collision}`; break; }
    }
    this.robot.applyJointState(GRIPPER_JOINT_NAMES,
      GRIPPER_JOINT_NAMES.map(j => _savedGrip[j]));
    if (c3Hit) {
      const suggested = this._suggestChassisFor(target);
      return {
        ok: false,
        reason: 'trajectory_collision',
        detail: `目标接近段碰撞(${c3Hit})`,
        suggested_chassis: suggested,
        hint: suggested.no_collision_free
          ? '⚠ 所有站位均无法避免碰撞, 目标可能与工作台表面过近。请勿继续重试同一目标, 更换目标或放弃'
          : `建议先move_base到(${suggested.x.toFixed(2)}, ${suggested.y.toFixed(2)}, yaw=${suggested.yaw.toFixed(2)})再重试`,
      };
    }
    // 记录"目标正上方安全高度"关节角, 供后续 retract 做逆序垂直抬升
    // (抬升 = 下降的精确逆, 下降已用两种夹爪状态通过碰撞检测 ⇒ 逆序必然无碰撞)
    this._lastAboveJoints = { ...segmentStart };
    segments.push({ joints: ikJoints, duration: xyDist > 0.02 ? 1.0 : 1.5, desc: xyDist > 0.02 ? '下降到目标' : '安全直达目标' });

    // 5. 恢复原始关节, 然后自动执行轨迹 (避免碰撞检测残留导致抽搐)
    this.robot.applyJointState(ARM_JOINT_NAMES, ARM_JOINT_NAMES.map(j => cur[j]));
    const executed = [];
    for (let i = 0; i < segments.length; i++) {
      const seg = segments[i];
      this._log(`[MCP] 执行段 ${i + 1}/${segments.length}: ${seg.desc} (${seg.duration}s)`);
      await this._tweenArmTo(seg.joints, seg.duration);
      const tcp = this.robot.getGripperTCP();
      executed.push({
        segment: i + 1, desc: seg.desc,
        tcp: { x: +tcp.x.toFixed(3), y: +tcp.y.toFixed(3), z: +tcp.z.toFixed(3) },
      });
    }

    const finalTcp = this.robot.getGripperTCP();
    return {
      ok: true, mode: 'moveit2_simulation',
      ik: { solved: true, error_mm: +(ikResult.error * 1000).toFixed(2), start: ikResult.start },
      collision_free: true,
      segments: segments.length,
      segment_descriptions: segments.map(s => s.desc),
      executed: true,
      trajectory: executed,
      tcp_position: { x: +finalTcp.x.toFixed(3), y: +finalTcp.y.toFixed(3), z: +finalTcp.z.toFixed(3) },
    };
  }

  // ── grasp ──
  async _grasp() {
    await this._tweenArmTo({ ...GRIPPER_POSES.close }, 0.8);
    const ok = this.agent._graspNearest();
    const held = this._getHeldObject();
    return {
      ok,
      reason: ok ? 'grasped' : 'no_object_in_range',
      grasped_object: held ? held.class : null,
      hint: ok ? null : '夹爪3.5cm内无可抓取物体。调用 perceive 确认目标坐标, 或 move_base 更换站位靠近目标。',
    };
  }

  // ── release ──
  async _release() {
    const heldBefore = this._getHeldObject();
    await this._tweenArmTo({ ...GRIPPER_POSES.open }, 0.6);
    this.agent._release();
    const tcp = this.robot.getGripperTCP();
    // 检查是否落入某个料箱格子
    let landedSlot = null;
    for (const [slotNum, slotXyz] of Object.entries(this.binSlots)) {
      const d = Math.hypot(tcp.x - slotXyz[0], tcp.y - slotXyz[1]);
      if (d < 0.06) { landedSlot = +slotNum; break; }
    }
    return {
      ok: true,
      released_object: heldBefore ? heldBefore.class : null,
      released_at: { x: +tcp.x.toFixed(3), y: +tcp.y.toFixed(3), z: +tcp.z.toFixed(3) },
      landed_in_slot: landedSlot,
      bin_occupancy: this._getBinOccupancy(),
    };
  }

  // ── retract (收回机械臂: 垂直抬升 + 安全收臂, 从上往下抓取/放置的逆操作) ──
  async _retract(p = {}) {
    const cur = {};
    ARM_JOINT_NAMES.forEach(j => cur[j] = this.robot.getJoint(j));
    const tcpNow = this.robot.getGripperTCP();
    // 安全收回高度: 高于工作台(0.060)与料箱壁(0.113), 确保抬起后连杆不穿模
    const SAFE_RETRACT_Z = 0.16;

    const segments = [];
    let segStart = cur;

    // ── 阶段1a: 逆序抬升 (复用 plan_arm_motion 记录的"目标正上方"关节角) ──
    //   抬升 = 下降的精确逆序, 下降已用闭合夹爪通过碰撞检测 ⇒ 逆序必然无碰撞
    if (this._lastAboveJoints) {
      const above = this._lastAboveJoints;
      const c = _checkTrajectoryCollision(this.robot, cur, above, 20);
      if (!c.collision) {
        segments.push({ joints: above, duration: 0.9, desc: '垂直抬升到安全高度' });
        segStart = above;
      } else {
        this._log(`[MCP] 逆序抬升碰撞, 改用多策略抬升: ${c.collision}`);
      }
      this._lastAboveJoints = null;  // 一次性消费
    }

    // ── 阶段1b: 多解IK垂直抬升 (尝试所有分支解, 选首个无碰撞的) ──
    if (segments.length === 0 && tcpNow.z < SAFE_RETRACT_Z - 0.01) {
      const liftZ = Math.max(tcpNow.z + (p.lift ?? 0.08), SAFE_RETRACT_Z);
      const sols = this._multiIKSolutions(new THREE.Vector3(tcpNow.x, tcpNow.y, liftZ));
      for (const sol of sols) {
        const liftJoints = {};
        ARM_JOINT_NAMES.forEach(j => liftJoints[j] = Math.atan2(
          Math.sin(sol.joints[j]), Math.cos(sol.joints[j])));
        if (!_jointsWithinLimits(this.robot, liftJoints)) continue;
        const c = _checkTrajectoryCollision(this.robot, cur, liftJoints, 20);
        if (!c.collision) {
          segments.push({ joints: liftJoints, duration: 0.9, desc: '垂直抬升到安全高度' });
          segStart = liftJoints;
          break;
        }
      }
    }

    // ── 阶段1c: 后退脱离 + 抬升 (臂伸出在工作台上方时, 先向臂基座方向后退再抬起) ──
    if (segments.length === 0) {
      const armBase = new THREE.Vector3();
      this.robot.jointGroups['joint1'].getWorldPosition(armBase);
      const dx = armBase.x - tcpNow.x;
      const dy = armBase.y - tcpNow.y;
      const dist = Math.hypot(dx, dy);
      if (dist > 0.02) {
        const step = Math.min(0.06, dist * 0.3);
        const rx = tcpNow.x + (dx / dist) * step;
        const ry = tcpNow.y + (dy / dist) * step;
        // 后退 (同高度)
        const rs = this._multiIKSolutions(new THREE.Vector3(rx, ry, tcpNow.z));
        for (const sol of rs) {
          const rj = {};
          ARM_JOINT_NAMES.forEach(j => rj[j] = Math.atan2(
            Math.sin(sol.joints[j]), Math.cos(sol.joints[j])));
          if (!_jointsWithinLimits(this.robot, rj)) continue;
          const c = _checkTrajectoryCollision(this.robot, cur, rj, 16);
          if (!c.collision) {
            segments.push({ joints: rj, duration: 0.6, desc: '后退脱离障碍' });
            // 从后退位置垂直抬升
            const liftZ = Math.max(tcpNow.z + 0.08, SAFE_RETRACT_Z);
            const ls = this._multiIKSolutions(new THREE.Vector3(rx, ry, liftZ));
            for (const sol2 of ls) {
              const lj = {};
              ARM_JOINT_NAMES.forEach(j => lj[j] = Math.atan2(
                Math.sin(sol2.joints[j]), Math.cos(sol2.joints[j])));
              if (!_jointsWithinLimits(this.robot, lj)) continue;
              const c2 = _checkTrajectoryCollision(this.robot, rj, lj, 20);
              if (!c2.collision) {
                segments.push({ joints: lj, duration: 0.8, desc: '垂直抬升到安全高度' });
                segStart = lj;
                break;
              }
            }
            break;
          }
        }
      }
    }

    // ── 阶段2: 过渡到安全收臂姿态 (非致命: 碰撞则跳过, 臂已在安全高度可移动底盘) ──
    const retractPose = {
      joint1: cur['joint1'],
      joint2: PRESETS.arm_uplift.joint2,
      joint3: PRESETS.arm_uplift.joint3,
      joint4: PRESETS.arm_uplift.joint4,
      joint5: 0,
    };
    const c2 = _checkTrajectoryCollision(this.robot, segStart, retractPose, 20);
    if (!c2.collision) {
      segments.push({ joints: retractPose, duration: 0.9, desc: '收回至安全姿态' });
    }

    if (segments.length === 0) {
      return { ok: false, reason: 'trajectory_collision', detail: '收回轨迹碰撞 (所有抬升策略均失败)' };
    }

    this._log(`[MCP] 收回机械臂到安全姿态 (保持方向 joint1=${cur['joint1'].toFixed(2)})`);
    for (let i = 0; i < segments.length; i++) {
      const seg = segments[i];
      this._log(`[MCP] 执行段 ${i + 1}/${segments.length}: ${seg.desc} (${seg.duration}s)`);
      await this._tweenArmTo(seg.joints, seg.duration);
    }
    const tcp = this.robot.getGripperTCP();
    return {
      ok: true,
      action: 'retract',
      pose: 'uplift (direction preserved)',
      tcp: { x: +tcp.x.toFixed(3), y: +tcp.y.toFixed(3), z: +tcp.z.toFixed(3) },
    };
  }

  // ── verify (直接检查场景状态, 不依赖 MockAgent._target/_slotXyz) ──
  _verify(p) {
    if (p.type === 'grasp') {
      const held = this._getHeldObject();
      return {
        ok: !!held,
        reason: held ? `grasped: ${held.class}` : 'no_object_grasped',
        held_object: held ? held.class : null,
      };
    }
    if (p.type === 'place') {
      const slotNum = p.slot;
      const slotXyz = this.binSlots[slotNum] || this.binSlots[String(slotNum)];
      if (!slotXyz) return { ok: false, reason: 'invalid_slot', detail: `料箱${slotNum}不存在` };
      const objects = [];
      if (this.agent?.tools) {
        for (const t of this.agent.tools) {
          if (t.grasped) continue; // 跳过正在夹持的
          const d = Math.hypot(t.currentXyz[0] - slotXyz[0], t.currentXyz[1] - slotXyz[1]);
          if (d < 0.06) objects.push(t.class);
        }
      }
      return {
        ok: objects.length > 0,
        reason: objects.length > 0 ? 'objects_in_slot' : 'slot_empty',
        slot: slotNum,
        objects_in_slot: objects,
        count: objects.length,
      };
    }
    return { ok: false, reason: 'unknown_verify_type', detail: `type必须是grasp或place` };
  }

  // ── get_robot_state ──
  _getRobotState() {
    const pos = this.robot.root.position;
    const joints = {};
    ARM_JOINT_NAMES.forEach(j => joints[j] = +this.robot.getJoint(j).toFixed(3));
    const tcp = this.robot.getGripperTCP();
    const joint6 = this.robot.getJoint('joint6');
    const held = this._getHeldObject();
    return {
      ok: true,
      chassis: { x: +pos.x.toFixed(3), y: +pos.y.toFixed(3), yaw: +this.robot.root.rotation.z.toFixed(3) },
      arm_joints: joints,
      gripper: joint6 > 0 ? 'open' : 'closed',
      gripper_value: +joint6.toFixed(3),
      holding: held ? held.class : null,
      tcp: { x: +tcp.x.toFixed(3), y: +tcp.y.toFixed(3), z: +tcp.z.toFixed(3) },
    };
  }

  // ── get_scene_info ──
  _getSceneInfo() {
    return {
      ok: true,
      arm_base_offset: { x: ARM_BASE_OFFSET.x, y: ARM_BASE_OFFSET.y, z: ARM_BASE_OFFSET.z },
      arm_reach_m: ARM_REACH,
      workbenches: {
        bin_bench: { center: [0.30, -0.30], y_range: [-0.49, -0.11], x_range: [0.15, 0.45], z_range: [0.04, 0.045] },
        tool_bench: { center: [0.30, 0.30], y_range: [0.11, 0.49], x_range: [0.15, 0.45], z_range: [0.04, 0.045] },
        mobile_area: { x_range: [MOBILE_AREA.xMin, MOBILE_AREA.xMax], y_range: [MOBILE_AREA.yMin, MOBILE_AREA.yMax], note: '底盘可在覆盖双工作区的大作业区内自由移动, 路径规划会按底盘footprint避开工作台' },
      },
      chassis_safety: { radius_m: CHASSIS_RADIUS, clearance_m: CHASSIS_CLEARANCE },
      collision_boxes: COLLISION_BOXES.map(b => ({
        name: b.name, x: b.x, y: b.y, z: b.z,
      })),
      bin_slots: Object.fromEntries(
        Object.entries(this.binSlots).map(([k, v]) => [k, { x: v[0], y: v[1], z: v[2] }])
      ),
      bench_top_z: 0.060,
      safe_height_z: 0.14,
      bin_occupancy: this._getBinOccupancy(),
      planning_note: '底盘站位按最近可达点规划, 并按底盘半径+安全间隙避开工作台; 臂移动时不能穿入工作台碰撞盒, 必要时经安全高度绕行。',
    };
  }
}
