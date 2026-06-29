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
import { PRESETS, GRIPPER_POSES, ARM_JOINT_NAMES } from './RobotModel.js';

// ─────────────── 环境碰撞模型 (MoveIt2 Planning Scene 仿真) ───────────────

// 工作台碰撞盒 (AABB): 臂连杆不能穿入这些区域
const COLLISION_BOXES = [
  // 左台 (料箱): 台面 + 料箱壁
  { name: 'bin_bench', x: [0.15, 0.45], y: [-0.49, -0.11], z: [0.040, 0.115] },
  // 右台 (工具): 台面
  { name: 'tool_bench', x: [0.15, 0.45], y: [0.11, 0.49], z: [0.040, 0.115] },
];

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
  const links = ['link1', 'link2', 'link3', 'link4', 'link5'];
  const p = new THREE.Vector3();
  for (const name of links) {
    const g = robot.jointGroups[name];
    if (!g) continue;
    g.getWorldPosition(p);
    const hit = _pointInCollision(p.x, p.y, p.z);
    if (hit) return `${name}↔${hit}`;
  }
  return null;
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
    // 记录最高 TCP Z (用于安全高度)
    const tcp = robot.getGripperTCP();
    if (tcp.z > maxZ) maxZ = tcp.z;
  }
  // 恢复关节状态和碰撞设置
  robot.applyJointState(names, saved);
  robot.collisionEnabled = wasEnabled;
  return { collision: collisionFound, safeHeight: maxZ };
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
      description: '闭合夹爪, 抓取距离TCP最近的工具(12cm内)。返回是否抓取成功。',
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
      description: '收回机械臂到安全姿态(抬升joint2-5, 保持当前方向)。总是成功, 不依赖IK。用于抓取/放置后收回, 避免移动底盘时碰撞。',
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
   * @param {object} ctx — { mockAgent, robot, ik, chassis, binSlots }
   */
  constructor(ctx) {
    this.agent = ctx.mockAgent;
    this.robot = ctx.robot;
    this.ik = ctx.ik;
    this.chassis = ctx.chassis;
    this.binSlots = ctx.binSlots || {};
    this._plannedTrajectory = null;  // plan_arm_motion → execute_arm_motion
    this._logCb = null;
  }

  onLog(cb) { this._logCb = cb; }
  _log(msg) { if (this._logCb) this._logCb(msg); }

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
      this._log(`[MCP] ${name} → ${r.ok ? 'ok' : 'fail'}${r.reason ? ': ' + r.reason : ''}`);
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
    const armX = this.robot.root.position.x + 0.054;
    const armY = this.robot.root.position.y + 0.001;
    const armZ = 0.156;
    return {
      ok: true,
      objects: objs.map(o => {
        const dist = Math.hypot(o.xyz[0] - armX, o.xyz[1] - armY, o.xyz[2] - armZ);
        const reachable = dist <= 0.40;
        const obj = {
          class: o.class,
          position: { x: +o.xyz[0].toFixed(3), y: +o.xyz[1].toFixed(3), z: +o.xyz[2].toFixed(3) },
          confidence: o.conf,
          distance_m: +dist.toFixed(3),
          reachable,
        };
        if (!reachable) {
          const sx = o.xyz[0] - 0.054;
          const sy = Math.max(-0.08, Math.min(0.08, o.xyz[1] * 0.25));
          obj.suggested_chassis = { x: +sx.toFixed(3), y: +sy.toFixed(3), yaw: 0 };
        }
        return obj;
      }),
      count: objs.length,
      arm_base: { x: +armX.toFixed(3), y: +armY.toFixed(3), z: +armZ.toFixed(3) },
      arm_reach_m: 0.40,
    };
  }

  // ── move_base ──
  async _moveBase(p) {
    await this.agent._moveChassisTo(p.x, p.y, p.yaw || 0, 2.0);
    const pos = this.robot.root.position;
    return {
      ok: true,
      chassis_position: { x: +pos.x.toFixed(3), y: +pos.y.toFixed(3), yaw: +this.robot.root.rotation.z.toFixed(3) },
    };
  }

  // ── 多起始点IK求解 (预旋转joint1面朝目标 + 多planar起始姿态) ──
  _multiStartIK(target) {
    const cur = {};
    ARM_JOINT_NAMES.forEach(j => cur[j] = this.robot.getJoint(j));

    // 计算臂基座世界坐标 → 目标方向的yaw角
    const armBase = new THREE.Vector3();
    this.robot.jointGroups['joint1'].getWorldPosition(armBase);
    const targetYaw = Math.atan2(target.y - armBase.y, target.x - armBase.x);

    // 起始姿态: joint1预旋转到目标方向 + 不同planar初始姿态
    // CCD只需求解planar问题 (joint2-4), 不再需要同时旋转joint1, 大幅提高收敛率
    const startPoints = [
      { name: 'current', joints: cur },
      { name: 'pre_rot_up',    joints: { joint1: targetYaw, joint2: 0.54, joint3: 1.57, joint4: 1.57, joint5: 0 } },
      { name: 'pre_rot_reach', joints: { joint1: targetYaw, joint2: -0.8,  joint3: 1.2,  joint4: 1.0,  joint5: 0 } },
      { name: 'pre_rot_home',  joints: { joint1: targetYaw, joint2: 0,    joint3: 0,    joint4: 0,    joint5: 0 } },
      { name: 'arm_uplift',    joints: PRESETS.arm_uplift },
      { name: 'arm_home',      joints: PRESETS.arm_home },
    ];

    let best = null;
    for (const sp of startPoints) {
      this.robot.applyJointState(ARM_JOINT_NAMES,
        ARM_JOINT_NAMES.map(j => sp.joints[j] ?? 0));
      const result = this.ik.solve(target);
      const joints = {};
      ARM_JOINT_NAMES.forEach(j => joints[j] = this.robot.getJoint(j));

      if (result.solved) {
        this.robot.applyJointState(ARM_JOINT_NAMES,
          ARM_JOINT_NAMES.map(j => cur[j]));
        return { solved: true, error: result.error, joints, start: sp.name };
      }
      if (!best || result.error < best.error) {
        best = { solved: false, error: result.error, joints, start: sp.name };
      }
    }
    this.robot.applyJointState(ARM_JOINT_NAMES,
      ARM_JOINT_NAMES.map(j => cur[j]));
    return best;
  }

  // ── plan_arm_motion (MoveIt2: 多起始IK + 碰撞检测 + 经由点轨迹 + 自动执行) ──
  async _planArmMotion(p) {
    const target = new THREE.Vector3(p.x, p.y, p.z);
    const cur = {};
    ARM_JOINT_NAMES.forEach(j => cur[j] = this.robot.getJoint(j));
    const tcpNow = this.robot.getGripperTCP();

    // 1. 多起始点 IK 求解
    const ikResult = this._multiStartIK(target);

    if (!ikResult.solved) {
      this._log(`[MCP] IK失败: 误差${(ikResult.error * 1000).toFixed(1)}mm (from ${ikResult.start})`);
      const sx = target.x - 0.054;
      const sy = Math.max(-0.08, Math.min(0.08, target.y * 0.25));
      return {
        ok: false, reason: 'ik_unreachable',
        detail: `IK求解失败(多起始点), 最小误差${(ikResult.error * 1000).toFixed(1)}mm (from ${ikResult.start})`,
        error_mm: +(ikResult.error * 1000).toFixed(1),
        suggested_chassis: { x: +sx.toFixed(3), y: +sy.toFixed(3), yaw: 0 },
        hint: `建议先move_base到(${sx.toFixed(2)}, ${sy.toFixed(2)})再重试`,
      };
    }

    const ikJoints = {};
    ARM_JOINT_NAMES.forEach(j => ikJoints[j] = Math.atan2(
      Math.sin(ikResult.joints[j]), Math.cos(ikResult.joints[j])));

    // 2. 检查目标姿态碰撞
    this.robot.applyJointState(ARM_JOINT_NAMES, ARM_JOINT_NAMES.map(j => ikJoints[j]));
    const goalCollision = _checkArmEnvCollision(this.robot);
    this.robot.applyJointState(ARM_JOINT_NAMES, ARM_JOINT_NAMES.map(j => cur[j]));
    if (goalCollision) {
      return { ok: false, reason: 'goal_collision', detail: `目标姿态碰撞: ${goalCollision}` };
    }

    // 3. 始终使用经由点轨迹 (抬起→水平→下降), 运动更自然可见
    const safeZ = Math.max(tcpNow.z, target.z + 0.06, 0.14);

    // Via1: 当前位置抬升到安全高度
    const r1 = this._multiStartIK(new THREE.Vector3(tcpNow.x, tcpNow.y, safeZ));
    const via1Joints = r1.solved ? {} : null;
    if (via1Joints) ARM_JOINT_NAMES.forEach(j => via1Joints[j] = Math.atan2(
      Math.sin(r1.joints[j]), Math.cos(r1.joints[j])));

    // Via2: 目标正上方安全高度
    const r2 = this._multiStartIK(new THREE.Vector3(target.x, target.y, safeZ));
    const via2Joints = r2.solved ? {} : null;
    if (via2Joints) ARM_JOINT_NAMES.forEach(j => via2Joints[j] = Math.atan2(
      Math.sin(r2.joints[j]), Math.cos(r2.joints[j])));

    let segments = [];

    // Seg1: 抬升 (仅当当前高度低于安全高度)
    if (via1Joints && tcpNow.z < safeZ - 0.01) {
      const c1 = _checkTrajectoryCollision(this.robot, cur, via1Joints, 15);
      if (!c1.collision) segments.push({ joints: via1Joints, duration: 1.0, desc: '抬升到安全高度' });
    }

    // Seg2: 水平移动 (仅当XY距离 > 2cm)
    const xyDist = Math.hypot(target.x - tcpNow.x, target.y - tcpNow.y);
    if (via1Joints && via2Joints && xyDist > 0.02) {
      this.robot.applyJointState(ARM_JOINT_NAMES, ARM_JOINT_NAMES.map(j => via1Joints[j]));
      const c2 = _checkTrajectoryCollision(this.robot, via1Joints, via2Joints, 20);
      this.robot.applyJointState(ARM_JOINT_NAMES, ARM_JOINT_NAMES.map(j => cur[j]));
      if (!c2.collision) segments.push({ joints: via2Joints, duration: 1.5, desc: '水平移动到目标上方' });
    }

    // Seg3: 下降到目标
    if (via2Joints) {
      const c3 = _checkTrajectoryCollision(this.robot, via2Joints, ikJoints, 15);
      if (!c3.collision) segments.push({ joints: ikJoints, duration: 1.0, desc: '下降到目标' });
    }

    // 经由点全部失败 → 直接插值
    if (segments.length === 0) {
      segments = [{ joints: ikJoints, duration: 2.0, desc: '直达目标' }];
    }

    // 5. 恢复原始关节, 然后自动执行轨迹 (避免碰撞检测残留导致抽搐)
    this.robot.applyJointState(ARM_JOINT_NAMES, ARM_JOINT_NAMES.map(j => cur[j]));
    const executed = [];
    for (let i = 0; i < segments.length; i++) {
      const seg = segments[i];
      this._log(`[MCP] 执行段 ${i + 1}/${segments.length}: ${seg.desc} (${seg.duration}s)`);
      await new Promise(resolve => {
        this.robot.tweenTo(seg.joints, seg.duration, () => resolve());
      });
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
    await new Promise(resolve => {
      this.robot.tweenTo({ ...GRIPPER_POSES.close }, 0.8, () => resolve());
    });
    const ok = this.agent._graspNearest();
    return { ok, reason: ok ? 'grasped' : 'no_object_in_range' };
  }

  // ── release ──
  async _release() {
    await new Promise(resolve => {
      this.robot.tweenTo({ ...GRIPPER_POSES.open }, 0.6, () => resolve());
    });
    this.agent._release();
    const tcp = this.robot.getGripperTCP();
    return { ok: true, released_at: { x: +tcp.x.toFixed(3), y: +tcp.y.toFixed(3), z: +tcp.z.toFixed(3) } };
  }

  // ── retract (收回机械臂到安全姿态, 保持当前方向, 不依赖IK) ──
  async _retract(p = {}) {
    const cur = {};
    ARM_JOINT_NAMES.forEach(j => cur[j] = this.robot.getJoint(j));
    // 保持当前joint1方向, 只抬升joint2-5到安全姿态 (避免不必要旋转)
    const retractPose = {
      joint1: cur['joint1'],
      joint2: PRESETS.arm_uplift.joint2,
      joint3: PRESETS.arm_uplift.joint3,
      joint4: PRESETS.arm_uplift.joint4,
      joint5: 0,
    };
    this._log(`[MCP] 收回机械臂到安全姿态 (保持方向 joint1=${cur['joint1'].toFixed(2)})`);
    await new Promise(resolve => {
      this.robot.tweenTo(retractPose, 1.0, () => resolve());
    });
    const tcp = this.robot.getGripperTCP();
    return {
      ok: true,
      action: 'retract',
      pose: 'uplift (direction preserved)',
      tcp: { x: +tcp.x.toFixed(3), y: +tcp.y.toFixed(3), z: +tcp.z.toFixed(3) },
    };
  }

  // ── verify ──
  _verify(p) {
    if (p.type === 'grasp') {
      const r = this.agent._verifyGrasp();
      return r;
    }
    if (p.type === 'place') {
      const r = this.agent._verifyPlace();
      return r;
    }
    return { ok: false, reason: 'unknown_verify_type' };
  }

  // ── get_robot_state ──
  _getRobotState() {
    const pos = this.robot.root.position;
    const joints = {};
    ARM_JOINT_NAMES.forEach(j => joints[j] = +this.robot.getJoint(j).toFixed(3));
    const tcp = this.robot.getGripperTCP();
    const joint6 = this.robot.getJoint('joint6');
    return {
      ok: true,
      chassis: { x: +pos.x.toFixed(3), y: +pos.y.toFixed(3), yaw: +this.robot.root.rotation.z.toFixed(3) },
      arm_joints: joints,
      gripper: joint6 > 0 ? 'open' : 'closed',
      gripper_value: +joint6.toFixed(3),
      tcp: { x: +tcp.x.toFixed(3), y: +tcp.y.toFixed(3), z: +tcp.z.toFixed(3) },
    };
  }

  // ── get_scene_info ──
  _getSceneInfo() {
    return {
      ok: true,
      arm_base_offset: { x: 0.054, y: 0.001, z: 0.156 },
      arm_reach_m: 0.40,
      workbenches: {
        bin_bench: { center: [0.30, -0.30], y_range: [-0.49, -0.11], x_range: [0.15, 0.45], z_range: [0.04, 0.115] },
        tool_bench: { center: [0.30, 0.30], y_range: [0.11, 0.49], x_range: [0.15, 0.45], z_range: [0.04, 0.115] },
        corridor: { y_range: [-0.11, 0.11], width_m: 0.22 },
      },
      collision_boxes: COLLISION_BOXES.map(b => ({
        name: b.name, x: b.x, y: b.y, z: b.z,
      })),
      bin_slots: Object.fromEntries(
        Object.entries(this.binSlots).map(([k, v]) => [k, { x: v[0], y: v[1], z: v[2] }])
      ),
      bench_top_z: 0.060,
      safe_height_z: 0.14,
      planning_note: '臂移动时不能穿入工作台碰撞盒。规划会自动检测路径碰撞, 必要时经安全高度绕行。',
    };
  }
}
