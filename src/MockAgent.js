/**
 * MockAgent.js — 浏览器内智能体 (真实 RGB-D + YOLO 感知)
 *
 * MockAgent 作为 MCPToolExecutor 的物理后端, 提供:
 *   _perceive / _moveChassisTo / _moveToPose / _graspNearest / _release
 * 场景管理: resetScene
 * 物体来源: 真实 YOLO+RGB-D 检测 (setRealAnnotations → _syncRealTools)
 *
 * 用法 (main.js):
 *   const mockAgent = new MockAgent(robot, ik, scene, agentPanel, ros, chassis);
 *   agentPanel.setMockAgent(mockAgent);
 *   mockAgent.setLLMAgent(llmAgent);
 *   // 渲染循环里: mockAgent.update();
 */
import * as THREE from 'three';
import { PRESETS, GRIPPER_POSES, ARM_JOINT_NAMES, GRIPPER_JOINT_NAMES, WHEEL_JOINT_NAMES } from './RobotModel.js';
import {
  createToolMesh, TOOL_CN,
} from './SceneLayouts.js';

const GRASP_ATTACH_DISTANCE = 0.035;

// 料箱3格×12cm, 在左工作台上 (Y center=-0.30)
// 旋转180°: slot1靠近过道(机械臂侧), slot3远离
// Z = 台面顶(0.060) + 底板(0.003) + 余量(0.010) = 0.073
const BIN_SLOTS = {
  1: [0.30, -0.18, 0.073], 2: [0.30, -0.30, 0.073],
  3: [0.30, -0.42, 0.073],
};

// ─────────────── MockAgent ───────────────
export class MockAgent {
  constructor(robot, ik, sceneSetup, panel, ros = null, chassis = null) {
    this.robot = robot;
    this.ik = ik;
    this.scene = sceneSetup.scene;
    this.panel = panel;
    this.ros = ros;
    this.chassis = chassis;
    this._busy = false;
    this._injectFailure = false;
    this._benchTopZ = 0.060;     // 台面顶高度 (碰撞检测参考)
    this._binSlots = BIN_SLOTS;  // 料箱格子坐标 (供 MCPToolExecutor 引用)
    this._realAnnotations = null;
    this._realAnnotationsTime = 0;
    this._realAnnotationsTimeout = 5000;

    // 工具状态 (含 3D mesh), 由真实 YOLO+RGB-D 检测结果同步填充
    this.tools = [];

    // 场景中的工具 3D 对象 (mesh + label)
    this._toolObjects = [];

    this._lastAboveJoints = null;
    this._lastAboveGripper = null;

    this.robot.collisionEnabled = false;
    this._lastCmdVelSent = 0;

    // 构建工业环境 (工作台 + 料箱, 可见 3D 网格)
    this._buildEnvironment();

    // 初始化臂到安全抬起姿态
    this.robot.applyJointState(ARM_JOINT_NAMES,
      ARM_JOINT_NAMES.map(j => PRESETS.arm_uplift[j]));
  }

  // ─── 工业环境 3D 网格 (工作台 + 料箱) ───
  _buildEnvironment() {
    const benchTopZ = 0.060;
    const benchCx = 0.30, benchW = 0.15, benchD = 0.38, benchT = 0.020;
    const benchMat = new THREE.MeshStandardMaterial({ color: 0x3A3A44, metalness: 0.6, roughness: 0.35 });
    const legMat = new THREE.MeshStandardMaterial({ color: 0x2A2A30, metalness: 0.5, roughness: 0.4 });
    const legH = benchTopZ - benchT;
    const binMat = new THREE.MeshStandardMaterial({ color: 0x4A4A55, metalness: 0.4, roughness: 0.5, transparent: true, opacity: 0.85 });

    const buildBench = (cy) => {
      const top = new THREE.Mesh(new THREE.BoxGeometry(benchW, benchD, benchT), benchMat);
      top.position.set(benchCx, cy, benchTopZ - benchT / 2);
      top.receiveShadow = true;
      this.scene.add(top);
      for (const [lx, ly] of [[-1,-1],[-1,1],[1,-1],[1,1]]) {
        const leg = new THREE.Mesh(new THREE.BoxGeometry(0.03, 0.03, legH), legMat);
        leg.position.set(benchCx + lx*(benchW/2-0.03), cy + ly*(benchD/2-0.03), legH/2);
        leg.castShadow = true;
        this.scene.add(leg);
      }
    };

    // 左台 (料箱台) + 右台 (工具台)
    buildBench(-0.30);
    buildBench(0.30);

    // 料箱 (左台上: 3 格, 隔板 + 侧壁)
    const binY = -0.30, binW = 0.10, binH = 0.053, binT = 0.002;
    const wallH = benchTopZ + binH;
    for (const wy of [-0.48, -0.36, -0.24, -0.12]) {
      const wall = new THREE.Mesh(new THREE.BoxGeometry(binW, binT*2, binH), binMat);
      wall.position.set(benchCx, wy, wallH - binH/2);
      wall.castShadow = true;
      this.scene.add(wall);
    }
    for (const wx of [benchCx - binW/2, benchCx + binW/2]) {
      const wall = new THREE.Mesh(new THREE.BoxGeometry(binT*2, 0.36, binH), binMat);
      wall.position.set(wx, binY, wallH - binH/2);
      wall.castShadow = true;
      this.scene.add(wall);
    }
    // 料箱底板
    const floor = new THREE.Mesh(new THREE.BoxGeometry(binW, 0.36, 0.002), binMat);
    floor.position.set(benchCx, binY, benchTopZ + 0.001);
    floor.receiveShadow = true;
    this.scene.add(floor);
  }

  /** 重置场景 (清空检测物体, 机械臂归零, 夹爪张开, 底盘停止) */
  resetScene() {
    for (const t of this.tools) this._removeToolFromScene(t);
    this.tools = [];
    if (this.chassis && this.chassis._navActive) this.chassis.cancelNav();
    if (this.chassis) { this.chassis.v = 0; this.chassis.w = 0; }
    const resetPose = { ...PRESETS.arm_home, ...GRIPPER_POSES.open };
    this.robot.tweenTo(resetPose, 1.0);
    this._sendArmCommand(resetPose);
    if (this.chassis && this.ros?.publishEnabled && this.ros?.connected) this.ros.sendCmdVel(0, 0);
    this._log('[reset] 场景已重置, 机械臂归零, 夹爪张开');
    this._setStatus('done', '场景重置');
  }

  /** 生成模拟测试工具 (无需真实相机即可测试全链路) */
  spawnTestTools() {
    // 清空现有工具
    for (const t of this.tools) this._removeToolFromScene(t);
    this.tools = [];

    // 5种工业工具, 摆放在右台 (工具台)
    const benchTopZ = this._benchTopZ;
    const testDefs = [
      { class: 'screwdriver', xyz: [0.26, 0.20, benchTopZ + 0.008], rot: 0.3 },
      { class: 'wrench',      xyz: [0.30, 0.25, benchTopZ + 0.002], rot: -0.5 },
      { class: 'nut',         xyz: [0.34, 0.20, benchTopZ + 0.007], rot: 0 },
      { class: 'roller',      xyz: [0.28, 0.35, benchTopZ + 0.009], rot: 1.2 },
      { class: 'screw',       xyz: [0.32, 0.38, benchTopZ + 0.004], rot: 0.8 },
    ];

    for (const def of testDefs) {
      const mesh = createToolMesh(def.class);
      mesh.position.set(...def.xyz);
      mesh.rotation.z = def.rot || 0;
      mesh.traverse(c => { if (c.isMesh) { c.castShadow = true; } });
      this.scene.add(mesh);
      this._toolObjects.push(mesh);

      const label = this._createLabel(TOOL_CN[def.class] || def.class, '#FFCC44');
      label.position.set(def.xyz[0], def.xyz[1], def.xyz[2] + 0.035);
      label.scale.set(0.040, 0.009, 1);
      this.scene.add(label);
      this._toolObjects.push(label);

      this.tools.push({
        class: def.class,
        class_cn: TOOL_CN[def.class] || def.class,
        xyz: [...def.xyz],
        rot: def.rot,
        mesh, label,
        grasped: false,
        currentXyz: [...def.xyz],
        real: false,
        confidence: 0.95,
      });
    }
    this._log(`[test] 已生成 ${testDefs.length} 个模拟工具: ${testDefs.map(d=>TOOL_CN[d.class]).join(', ')}`);
    this._setStatus('done', '模拟工具已生成');
  }

  _createLabel(text, color = '#FFCC44') {
    const cv = document.createElement('canvas');
    cv.width = 256; cv.height = 48;
    const ctx = cv.getContext('2d');
    ctx.fillStyle = 'rgba(0,0,0,0.55)';
    ctx.fillRect(0, 0, 256, 48);
    ctx.font = 'bold 26px sans-serif';
    ctx.fillStyle = color;
    ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
    ctx.fillText(text, 128, 24);
    const tex = new THREE.CanvasTexture(cv);
    const sp = new THREE.Sprite(new THREE.SpriteMaterial({ map: tex, transparent: true }));
    sp.scale.set(0.06, 0.012, 1);
    return sp;
  }

  // ─── 每帧更新 (渲染循环调用) ───
  update() {
    for (const t of this.tools) {
      if (t.grasped && t.mesh) {
        const tcp = this.robot.getGripperTCP();
        t.mesh.position.copy(tcp);
        t.currentXyz = [tcp.x, tcp.y, tcp.z];
        if (t.label) t.label.visible = false;
      } else {
        if (t.label) {
          t.label.visible = true;
          t.label.position.set(
            t.currentXyz[0], t.currentXyz[1], t.currentXyz[2] + 0.035);
        }
      }
    }
  }

  // ─── 感知 ───
  // 真实 YOLO+RGB-D 检测结果注入 (由 AgentPanel 从 /industrial_tools/annotations 接收后调用)
  // 将真实检测到的物体同步为 3D Mesh 放入场景 + this.tools,
  // 使 grasp/release/verify 全链路打通 (sim2real 闭环)
  setRealAnnotations(objects) {
    this._realAnnotations = objects || [];
    this._realAnnotationsTime = performance.now();
    this._syncRealTools(this._realAnnotations);
  }

  // 将真实检测结果同步到 this.tools 和 3D 场景
  _syncRealTools(detections) {
    const kept = [];
    for (const t of this.tools) {
      if (t.grasped) { kept.push(t); }
      else { this._removeToolFromScene(t); }
    }
    this.tools = kept;
    for (const obj of detections) {
      if (!obj || !obj.position_world_m) continue;
      this._addRealToolToScene(obj);
    }
  }

  _removeToolFromScene(t) {
    if (t.mesh) {
      this.scene.remove(t.mesh);
      t.mesh.traverse(c => {
        if (c.isMesh) { c.geometry?.dispose?.(); c.material?.dispose?.(); }
      });
      const i = this._toolObjects.indexOf(t.mesh);
      if (i >= 0) this._toolObjects.splice(i, 1);
    }
    if (t.label) {
      this.scene.remove(t.label);
      if (t.label.material?.map) t.label.material.map.dispose();
      t.label.material?.dispose();
      const i = this._toolObjects.indexOf(t.label);
      if (i >= 0) this._toolObjects.splice(i, 1);
    }
  }

  _addRealToolToScene(obj) {
    const xyz = [...obj.position_world_m];
    const cls = obj.class || 'unknown';
    const clsCn = obj.class_cn || TOOL_CN[cls] || cls;

    const KNOWN_TOOLS = ['screwdriver', 'wrench', 'nut', 'roller', 'screw'];
    let mesh;
    if (KNOWN_TOOLS.includes(cls)) {
      mesh = createToolMesh(cls);
    } else {
      mesh = this._createGenericObjectMesh(cls);
    }
    mesh.position.set(...xyz);
    mesh.traverse(c => { if (c.isMesh) { c.castShadow = true; } });
    this.scene.add(mesh);
    this._toolObjects.push(mesh);

    const label = this._createLabel(clsCn, '#44FF88');
    label.position.set(xyz[0], xyz[1], xyz[2] + 0.035);
    label.scale.set(0.040, 0.009, 1);
    this.scene.add(label);
    this._toolObjects.push(label);

    this.tools.push({
      class: cls,
      class_cn: clsCn,
      xyz: [...xyz],
      rot: 0,
      mesh,
      label,
      grasped: false,
      currentXyz: [...xyz],
      real: true,
      confidence: obj.confidence ?? 0.5,
    });
  }

  _createGenericObjectMesh(className) {
    const g = new THREE.Group();
    let hash = 0;
    for (let i = 0; i < className.length; i++) {
      hash = ((hash << 5) - hash + className.charCodeAt(i)) | 0;
    }
    const hue = Math.abs(hash) % 360;
    const color = new THREE.Color().setHSL(hue / 360, 0.6, 0.5);
    const body = new THREE.Mesh(
      new THREE.BoxGeometry(0.025, 0.025, 0.025),
      new THREE.MeshStandardMaterial({ color, metalness: 0.3, roughness: 0.5 })
    );
    body.castShadow = true;
    g.add(body);
    g.scale.setScalar(0.5);
    return g;
  }

  _perceive() {
    return this.tools
      .filter(t => !t.grasped)
      .map(t => ({
        class: t.class,
        xyz: [...t.currentXyz],
        conf: t.confidence ?? 0.5,
        real: !!t.real,
      }));
  }

  // 工作台占地范围 (底盘不能进入)
  static BENCH_FOOTPRINTS = [
    { xMin: 0.225, xMax: 0.375, yMin: -0.49, yMax: -0.11 }, // 左台(料箱)
    { xMin: 0.225, xMax: 0.375, yMin: 0.11, yMax: 0.49 },   // 右台(工具)
  ];
  // 底盘可移动大作业区 (覆盖双工作台和中间区域, 但不能压到工作台本体)
  static MOBILE_AREA = { xMin: -0.25, xMax: 0.70, yMin: -0.62, yMax: 0.62 };
  static CHASSIS_RADIUS = 0.13;
  static CHASSIS_CLEARANCE = 0.03;
  static ARM_COMFORT_DIST = 0.28;
  static ARM_COLLISION_LINKS = ['link1', 'link2', 'link3', 'link4', 'link5', 'link6', 'link7', 'link8', 'link9', 'link10', 'link11'];
  static ARM_COLLISION_BOXES = [
    // 台面实心 (碰撞盒 z 顶 = 0.045, 低于视觉台面 0.060, 给夹爪手指下沉余量)
    { name: 'bin_bench', x: [0.225, 0.375], y: [-0.49, -0.11], z: [0.040, 0.045] },
    { name: 'tool_bench', x: [0.225, 0.375], y: [0.11, 0.49], z: [0.040, 0.045] },
    // 料箱隔板 (4条, 薄壁)
    { name: 'bin_wall_p1', x: [0.25, 0.35], y: [-0.482, -0.478], z: [0.060, 0.113] },
    { name: 'bin_wall_p2', x: [0.25, 0.35], y: [-0.362, -0.358], z: [0.060, 0.113] },
    { name: 'bin_wall_p3', x: [0.25, 0.35], y: [-0.242, -0.238], z: [0.060, 0.113] },
    { name: 'bin_wall_p4', x: [0.25, 0.35], y: [-0.122, -0.118], z: [0.060, 0.113] },
    // 料箱侧壁 (左右)
    { name: 'bin_wall_left',  x: [0.248, 0.252], y: [-0.48, -0.12], z: [0.060, 0.113] },
    { name: 'bin_wall_right', x: [0.348, 0.352], y: [-0.48, -0.12], z: [0.060, 0.113] },
  ];

  _pointInArmObstacle(x, y, z) {
    for (const box of MockAgent.ARM_COLLISION_BOXES) {
      if (x >= box.x[0] && x <= box.x[1] && y >= box.y[0] && y <= box.y[1] && z >= box.z[0] && z <= box.z[1]) {
        return box.name;
      }
    }
    return null;
  }

  _checkArmEnvCollision() {
    this.robot.root.updateMatrixWorld(true);
    const corner = new THREE.Vector3();
    for (const name of MockAgent.ARM_COLLISION_LINKS) {
      const mesh = this.robot.linkMeshes?.[name];
      const bb = this.robot.boundingBoxes?.[name];
      if (!mesh || !bb) continue;
      const mins = [bb.min.x, bb.min.y, bb.min.z];
      const maxs = [bb.max.x, bb.max.y, bb.max.z];
      for (let i = 0; i < 8; i++) {
        corner.set(
          (i & 1) ? maxs[0] : mins[0],
          (i & 2) ? maxs[1] : mins[1],
          (i & 4) ? maxs[2] : mins[2],
        ).applyMatrix4(mesh.matrixWorld);
        const hit = this._pointInArmObstacle(corner.x, corner.y, corner.z);
        if (hit) return `${name}<->${hit}`;
      }
    }
    const ground = this.robot._checkGroundCollision?.();
    return ground ? `ground<->${ground}` : null;
  }

  _checkArmTrajectoryCollision(targetMap, samples = 24) {
    const names = Object.keys(targetMap).filter(n => ARM_JOINT_NAMES.includes(n));
    if (!names.length) return null;
    const saved = ARM_JOINT_NAMES.map(n => this.robot.getJoint(n));
    const from = Object.fromEntries(ARM_JOINT_NAMES.map(n => [n, this.robot.getJoint(n)]));
    const to = { ...from, ...targetMap };
    for (const n of ARM_JOINT_NAMES) {
      const lim = this.robot.jointLimits?.[n];
      if (lim && (to[n] < lim[0] - 1e-6 || to[n] > lim[1] + 1e-6)) return `${n}_limit`;
    }
    for (let i = 0; i <= samples; i++) {
      const t = i / samples;
      const vals = ARM_JOINT_NAMES.map(n => from[n] + (to[n] - from[n]) * t);
      this.robot.applyJointState(ARM_JOINT_NAMES, vals);
      const hit = this._checkArmEnvCollision();
      if (hit) {
        this.robot.applyJointState(ARM_JOINT_NAMES, saved);
        return hit;
      }
    }
    this.robot.applyJointState(ARM_JOINT_NAMES, saved);
    return null;
  }

  /** 在关节插值轨迹上采样检查碰撞 (fromJoints → toJoints), 不依赖当前关节状态 */
  _checkTrajCollision(fromJoints, toJoints, samples = 20) {
    const names = ARM_JOINT_NAMES;
    const saved = names.map(n => this.robot.getJoint(n));
    const from = names.map(n => fromJoints[n] ?? 0);
    const to = names.map(n => toJoints[n] ?? 0);
    let hit = null;
    for (let i = 0; i <= samples; i++) {
      const t = i / samples;
      const vals = names.map((n, idx) => from[idx] + (to[idx] - from[idx]) * t);
      this.robot.applyJointState(names, vals);
      const c = this._checkArmEnvCollision();
      if (c) { hit = c; break; }
    }
    this.robot.applyJointState(names, saved);
    return hit;
  }

  /** 关节角是否全部在 URDF 限位内 */
  _jointsWithinLimits(joints) {
    for (const n of ARM_JOINT_NAMES) {
      const lim = this.robot.jointLimits?.[n];
      if (!lim) continue;
      const v = joints[n] ?? 0;
      if (v < lim[0] - 1e-6 || v > lim[1] + 1e-6) return false;
    }
    return true;
  }

  /** 返回所有 IK 解 (不同分支), 供 retract 逐个尝试碰撞检测 */
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

  _isInMobileArea(x, y) {
    const a = MockAgent.MOBILE_AREA;
    const r = MockAgent.CHASSIS_RADIUS;
    return x >= a.xMin + r && x <= a.xMax - r && y >= a.yMin + r && y <= a.yMax - r;
  }

  /** 检查点是否在工作台占地范围内 */
  _isInBenchArea(x, y) {
    for (const f of MockAgent.BENCH_FOOTPRINTS) {
      if (x >= f.xMin - 0.03 && x <= f.xMax + 0.03 &&
          y >= f.yMin - 0.03 && y <= f.yMax + 0.03) return true;
    }
    return false;
  }

  _isChassisPoseSafe(x, y) {
    if (!this._isInMobileArea(x, y)) return false;
    const m = MockAgent.CHASSIS_RADIUS + MockAgent.CHASSIS_CLEARANCE;
    for (const f of MockAgent.BENCH_FOOTPRINTS) {
      if (x >= f.xMin - m && x <= f.xMax + m &&
          y >= f.yMin - m && y <= f.yMax + m) return false;
    }
    return true;
  }

  _nearestSafeChassisPoint(x, y) {
    if (this._isChassisPoseSafe(x, y)) return { x, y };
    const step = 0.04;
    let best = null;
    for (let r = step; r <= 0.8; r += step) {
      const n = Math.max(12, Math.ceil((2 * Math.PI * r) / step));
      for (let i = 0; i < n; i++) {
        const a = i * 2 * Math.PI / n;
        const cx = x + Math.cos(a) * r;
        const cy = y + Math.sin(a) * r;
        if (!this._isChassisPoseSafe(cx, cy)) continue;
        const score = Math.hypot(cx - x, cy - y);
        if (!best || score < best.score) best = { x: cx, y: cy, score };
      }
      if (best) return best;
    }
    return null;
  }

  /** 计算底盘站位: 在大作业区内采样最近、可达、避开工作台的抓取站位 */
  _chassisGoalFor(xyz) {
    const cur = this.robot.root.position;
    const armBase = new THREE.Vector3();
    this.robot.jointGroups['joint1'].getWorldPosition(armBase);
    const dz = xyz[2] - MockAgent.ARM_BASE[2];
    const planarDist = Math.max(0.18, Math.min(0.34,
      Math.sqrt(Math.max(0, MockAgent.ARM_COMFORT_DIST * MockAgent.ARM_COMFORT_DIST - dz * dz))));
    const angles = [Math.atan2(armBase.y - xyz[1], armBase.x - xyz[0])];
    for (let i = 0; i < 16; i++) angles.push(i * Math.PI / 8);

    let best = null;
    for (const a of angles) {
      const armGoalX = xyz[0] + Math.cos(a) * planarDist;
      const armGoalY = xyz[1] + Math.sin(a) * planarDist;
      const yaw = Math.atan2(xyz[1] - armGoalY, xyz[0] - armGoalX);
      const cos = Math.cos(yaw), sin = Math.sin(yaw);
      const chassisX = armGoalX - (cos * MockAgent.ARM_BASE[0] - sin * MockAgent.ARM_BASE[1]);
      const chassisY = armGoalY - (sin * MockAgent.ARM_BASE[0] + cos * MockAgent.ARM_BASE[1]);
      if (!this._isChassisPoseSafe(chassisX, chassisY)) continue;
      const score = Math.hypot(chassisX - cur.x, chassisY - cur.y);
      if (!best || score < best.score) best = { x: chassisX, y: chassisY, yaw, score };
    }

    if (!best) {
      const safe = this._nearestSafeChassisPoint(xyz[0] - MockAgent.ARM_BASE[0], xyz[1] - MockAgent.ARM_BASE[1]);
      if (safe) return [safe.x, safe.y, 0];
      return [this.robot.root.position.x, this.robot.root.position.y, this.robot.root.rotation.z];
    }
    return [best.x, best.y, best.yaw];
  }

  /** A*路径规划: 使用底盘footprint避开工作台占地, 返回航点数组 */
  _planChassisPath(targetX, targetY) {
    const start = { x: this.robot.root.position.x, y: this.robot.root.position.y };
    const safeGoal = this._nearestSafeChassisPoint(targetX, targetY);
    if (!safeGoal) return [start];
    const goal = { x: safeGoal.x, y: safeGoal.y };
    if (Math.hypot(goal.x - targetX, goal.y - targetY) > 0.01) {
      this._log(`[chassis] 目标站位碰撞, 改到最近安全点 (${goal.x.toFixed(2)}, ${goal.y.toFixed(2)})`);
    }
    if (this._segmentChassisSafe(start, goal)) return [goal];

    const res = 0.04;
    const area = MockAgent.MOBILE_AREA;
    const ix = (x) => Math.round((x - area.xMin) / res);
    const iy = (y) => Math.round((y - area.yMin) / res);
    const wx = (i) => area.xMin + i * res;
    const wy = (i) => area.yMin + i * res;
    const sx = ix(start.x), sy = iy(start.y);
    let gx = ix(goal.x), gy = iy(goal.y);
    // 网格离散化可能导致目标格不安全 → 搜索最近安全格
    if (!this._isChassisPoseSafe(wx(gx), wy(gy))) {
      let found = null;
      for (let r = 1; r <= 30 && !found; r++) {
        for (let i = -r; i <= r && !found; i++) {
          for (const [cx, cy] of [[i,r],[i,-r],[r,i],[-r,i]]) {
            const nx = gx + cx, ny = gy + cy;
            if (this._isChassisPoseSafe(wx(nx), wy(ny))) { found = { x: nx, y: ny }; break; }
          }
        }
      }
      if (found) { gx = found.x; gy = found.y; }
      else {
        this._log('[chassis] 目标附近无安全格, 保持当前位置');
        return [start];
      }
    }
    const key = (x, y) => `${x},${y}`;
    const open = [{ x: sx, y: sy, g: 0, f: Math.hypot(gx - sx, gy - sy) }];
    const came = new Map();
    const cost = new Map([[key(sx, sy), 0]]);
    const closed = new Set();
    const dirs = [[1,0],[-1,0],[0,1],[0,-1],[1,1],[1,-1],[-1,1],[-1,-1]];

    while (open.length) {
      open.sort((a, b) => a.f - b.f);
      const cur = open.shift();
      const ck = key(cur.x, cur.y);
      if (closed.has(ck)) continue;
      closed.add(ck);
      if (cur.x === gx && cur.y === gy) break;
      for (const [dx, dy] of dirs) {
        const nx = cur.x + dx, ny = cur.y + dy;
        const px = wx(nx), py = wy(ny);
        if (!this._isChassisPoseSafe(px, py)) continue;
        const nk = key(nx, ny);
        const ng = cur.g + Math.hypot(dx, dy);
        if (ng >= (cost.get(nk) ?? Infinity)) continue;
        cost.set(nk, ng);
        came.set(nk, ck);
        open.push({ x: nx, y: ny, g: ng, f: ng + Math.hypot(gx - nx, gy - ny) });
      }
    }

    const gk = key(gx, gy);
    if (!came.has(gk)) {
      this._log('[chassis] A*未找到完整路径, 保持当前位置');
      return [start];
    }
    const cells = [];
    let k = gk;
    while (k && k !== key(sx, sy)) {
      const [cx, cy] = k.split(',').map(Number);
      cells.push({ x: wx(cx), y: wy(cy) });
      k = came.get(k);
    }
    cells.reverse();
    return this._simplifyChassisPath([start, ...cells]);
  }

  _segmentChassisSafe(p1, p2) {
    const samples = Math.max(2, Math.ceil(Math.hypot(p2.x - p1.x, p2.y - p1.y) / 0.02));
    for (let i = 0; i <= samples; i++) {
      const t = i / samples;
      const x = p1.x + (p2.x - p1.x) * t;
      const y = p1.y + (p2.y - p1.y) * t;
      if (!this._isChassisPoseSafe(x, y)) return false;
    }
    return true;
  }

  _simplifyChassisPath(points) {
    if (points.length <= 2) return points.slice(1);
    const out = [];
    let anchor = points[0];
    let i = 1;
    while (i < points.length) {
      let furthest = i;
      for (let j = i; j < points.length; j++) {
        if (this._segmentChassisSafe(anchor, points[j])) furthest = j;
        else break;
      }
      out.push(points[furthest]);
      anchor = points[furthest];
      i = furthest + 1;
    }
    return out;
  }

  // ─── 执行 ───
  async _tween(target, duration) {
    const hit = this._checkArmTrajectoryCollision(target);
    if (hit) {
      this._log(`[arm] 拒绝执行: 轨迹碰撞 ${hit}`);
      return { ok: false, reason: 'trajectory_collision', detail: hit };
    }
    this._sendArmCommand(target);
    return new Promise(resolve => {
      this.robot.tweenTo(target, duration, () => {
        this._sendArmCommand(target);
        resolve({ ok: true });
      });
    });
  }

  _sendArmCommand(target) {
    if (!this.ros?.publishEnabled || !this.ros?.connected) return;
    this.ros.sendArmCommand(target);
  }

  _sendCmdVel(v, w, force = false) {
    if (!this.ros?.publishEnabled || !this.ros?.connected) return;
    const now = performance.now();
    if (!force && now - this._lastCmdVelSent < 50) return;
    this._lastCmdVelSent = now;
    this.ros.sendCmdVel(v, w);
  }

  _navigateChassisTo(x, y, yaw) {
    if (!this.chassis) return null;
    return new Promise(resolve => {
      this.chassis.navigateTo(x, y, yaw);
      this.chassis.onNavArrived = () => {
        this._sendCmdVel(0, 0, true);
        resolve();
      };
    });
  }

  async _delay(ms) {
    return new Promise(r => setTimeout(r, ms));
  }

  // ─── 底盘移动 (重试超限后第4级恢复) ───
  // 臂基座在 base_link 中的偏移 (URDF joint1 origin)
  static ARM_BASE = [0.054, 0.001, 0.156];
  static ARM_REACH = 0.40;

  /** 计算底盘最优站位: target 与 slot 中点 - 臂偏移 */
  _computeChassisGoal(targetXyz, slotXyz) {
    const tx = targetXyz[0], ty = targetXyz[1];
    if (slotXyz) {
      const midX = (tx + slotXyz[0]) / 2;
      const midY = (ty + slotXyz[1]) / 2;
      return [midX - MockAgent.ARM_BASE[0], midY - MockAgent.ARM_BASE[1], 0];
    }
    return [tx - MockAgent.ARM_BASE[0], ty - MockAgent.ARM_BASE[1], 0];
  }

  /** 平滑移动底盘到世界坐标 (路径规划 + 避障 + 臂先收回) */
  async _moveChassisTo(x, y, yaw = 0, duration = 2.0) {
    // 1. 先把臂抬到安全高度 (避免移动时臂撞工作台)
    const curJoints = {};
    ARM_JOINT_NAMES.forEach(j => curJoints[j] = this.robot.getJoint(j));
    const isUplift = curJoints['joint2'] > 0.3 && curJoints['joint3'] > 1.0;
    if (!isUplift) {
      this._log('[chassis] 臂未收回, 先抬到安全高度');
      // 保持当前joint1方向, 只抬升joint2-5 (避免不必要旋转)
      const lifted = await this._tween({
        joint1: curJoints['joint1'],
        joint2: PRESETS.arm_uplift.joint2,
        joint3: PRESETS.arm_uplift.joint3,
        joint4: PRESETS.arm_uplift.joint4,
        joint5: 0,
      }, 1.0);
      if (!lifted.ok) return lifted;
    }

    // 2. 路径规划 (避开工作台占地)
    const startX0 = this.robot.root.position.x;
    const startY0 = this.robot.root.position.y;
    let waypoints = this._planChassisPath(x, y);
    // 航点=起点 (A*失败或目标极近): 判断是否仅需近距离对齐
    if (waypoints.length <= 1 && Math.hypot(waypoints[0].x - startX0, waypoints[0].y - startY0) < 0.01) {
      const reqDist = Math.hypot(x - startX0, y - startY0);
      if (reqDist < 0.05) {
        const safe = this._nearestSafeChassisPoint(x, y) || { x, y };
        waypoints = [safe];
        this._log(`[chassis] 近距离对齐 (${(reqDist * 100).toFixed(1)}cm), 直接执行`);
      } else {
        this._log('[chassis] 路径规划失败, 底盘未移动');
        return { ok: false, reason: 'path_not_found', detail: 'A*无法找到安全路径到目标位置' };
      }
    }
    if (waypoints.length > 1) {
      this._log(`[chassis] 路径规划: ${waypoints.length}个航点 (绕行工作台)`);
    }

    if (this.chassis) {
      for (let i = 0; i < waypoints.length; i++) {
        const wp = waypoints[i];
        const isLast = i === waypoints.length - 1;
        const next = waypoints[i + 1];
        const wpYaw = isLast
          ? yaw
          : (next ? Math.atan2(next.y - wp.y, next.x - wp.x) : this.robot.root.rotation.z);
        this._log(`[chassis] 航点 ${i + 1}/${waypoints.length}: → (${wp.x.toFixed(2)}, ${wp.y.toFixed(2)})`);
        const navPromise = this._navigateChassisTo(wp.x, wp.y, wpYaw).then(() => 'arrived');
        const timeoutPromise = new Promise(resolve => setTimeout(() => resolve('timeout'), 20000));
        const navResult = await Promise.race([navPromise, timeoutPromise]);
        if (navResult === 'timeout') {
          this.chassis.cancelNav();
          this._sendCmdVel(0, 0, true);
          return { ok: false, reason: 'chassis_timeout', detail: `底盘移动到航点${i + 1}超时` };
        }
      }
      this._sendCmdVel(0, 0, true);
      return { ok: true };
    }

    // 3. 沿航点差速移动 (原地转向 → 直行到达, 符合4WD差速运动学)
    const root = this.robot.root;
    const maxLin = 0.35, maxAng = 1.2, accel = 1.5, angAccel = 4.0;
    const wheelRadius = 0.038, trackWidth = 0.152;
    const posTol = 0.012, yawTol = 0.05;
    const norm = (a) => Math.atan2(Math.sin(a), Math.cos(a));

    for (let i = 0; i < waypoints.length; i++) {
      const wp = waypoints[i];
      const isLast = i === waypoints.length - 1;
      this._log(`[chassis] 航点 ${i + 1}/${waypoints.length}: → (${wp.x.toFixed(2)}, ${wp.y.toFixed(2)})`);

      let v = 0, w = 0, prevT = performance.now();
      const tStart = performance.now();
      let timeout = false;

      await new Promise(resolve => {
        const step = () => {
          const now = performance.now();
          const dt = Math.min((now - prevT) / 1000, 0.05);
          prevT = now;
          if (now - tStart > 15000) { timeout = true; resolve(); return; }

          const dx = wp.x - root.position.x;
          const dy = wp.y - root.position.y;
          const dist = Math.hypot(dx, dy);
          const heading = Math.atan2(dy, dx);
          const yawErr = norm(heading - root.rotation.z);
          const ay = Math.abs(yawErr);

          let targetV = 0, targetW = 0, arrived = false;

          if (dist > posTol) {
            const brakeV = Math.sqrt(2 * accel * dist) * 0.9;
            let vmax = Math.min(maxLin, dist, brakeV);
            if (dist < 0.12) vmax = Math.min(vmax, dist * 0.5);
            if (ay > 0.25) {
              targetW = Math.sign(yawErr) * Math.min(maxAng, ay * 2.5);
            } else {
              targetV = vmax;
              targetW = Math.max(-maxAng, Math.min(maxAng, yawErr * 2.0));
            }
          } else if (isLast) {
            const finalErr = norm(yaw - root.rotation.z);
            if (Math.abs(finalErr) < yawTol) arrived = true;
            else targetW = Math.sign(finalErr) * Math.min(maxAng, Math.abs(finalErr) * 2.0);
          } else {
            arrived = true;
          }

          if (arrived) { this._sendCmdVel(0, 0, true); resolve(); return; }

          const maxDv = accel * dt, maxDw = angAccel * dt;
          v += Math.max(-maxDv, Math.min(maxDv, targetV - v));
          w += Math.max(-maxDw, Math.min(maxDw, targetW - w));

          const curYaw = root.rotation.z;
          root.position.x += v * Math.cos(curYaw) * dt;
          root.position.y += v * Math.sin(curYaw) * dt;
          root.rotation.z = norm(curYaw + w * dt);

          const vL = v - w * trackWidth / 2;
          const vR = v + w * trackWidth / 2;
          const wL = vL / wheelRadius * dt;
          const wR = vR / wheelRadius * dt;
          this.robot.setJoint(WHEEL_JOINT_NAMES[0], this.robot.getJoint(WHEEL_JOINT_NAMES[0]) + wL);
          this.robot.setJoint(WHEEL_JOINT_NAMES[1], this.robot.getJoint(WHEEL_JOINT_NAMES[1]) + wL);
          this.robot.setJoint(WHEEL_JOINT_NAMES[2], this.robot.getJoint(WHEEL_JOINT_NAMES[2]) + wR);
          this.robot.setJoint(WHEEL_JOINT_NAMES[3], this.robot.getJoint(WHEEL_JOINT_NAMES[3]) + wR);
          this._sendCmdVel(v, w);

          requestAnimationFrame(step);
        };
        step();
      });
      if (timeout) { this._sendCmdVel(0, 0, true); this._log('[chassis] 航点超时, 跳过'); break; }
    }
    this._sendCmdVel(0, 0, true);
    return { ok: true };
  }

  async _moveToPose(p) {
    const target = new THREE.Vector3(p.x, p.y, p.z);
    // 保存当前臂关节
    const cur = {};
    ARM_JOINT_NAMES.forEach(j => cur[j] = this.robot.getJoint(j));
    // IK 求解 (会修改 robot 关节)
    const result = this.ik.solve(target);
    if (!result.solved) {
      this.robot.applyJointState(ARM_JOINT_NAMES, ARM_JOINT_NAMES.map(j => cur[j]));
      return { ok: false, reason: 'unreachable' };
    }
    // 取 IK 结果关节角
    const ikJoints = {};
    ARM_JOINT_NAMES.forEach(j => ikJoints[j] = this.robot.getJoint(j));
    // 恢复当前关节
    this.robot.applyJointState(ARM_JOINT_NAMES, ARM_JOINT_NAMES.map(j => cur[j]));

    // 检查 IK 结果关节角是否合理 (joint1 不应超过 ±π)
    for (const j of ARM_JOINT_NAMES) {
      if (Math.abs(ikJoints[j]) > Math.PI) {
        // 归一化到 [-π, π]
        ikJoints[j] = Math.atan2(Math.sin(ikJoints[j]), Math.cos(ikJoints[j]));
      }
    }

    // 通过点规划: 如果目标Z低于当前TCP-Z, 先抬到安全高度再过去
    const tcpNow = this.robot.getGripperTCP();
    const safeZ = Math.max(tcpNow.z, target.z + 0.05, 0.12); // 安全高度至少12cm
    const needLift = tcpNow.z < safeZ - 0.02;

    // 记录"目标正上方"关节角: 当本次为竖直下降 (已在安全高度, xy接近, z降低) 时,
    // 当前位姿即下降起点(目标正上方), 供后续 retract 逆序抬升 (逆序 = 下降的精确逆, 必然无碰撞)
    // 同时记录夹爪状态: 放置后 retract 用张开, 抓取后 retract 用闭合 — 逆序必须用下降段同状态
    if (!needLift && target.z < tcpNow.z - 0.02 &&
        Math.hypot(target.x - tcpNow.x, target.y - tcpNow.y) < 0.03) {
      this._lastAboveJoints = { ...cur };
      const g = {};
      GRIPPER_JOINT_NAMES.forEach(j => g[j] = this.robot.getJoint(j));
      this._lastAboveGripper = g;
    }

    if (needLift && p.d > 0.8) {
      // 先抬到安全高度 (用 IK 求解上方点)
      const liftTarget = new THREE.Vector3(tcpNow.x, tcpNow.y, safeZ);
      this.ik.solve(liftTarget); // 会修改 robot 关节
      const liftJoints = {};
      ARM_JOINT_NAMES.forEach(j => liftJoints[j] = this.robot.getJoint(j));
      this.robot.applyJointState(ARM_JOINT_NAMES, ARM_JOINT_NAMES.map(j => cur[j]));
      // 抬升 (短时间)
      const lifted = await this._tween(liftJoints, Math.min(0.5, p.d * 0.4));
      if (!lifted.ok) return lifted;
      // 再到目标
      return this._tween(ikJoints, p.d);
    }

    return this._tween(ikJoints, p.d || 1.0);
  }

  _graspNearest() {
    const tcp = this.robot.getGripperTCP();
    let best = null, bestD = Infinity;
    for (const t of this.tools) {
      if (t.grasped) continue;
      const d = Math.hypot(
        t.currentXyz[0] - tcp.x, t.currentXyz[1] - tcp.y, t.currentXyz[2] - tcp.z);
      if (d < bestD) { bestD = d; best = t; }
    }
    if (best && bestD < GRASP_ATTACH_DISTANCE) {
      best.grasped = true;
      return true;
    }
    return false;
  }

  _release() {
    const tcp = this.robot.getGripperTCP();
    for (const t of this.tools) {
      if (t.grasped) {
        t.grasped = false;
        let dx = 0, dy = 0;
        if (this._injectFailure) {
          // 模拟放置偏移 (零件未入格)
          dx = 0.04; dy = 0.03;
          this._injectFailure = false;
          this._log('[release] ⚠ 注入放置偏移 (模拟未入格)');
        }
        t.currentXyz = [tcp.x + dx, tcp.y + dy, tcp.z];
        if (t.mesh) t.mesh.position.set(...t.currentXyz);
        return true;
      }
    }
    return false;
  }

  // ── retract (收回机械臂: 垂直抬升 + 安全收臂, 从上往下抓取/放置的逆操作) ──
  // 多策略抬升, 避免料箱壁/工作台穿模 (与 MCPTools._retract 一致):
  //   1a) 逆序抬升: 复用 _moveToPose 下降段记录的"目标正上方"关节角 (下降已过碰撞检测 ⇒ 逆序必然无碰撞)
  //   1b) 多解 IK 垂直抬升: 尝试所有分支解, 选首个无碰撞的
  //   1c) 后退脱离 + 抬升: 臂伸出在障碍上方时, 先向臂基座方向后退再抬起
  //   2)  过渡到安全收臂姿态 (保持 joint1 方向, 非致命: 碰撞则跳过, 臂已在安全高度)
  async _retract(p = {}) {
    const cur = {};
    ARM_JOINT_NAMES.forEach(j => cur[j] = this.robot.getJoint(j));
    const tcpNow = this.robot.getGripperTCP();
    // 安全收回高度: 高于工作台(0.060)与料箱壁(0.113), 确保抬起后连杆不穿模
    const SAFE_RETRACT_Z = 0.16;

    // 保存夹爪状态; 未持物时(放置后) 逆序抬升前临时复刻下降段夹爪状态:
    // 下降用闭合夹爪通过碰撞检测, 放置后夹爪已张开 → 张开态下 link7 等连杆位姿不同可能穿料箱壁,
    // 须先回到下降段夹爪状态再逆序, 使逆序 = 下降的精确逆 (必然无碰撞)
    const savedGrip = {};
    GRIPPER_JOINT_NAMES.forEach(j => savedGrip[j] = this.robot.getJoint(j));
    const holdingObject = (this.tools ?? []).some(t => t.grasped);
    let gripAdjusted = false;
    const restoreGrip = async () => {
      if (!gripAdjusted) return;
      await new Promise(resolve => {
        this.robot.tweenTo(savedGrip, 0.4, () => resolve());
      });
    };

    const segments = [];
    let segStart = cur;

    // ── 阶段1a: 逆序抬升 (复用下降段"目标正上方"关节角 + 下降段夹爪状态) ──
    if (this._lastAboveJoints) {
      const above = this._lastAboveJoints;
      // 未持物时(放置后), 临时切换到下降段夹爪状态, 使逆序 = 下降的精确逆 (必然无碰撞)
      if (!holdingObject && this._lastAboveGripper) {
        await new Promise(resolve => {
          this.robot.tweenTo(this._lastAboveGripper, 0.4, () => resolve());
        });
        gripAdjusted = true;
      }
      const c = this._checkTrajCollision(cur, above, 20);
      if (!c) {
        segments.push({ joints: above, duration: 0.9, desc: '垂直抬升到安全高度' });
        segStart = above;
      } else {
        this._log(`[arm] 逆序抬升碰撞, 改用多策略抬升: ${c}`);
      }
      this._lastAboveJoints = null;   // 一次性消费
      this._lastAboveGripper = null;
    }

    // ── 阶段1b: 多解 IK 垂直抬升 (尝试所有分支解, 选首个无碰撞的) ──
    if (segments.length === 0 && tcpNow.z < SAFE_RETRACT_Z - 0.01) {
      const liftZ = Math.max(tcpNow.z + (p.lift ?? 0.08), SAFE_RETRACT_Z);
      const sols = this._multiIKSolutions(new THREE.Vector3(tcpNow.x, tcpNow.y, liftZ));
      for (const sol of sols) {
        const liftJoints = {};
        ARM_JOINT_NAMES.forEach(j => liftJoints[j] = Math.atan2(
          Math.sin(sol.joints[j]), Math.cos(sol.joints[j])));
        if (!this._jointsWithinLimits(liftJoints)) continue;
        const c = this._checkTrajCollision(cur, liftJoints, 20);
        if (!c) {
          segments.push({ joints: liftJoints, duration: 0.9, desc: '垂直抬升到安全高度' });
          segStart = liftJoints;
          break;
        }
      }
    }

    // ── 阶段1c: 后退脱离 + 抬升 (臂伸出在障碍上方时, 先向臂基座方向后退再抬起) ──
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
        const rs = this._multiIKSolutions(new THREE.Vector3(rx, ry, tcpNow.z));
        for (const sol of rs) {
          const rj = {};
          ARM_JOINT_NAMES.forEach(j => rj[j] = Math.atan2(
            Math.sin(sol.joints[j]), Math.cos(sol.joints[j])));
          if (!this._jointsWithinLimits(rj)) continue;
          const c = this._checkTrajCollision(cur, rj, 16);
          if (!c) {
            segments.push({ joints: rj, duration: 0.6, desc: '后退脱离障碍' });
            const liftZ = Math.max(tcpNow.z + 0.08, SAFE_RETRACT_Z);
            const ls = this._multiIKSolutions(new THREE.Vector3(rx, ry, liftZ));
            for (const sol2 of ls) {
              const lj = {};
              ARM_JOINT_NAMES.forEach(j => lj[j] = Math.atan2(
                Math.sin(sol2.joints[j]), Math.cos(sol2.joints[j])));
              if (!this._jointsWithinLimits(lj)) continue;
              const c2 = this._checkTrajCollision(rj, lj, 20);
              if (!c2) {
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

    // ── 阶段2: 过渡到安全收臂姿态 (非致命: 碰撞则跳过, 臂已在安全高度) ──
    const retractPose = {
      joint1: cur['joint1'],
      joint2: PRESETS.arm_uplift.joint2,
      joint3: PRESETS.arm_uplift.joint3,
      joint4: PRESETS.arm_uplift.joint4,
      joint5: 0,
    };
    const c2 = this._checkTrajCollision(segStart, retractPose, 20);
    if (!c2) {
      segments.push({ joints: retractPose, duration: 0.9, desc: '收回至安全姿态' });
    }

    if (segments.length === 0) {
      await restoreGrip();
      return { ok: false, reason: 'trajectory_collision', detail: '收回轨迹碰撞 (所有抬升策略均失败)' };
    }

    this._log(`[arm] 收回机械臂到安全姿态 (保持方向 joint1=${cur['joint1'].toFixed(2)})`);
    for (const seg of segments) {
      this._log(`[arm] 执行段: ${seg.desc} (${seg.duration}s)`);
      await new Promise(resolve => {
        this.robot.tweenTo(seg.joints, seg.duration, () => resolve());
      });
    }
    await restoreGrip();  // 在安全高度恢复夹爪状态
    return { ok: true };
  }

  // ─── 面板回调 ───
  _log(line) {
    if (this.panel) this.panel._onLog(line);
  }
  _setStatus(status, info) {
    if (this.panel) {
      this.panel._onStatus(JSON.stringify({ status, info }));
    }
  }

  // ─── 主入口 (LLM 模式 / 正则 NLU 回退) ───
  async run(instruction) {
    if (this._busy) return;
    this._busy = true;

    this._log(`>>> 指令: ${instruction}`);
    this._setStatus('running', instruction);

    // 无 API Key → 正则 NLU 模式
    if (!this._llmAgent || !this._llmAgent.apiKey) {
      this._log('[NLU] 未配置 API Key, 启动正则指令解析模式');
      try {
        await this.runNLU(instruction);
      } catch (e) {
        this._log(`[NLU] 异常: ${e.message}`);
        this._setStatus('failed', e.message);
      }
      this._busy = false;
      return;
    }

    this._log('[LLM] 启动大模型决策模式');
    try {
      const result = await this._llmAgent.run(instruction);
      if (result.ok) {
        this._log(`<<< 结果: done (${result.turns}轮对话)`);
        this._setStatus('done', result.summary);
      } else {
        this._log(`<<< 结果: failed — ${result.summary}`);
        this._setStatus('failed', result.summary);
      }
    } catch (e) {
      this._log(`[LLM] 异常: ${e.message}`);
      this._setStatus('failed', e.message);
    }
    this._busy = false;
  }

  // ─── 正则 NLU 指令解析 (无 API Key 时的回退模式) ───

  /** 工具中文名 → 英文 class */
  static TOOL_CN_MAP = {
    '扳手': 'wrench', '螺丝刀': 'screwdriver', '螺母': 'nut',
    '滚柱': 'roller', '螺丝': 'screw',
  };
  static TOOL_CN_KEYS = ['螺丝刀', '扳手', '螺母', '滚柱', '螺丝'];

  /**
   * 正则解析自然语言指令, 提取动作/工具/位置/料箱格.
   * @returns {{action, toolClass, side?, slot?} | null}
   */
  parseInstruction(text) {
    const t = text.replace(/\s/g, '');
    let action = null;
    let toolClass = null;
    let side = null;
    let slot = null;

    // 动作识别
    if (/(?:取|拿|抓|给|捡|夹)/.test(t)) action = 'grasp';
    else if (/(?:放|置|丢|扔)/.test(t)) action = 'place';
    if (!action) return null;

    // 工具识别 (按长度倒序, 避免"螺丝"匹配"螺丝刀")
    for (const cn of MockAgent.TOOL_CN_KEYS) {
      if (t.includes(cn)) {
        toolClass = MockAgent.TOOL_CN_MAP[cn];
        break;
      }
    }
    if (!toolClass) return null;

    // 位置 (左/右)
    if (/(?:左)/.test(t)) side = 'left';
    else if (/(?:右)/.test(t)) side = 'right';

    // 料箱格子
    const slotMatch = t.match(/第\s*([一二三四1234])\s*[个]?(?:格|槽|料箱)/);
    if (slotMatch) {
      const m = slotMatch[1];
      slot = { '一': 1, '二': 2, '三': 3, '四': 4, '1': 1, '2': 2, '3': 3, '4': 4 }[m] || null;
    }

    return { action, toolClass, side, slot };
  }

  /** 正则 NLU 模式: 解析指令 → 脚本化执行 */
  async runNLU(instruction) {
    const plan = this.parseInstruction(instruction);
    if (!plan) {
      this._log('[NLU] 无法解析指令, 请使用格式如: "取出左侧的扳手" 或 "把滚柱放到料箱第三格"');
      this._setStatus('failed', '指令无法解析');
      return;
    }

    this._log(`[NLU] 解析结果: 动作=${plan.action} 工具=${plan.toolClass} 位置=${plan.side || '任意'} 格子=${plan.slot || '无'}`);

    // 步骤1: 感知
    this._log('[nlu] 步骤1: 感知场景物体');
    const objects = this._perceive();
    if (objects.length === 0) {
      this._log('[NLU] 未检测到任何物体, 请确认 RGB-D 相机和 YOLO 检测器已启动');
      this._setStatus('failed', '未检测到物体');
      return;
    }
    this._log(`[nlu] 检测到 ${objects.length} 个物体: ${objects.map(o => `${o.class}@(${o.xyz.map(v=>v.toFixed(2))})`).join(', ')}`);

    // 步骤2: 筛选目标工具
    let candidates = objects.filter(o => o.class === plan.toolClass);
    if (candidates.length === 0) {
      // 尝试模糊匹配 (COCO类名包含工具名)
      candidates = objects.filter(o => o.class.includes(plan.toolClass) || plan.toolClass.includes(o.class));
    }
    if (candidates.length === 0) {
      this._log(`[NLU] 未找到 ${plan.toolClass}, 当前检测到的类别: ${objects.map(o=>o.class).join(', ')}`);
      this._setStatus('failed', `未找到${plan.toolClass}`);
      return;
    }

    // 按位置筛选
    if (plan.side === 'left') {
      const left = candidates.filter(o => o.xyz[1] < 0);
      if (left.length > 0) candidates = left;
    } else if (plan.side === 'right') {
      const right = candidates.filter(o => o.xyz[1] > 0);
      if (right.length > 0) candidates = right;
    }

    // 选最近的
    const tcp = this.robot.getGripperTCP();
    candidates.sort((a, b) => {
      const da = Math.hypot(a.xyz[0]-tcp.x, a.xyz[1]-tcp.y);
      const db = Math.hypot(b.xyz[0]-tcp.x, b.xyz[1]-tcp.y);
      return da - db;
    });
    const target = candidates[0];
    this._log(`[nlu] 步骤2: 选定目标 ${target.class} @ (${target.xyz.map(v=>v.toFixed(3))}) 距离=${Math.hypot(target.xyz[0]-tcp.x, target.xyz[1]-tcp.y).toFixed(2)}m`);

    // 步骤3: 规划并移动到目标
    this._log('[nlu] 步骤3: 规划臂运动到目标位置');
    const moveResult = await this._moveToPose({ x: target.xyz[0], y: target.xyz[1], z: target.xyz[2], d: 1.5 });
    if (!moveResult.ok) {
      this._log(`[NLU] 无法到达目标: ${moveResult.reason}`);
      this._setStatus('failed', moveResult.reason);
      return;
    }

    // 步骤4: 抓取
    this._log('[nlu] 步骤4: 闭合夹爪抓取');
    await this._tween({ ...GRIPPER_POSES.close }, 0.8);
    this._sendArmCommand({ ...GRIPPER_POSES.close });
    const grasped = this._graspNearest();
    if (!grasped) {
      this._log('[NLU] 抓取失败: 物体不在夹爪范围内');
      this._setStatus('failed', '抓取失败');
      return;
    }
    this._log('[nlu] 抓取成功');

    // 步骤5: 收回
    this._log('[nlu] 步骤5: 收回机械臂');
    await this._retract({});

    // 步骤6: 如果是放置动作 → 移动到料箱并放置
    if (plan.action === 'place' && plan.slot) {
      const slotPos = this._binSlots[plan.slot];
      if (!slotPos) {
        this._log(`[NLU] 料箱第${plan.slot}格不存在`);
        this._setStatus('failed', '料箱格子无效');
        return;
      }
      this._log(`[nlu] 步骤6: 移动到料箱第${plan.slot}格 @ (${slotPos.map(v=>v.toFixed(2))})`);

      // 底盘导航 (如果太远)
      const distToSlot = Math.hypot(slotPos[0] - tcp.x, slotPos[1] - tcp.y);
      if (distToSlot > 0.35) {
        this._log('[nlu] 步骤6a: 底盘导航靠近料箱');
        const chassisX = slotPos[0] - 0.15;
        const chassisY = slotPos[1] > 0 ? Math.max(0.05, slotPos[1] - 0.15) : Math.min(-0.05, slotPos[1] + 0.15);
        await this._moveChassisTo(chassisX, chassisY, 0, 2.0);
      }

      // 规划到格子
      this._log('[nlu] 步骤6b: 规划臂运动到格子');
      const placeResult = await this._moveToPose({ x: slotPos[0], y: slotPos[1], z: slotPos[2], d: 1.5 });
      if (!placeResult.ok) {
        this._log(`[NLU] 无法到达格子: ${placeResult.reason}`);
        this._setStatus('failed', placeResult.reason);
        return;
      }

      // 释放
      this._log('[nlu] 步骤6c: 张开夹爪释放');
      await this._tween({ ...GRIPPER_POSES.open }, 0.8);
      this._sendArmCommand({ ...GRIPPER_POSES.open });
      this._release();
      this._log('[nlu] 放置完成');

      // 收回
      this._log('[nlu] 步骤7: 收回机械臂');
      await this._retract({});
    }

    this._log('<<< 结果: done (正则NLU模式完成)');
    this._setStatus('done', 'NLU模式完成');
  }

  /** 设置 LLM 智能体 (由 main.js 注入) */
  setLLMAgent(llmAgent) { this._llmAgent = llmAgent; }

  /** 应用工具布局 (供 DatasetGenerator 调用: 清空当前工具, 用给定定义重建) */
  applyLayout(toolDefs) {
    for (const t of this.tools) this._removeToolFromScene(t);
    this.tools = (toolDefs || []).map(t => ({
      ...t, mesh: null, label: null, grasped: false, currentXyz: [...t.xyz],
    }));
    for (const t of this.tools) {
      t.mesh = createToolMesh(t.class);
      t.mesh.position.set(...t.currentXyz);
      t.mesh.rotation.z = t.rot || 0;
      t.mesh.traverse(c => { if (c.isMesh) { c.castShadow = true; } });
      this.scene.add(t.mesh);
      this._toolObjects.push(t.mesh);
      t.label = this._createLabel(TOOL_CN[t.class] || t.class, '#FFCC44');
      t.label.position.set(t.currentXyz[0], t.currentXyz[1], t.currentXyz[2] + 0.035);
      t.label.scale.set(0.040, 0.009, 1);
      this.scene.add(t.label);
      this._toolObjects.push(t.label);
    }
  }
}
