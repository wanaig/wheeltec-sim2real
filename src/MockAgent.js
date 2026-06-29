/**
 * MockAgent.js — 浏览器内全流程智能体
 *
 * 双模式:
 *   A) LLM 模式 (有大模型 API Key): 大模型决策 → MCP工具调用 → MoveIt2规划 → 执行
 *   B) 正则模式 (无 API Key): 正则NLU → 仿真感知 → 规划 → IK执行 → 失败重试
 *
 * LLM 模式下, MockAgent 作为 MCPToolExecutor 的后端, 提供:
 *   _perceive / _moveChassisTo / _moveToPose / _graspNearest / _release / _verifyGrasp / _verifyPlace
 *
 * 用法 (main.js):
 *   const mockAgent = new MockAgent(robot, ik, scene, agentPanel);
 *   agentPanel.setMockAgent(mockAgent);
 *   // 渲染循环里: mockAgent.update();
 */
import * as THREE from 'three';
import { PRESETS, GRIPPER_POSES, ARM_JOINT_NAMES } from './RobotModel.js';
import {
  createToolMesh, TOOL_CN, generateLayout,
  LAYOUTS, LAYOUT_NAMES, LAYOUT_CN,
} from './SceneLayouts.js';

// ─────────────── 默认布局 (混杂摆放, 供初始场景使用) ───────────────
const DEFAULT_LAYOUT = 'cluttered';

// 料箱3格×12cm, 在左工作台上 (Y center=-0.30)
// 旋转180°: slot1靠近过道(机械臂侧), slot3远离
// Z = 台面顶(0.060) + 底板(0.003) + 余量(0.010) = 0.073
const BIN_SLOTS = {
  1: [0.30, -0.18, 0.073], 2: [0.30, -0.30, 0.073],
  3: [0.30, -0.42, 0.073],
};

const CN_NUM = { '一':1,'二':2,'三':3,'四':4,'五':5,'六':6,
                  '1':1,'2':2,'3':3,'4':4,'5':5,'6':6 };

const TOOL_PAT = {
  screwdriver: [/螺丝刀/, /改锥/, /起子/],
  wrench: [/扳手/, /扳子/],
  nut: [/螺母/, /螺帽/],
  roller: [/滚柱/, /滚子/],
  screw: [/螺丝/, /螺钉/],
};
const POS_PAT = {
  left: [/左(?:侧|边)/],
  right: [/右(?:侧|边)/],
  center: [/中(?:间|央|部)/],
};

// ─────────────── 正则 NLU ───────────────
function parseInstruction(text) {
  let action = 'unknown';
  if (/放/.test(text)) action = 'pick_and_place';
  else if (/给|递/.test(text)) action = 'fetch';
  else if (/抓取|抓起|取/.test(text)) action = 'pick';

  let tool = null;
  for (const [t, pats] of Object.entries(TOOL_PAT)) {
    if (pats.some(p => p.test(text))) { tool = t; break; }
  }
  let pos = null;
  for (const [p, pats] of Object.entries(POS_PAT)) {
    if (pats.some(p => p.test(text))) { pos = p; break; }
  }
  let slot = null;
  const m = text.match(/第\s*([一二三四五六123456])\s*(?:格|个格子|个位置|个)/);
  if (m) slot = CN_NUM[m[1]];

  const intent = {
    action, target: { tool, position_hint: pos },
    destination: { type: null },
  };
  if (action === 'pick_and_place') intent.destination = { type: 'bin', slot };
  else if (action === 'fetch') intent.destination = { type: 'user' };
  return intent;
}

// ─────────────── MockAgent ───────────────
export class MockAgent {
  constructor(robot, ik, sceneSetup, panel) {
    this.robot = robot;
    this.ik = ik;
    this.scene = sceneSetup.scene;
    this.panel = panel;
    this._busy = false;
    this._injectFailure = false; // 模拟放置失败 (下次 _release 偏移)
    this._benchTopZ = 0.060;     // 台面顶高度 (供 DatasetGenerator 引用)
    this._binSlots = BIN_SLOTS;  // 料箱格子坐标 (供 MCPToolExecutor 引用)

    // 关闭碰撞检测 (仿真不需要, 避免 IK 被拒绝)
    this.robot.collisionEnabled = false;

    // 当前布局名
    this._currentLayout = DEFAULT_LAYOUT;

    // 工具状态 (含 3D mesh), 初始用默认布局
    const toolDefs = generateLayout(DEFAULT_LAYOUT, this._benchTopZ);
    this.tools = toolDefs.map(t => ({
      ...t, mesh: null, grasped: false, currentXyz: [...t.xyz],
    }));

    // 静态场景对象 (台面/料箱等, 切换布局时不重建)
    this._staticObjects = [];
    // 工具对象 (切换布局时清除重建)
    this._toolObjects = [];

    this._target = null;
    this._slotXyz = null;
    this._buildScene3D();
    this._buildTools();

    // 初始化臂到安全抬起姿态 (避免首次 move_base 时额外抬起)
    this.robot.applyJointState(ARM_JOINT_NAMES,
      ARM_JOINT_NAMES.map(j => PRESETS.arm_uplift[j]));
  }

  // ─── 3D 工业场景 (双工作台: 左料箱台 / 中过道 / 右工具台) ───
  _buildScene3D() {
    const benchTopZ = 0.060;  // 台面顶 (离地6cm)
    const benchCx = 0.30, benchW = 0.30, benchD = 0.38, benchT = 0.020;
    const benchMat = new THREE.MeshStandardMaterial({ color: 0x3A3A44, metalness: 0.6, roughness: 0.35 });
    const legMat = new THREE.MeshStandardMaterial({ color: 0x2A2A30, metalness: 0.5, roughness: 0.4 });
    const legH = benchTopZ - benchT;  // 0.040m

    // ── 工作台构建辅助 ──
    const buildBench = (cy) => {
      // 台面
      const top = new THREE.Mesh(new THREE.BoxGeometry(benchW, benchD, benchT), benchMat);
      top.position.set(benchCx, cy, benchTopZ - benchT / 2);
      top.receiveShadow = true;
      this.scene.add(top);
      // 台腿 (4条)
      for (const [lx, ly] of [[-1,-1],[-1,1],[1,-1],[1,1]]) {
        const leg = new THREE.Mesh(new THREE.BoxGeometry(0.03, 0.03, legH), legMat);
        leg.position.set(benchCx + lx * (benchW/2 - 0.03), cy + ly * (benchD/2 - 0.03), legH / 2);
        leg.castShadow = true;
        this.scene.add(leg);
      }
      // 安全黄黑条纹 (台面边缘)
      const stripeTex = this._createSafetyStripe();
      stripeTex.wrapS = THREE.RepeatWrapping;
      stripeTex.repeat.set(10, 1);
      const stripeMat = new THREE.MeshStandardMaterial({ map: stripeTex, roughness: 0.5 });
      const sT = 0.003;
      for (const py of [cy - benchD/2 + 0.006, cy + benchD/2 - 0.006]) {
        const e = new THREE.Mesh(new THREE.BoxGeometry(benchW - 0.012, 0.012, sT), stripeMat);
        e.position.set(benchCx, py, benchTopZ + sT / 2);
        this.scene.add(e);
      }
      for (const px of [benchCx - benchW/2 + 0.006, benchCx + benchW/2 - 0.006]) {
        const e = new THREE.Mesh(new THREE.BoxGeometry(0.012, benchD - 0.012, sT), stripeMat);
        e.position.set(px, cy, benchTopZ + sT / 2);
        this.scene.add(e);
      }
    };

    // 左工作台 (料箱台, Y center = -0.30)
    const binBenchY = -0.30;
    buildBench(binBenchY);
    // 右工作台 (工具台, Y center = +0.30)
    const toolBenchY = +0.30;
    buildBench(toolBenchY);

    // ── 机械臂小车大作业区 (覆盖双工作区, 底盘可自由选最近站位) ──
    // 地面作业区标识 (黄色, PlaneGeometry 在 Z-up 系中默认水平, 无需旋转)
    const corridorMat = new THREE.MeshStandardMaterial({ color: 0xFFCC00, metalness: 0.3, roughness: 0.5, side: THREE.DoubleSide });
    const corridor = new THREE.Mesh(
      new THREE.PlaneGeometry(0.95, 1.24), corridorMat);
    corridor.position.set(benchCx, 0, 0.001);  // 地面之上1mm
    this.scene.add(corridor);
    // 作业区边界白色边线
    for (const y of [-0.62, 0.62]) {
      const line = new THREE.Mesh(
        new THREE.PlaneGeometry(0.95, 0.008),
        new THREE.MeshStandardMaterial({ color: 0xFFFFFF, roughness: 0.6, side: THREE.DoubleSide }));
      line.position.set(benchCx, y, 0.0012);
      this.scene.add(line);
    }
    for (const x of [-0.25, 0.70]) {
      const line = new THREE.Mesh(
        new THREE.PlaneGeometry(0.008, 1.24),
        new THREE.MeshStandardMaterial({ color: 0xFFFFFF, roughness: 0.6, side: THREE.DoubleSide }));
      line.position.set(x, 0, 0.0012);
      this.scene.add(line);
    }
    // 作业区标签
    const corLbl = this._createLabel('机械臂小车作业区', '#FFCC00');
    corLbl.position.set(benchCx, 0, benchTopZ + 0.08);
    corLbl.scale.set(0.05, 0.010, 1);
    this.scene.add(corLbl);

    // ── 地面安全区域标记 (黄黑斜纹, 覆盖双台+过道) ──
    const floorStripe = this._createSafetyStripe();
    floorStripe.wrapS = floorStripe.wrapT = THREE.RepeatWrapping;
    floorStripe.repeat.set(16, 14);
    const zone = new THREE.Mesh(
      new THREE.PlaneGeometry(2.00, 2.00),
      new THREE.MeshStandardMaterial({ map: floorStripe, transparent: true, opacity: 0.20, roughness: 0.6 }));
    zone.position.set(benchCx, 0, 0.0008);
    this.scene.add(zone);

    // ── 工业分拣料箱 (在左工作台上; 3格×12cm) ──
    const binColor = new THREE.MeshStandardMaterial({ color: 0x2A5AAA, roughness: 0.5, metalness: 0.1 });
    const wallH = 0.050, wallT = 0.004, slotW = 0.12, binX = benchCx, binDpt = 0.10;
    const binTotalW = 3 * slotW;  // 0.36m
    const binFloorT = 0.003;
    const binFloorZ = benchTopZ + binFloorT / 2;
    const binWallZ = benchTopZ + binFloorT + wallH / 2;
    // 底板
    const binFloor = new THREE.Mesh(
      new THREE.BoxGeometry(binDpt, binTotalW, binFloorT), binColor);
    binFloor.position.set(binX, binBenchY, binFloorZ);
    binFloor.receiveShadow = true;
    this.scene.add(binFloor);
    // 隔板 (4条 = 3格)
    for (let i = 0; i <= 3; i++) {
      const w = new THREE.Mesh(
        new THREE.BoxGeometry(binDpt, wallT, wallH), binColor);
      w.position.set(binX, binBenchY - binTotalW / 2 + i * slotW, binWallZ);
      w.castShadow = true; w.receiveShadow = true;
      this.scene.add(w);
    }
    // 侧壁 (左右)
    for (const xOff of [-binDpt/2, binDpt/2]) {
      const sw = new THREE.Mesh(
        new THREE.BoxGeometry(wallT, binTotalW, wallH), binColor);
      sw.position.set(binX + xOff, binBenchY, binWallZ);
      sw.castShadow = true;
      this.scene.add(sw);
    }
    // 料箱标签
    const binLbl = this._createLabel('零件分拣料箱', '#88CCFF');
    binLbl.position.set(binX, binBenchY, benchTopZ + binFloorT + wallH + 0.010);
    binLbl.scale.set(0.07, 0.012, 1);
    this.scene.add(binLbl);
    // 格子编号
    for (let i = 1; i <= 3; i++) {
      const lbl = this._createLabel(`${i}`, '#44FF88');
      lbl.position.set(binX, BIN_SLOTS[i][1], benchTopZ + binFloorT + wallH + 0.003);
      lbl.scale.set(0.016, 0.009, 1);
      this.scene.add(lbl);
    }
    // 工具台标签
    const toolLbl = this._createLabel('工具散乱区', '#FFCC44');
    toolLbl.position.set(benchCx, toolBenchY, benchTopZ + 0.035);
    toolLbl.scale.set(0.06, 0.011, 1);
    this.scene.add(toolLbl);

    // ── 工业标牌 ──
    const warn = this._createLabel('⚠ 自动作业区域  AUTO MODE', '#FFAA00');
    warn.position.set(benchCx, 0, benchTopZ + 0.22);
    warn.scale.set(0.12, 0.020, 1);
    this.scene.add(warn);
  }

  /** 构建工具 mesh (从当前 this.tools 定义) */
  _buildTools() {
    // 清除旧工具
    for (const obj of this._toolObjects) {
      this.scene.remove(obj);
      if (obj.traverse) obj.traverse(c => {
        if (c.isMesh) { c.geometry?.dispose?.(); c.material?.dispose?.(); }
      });
    }
    this._toolObjects = [];

    for (let i = 0; i < this.tools.length; i++) {
      const t = this.tools[i];
      t.mesh = createToolMesh(t.class);
      t.mesh.position.set(...t.currentXyz);
      t.mesh.rotation.z = t.rot || 0;
      t.mesh.traverse(c => { if (c.isMesh) { c.castShadow = true; } });
      this.scene.add(t.mesh);
      this._toolObjects.push(t.mesh);
      // 工具标签 (在工具上方)
      t.label = this._createLabel(TOOL_CN[t.class] || t.class, '#FFCC44');
      t.label.position.set(t.currentXyz[0], t.currentXyz[1], t.currentXyz[2] + 0.035);
      t.label.scale.set(0.040, 0.009, 1);
      this.scene.add(t.label);
      this._toolObjects.push(t.label);
    }
  }

  _createSafetyStripe() {
    const cv = document.createElement('canvas');
    cv.width = 128; cv.height = 16;
    const ctx = cv.getContext('2d');
    for (let i = -2; i < 10; i++) {
      ctx.fillStyle = i % 2 === 0 ? '#FFCC00' : '#1a1a1a';
      ctx.beginPath();
      ctx.moveTo(i * 16, 0);
      ctx.lineTo((i + 1) * 16, 0);
      ctx.lineTo((i + 1) * 16 + 8, 16);
      ctx.lineTo(i * 16 + 8, 16);
      ctx.fill();
    }
    return new THREE.CanvasTexture(cv);
  }

  /** 重置场景 (工具归位到当前布局) */
  resetScene() {
    // 重新生成当前布局 (回到初始状态)
    this.applyLayout(
      generateLayout(this._currentLayout, this._benchTopZ)
    );
    this.robot.tweenTo(
      { ...PRESETS.arm_home, joint6: 0.45 }, 1.0);
    this._log('[reset] 场景已重置, 工具归位');
    this._setStatus('done', '场景重置');
  }

  /**
   * 切换布局 (供 AgentPanel / DatasetGenerator 调用)
   * @param {Array} toolDefs - [{class, xyz, rot}, ...] 来自 SceneLayouts.generateLayout
   */
  applyLayout(toolDefs) {
    this.tools = toolDefs.map(t => ({
      ...t, mesh: null, grasped: false, currentXyz: [...t.xyz],
    }));
    this._buildTools();
  }

  /** 按名称切换布局 */
  setLayout(name) {
    if (!LAYOUTS[name]) {
      this._log(`[layout] 未知布局: ${name}`);
      return;
    }
    this._currentLayout = name;
    const toolDefs = generateLayout(name, this._benchTopZ);
    this.applyLayout(toolDefs);
    this._log(`[layout] 切换至「${LAYOUT_CN[name] || name}」→ ${toolDefs.length}个工具`);
    this._setStatus('done', `布局: ${LAYOUT_CN[name] || name}`);
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
  _perceive() {
    return this.tools
      .filter(t => !t.grasped)
      .map(t => ({
        class: t.class,
        xyz: [...t.currentXyz],
        conf: 0.9,
      }));
  }

  _selectTarget(objs, intent) {
    const tool = intent.target.tool;
    const hint = intent.target.position_hint;
    const cands = objs.filter(o => o.class === tool);
    if (cands.length === 0) return null;
    // Y负 = 视觉左, Y正 = 视觉右 (相机在+X-Y方向看向原点)
    if (hint === 'left') return cands.reduce((a, b) => a.xyz[1] < b.xyz[1] ? a : b);
    if (hint === 'right') return cands.reduce((a, b) => a.xyz[1] > b.xyz[1] ? a : b);
    if (hint === 'center') return cands.reduce((a, b) => Math.abs(a.xyz[1]) < Math.abs(b.xyz[1]) ? a : b);
    return cands[0];
  }

  // ─── 规划 ───

  // 工作台占地范围 (底盘不能进入)
  static BENCH_FOOTPRINTS = [
    { xMin: 0.15, xMax: 0.45, yMin: -0.49, yMax: -0.11 }, // 左台(料箱)
    { xMin: 0.15, xMax: 0.45, yMin: 0.11, yMax: 0.49 },   // 右台(工具)
  ];
  // 底盘可移动大作业区 (覆盖双工作台和中间区域, 但不能压到工作台本体)
  static MOBILE_AREA = { xMin: -0.25, xMax: 0.70, yMin: -0.62, yMax: 0.62 };
  static CHASSIS_RADIUS = 0.13;
  static CHASSIS_CLEARANCE = 0.03;
  static ARM_COMFORT_DIST = 0.28;

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
    const gx = ix(goal.x), gy = iy(goal.y);
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

  _buildPlan(action, targetXyz, slotXyz) {
    const aH = 0.08;
    const preZ = targetXyz[2] + aH;

    // 辅助: 检查从当前底盘位置能否到达目标
    const reachable = (xyz) => {
      const armBase = new THREE.Vector3();
      this.robot.jointGroups['joint1'].getWorldPosition(armBase);
      const armX = armBase.x;
      const armY = armBase.y;
      const armZ = armBase.z;
      const d = Math.hypot(xyz[0] - armX, xyz[1] - armY, xyz[2] - armZ);
      return d <= MockAgent.ARM_REACH;
    };

    if (action === 'pick_and_place') {
      const sPreZ = slotXyz[2] + aH;
      const plan = [
        { n: 'go_preset', p: { name: 'arm_uplift', d: 1.0 } },  // 先抬到安全高度
      ];
      // 目标不可达 → 底盘移到过道站位 (不进入工作台)
      if (!reachable(targetXyz)) {
        const [gx, gy, gyaw] = this._chassisGoalFor(targetXyz);
        plan.push({ n: 'move_base_to', p: { x: gx, y: gy, yaw: gyaw, d: 2.0 } });
      }
      // 抓取: 先到上方预抓取点, 再下扎 (避免水平穿模)
      plan.push(
        { n: 'move_to_pose', p: { x: targetXyz[0], y: targetXyz[1], z: preZ, d: 1.2 } },
        { n: 'open_gripper', p: { d: 0.4 } },
        { n: 'move_to_pose', p: { x: targetXyz[0], y: targetXyz[1], z: targetXyz[2], d: 0.7 } },
        { n: 'close_gripper', p: { d: 0.6 } },
        { n: 'retract', p: { lift: 0.10, d: 0.7 } },
        { n: 'go_preset', p: { name: 'arm_uplift', d: 0.8 } },  // 抬到安全高度再移动
      );
      // 料箱不可达 → 底盘移到料箱侧过道站位
      if (!reachable(slotXyz)) {
        const [sx, sy, syaw] = this._chassisGoalFor(slotXyz);
        plan.push({ n: 'move_base_to', p: { x: sx, y: sy, yaw: syaw, d: 2.0 } });
      }
      // 放置
      plan.push(
        { n: 'move_to_pose', p: { x: slotXyz[0], y: slotXyz[1], z: sPreZ, d: 1.2 } },
        { n: 'move_to_pose', p: { x: slotXyz[0], y: slotXyz[1], z: slotXyz[2], d: 0.7 } },
        { n: 'open_gripper', p: { d: 0.4 } },
        { n: 'retract', p: { lift: 0.10, d: 0.7 } },
        { n: 'verify_place', p: {} },
        { n: 'go_preset', p: { name: 'arm_home', d: 1.0 } },
      );
      return plan;
    }
    if (action === 'fetch') {
      return [
        { n: 'go_preset',    p: { name: 'arm_uplift', d: 1.0 } },
        { n: 'move_to_pose', p: { x: targetXyz[0], y: targetXyz[1], z: preZ, d: 1.2 } },
        { n: 'open_gripper', p: { d: 0.4 } },
        { n: 'move_to_pose', p: { x: targetXyz[0], y: targetXyz[1], z: targetXyz[2], d: 0.7 } },
        { n: 'close_gripper',p: { d: 0.6 } },
        { n: 'verify_grasp', p: {} },
        { n: 'retract',      p: { lift: 0.12, d: 0.7 } },
        { n: 'go_preset',    p: { name: 'arm_rotate_uplift', d: 1.0 } },
      ];
    }
    if (action === 'pick') {
      return [
        { n: 'go_preset',    p: { name: 'arm_uplift', d: 0.8 } },
        { n: 'open_gripper', p: { d: 0.4 } },
        { n: 'move_to_pose', p: { x: targetXyz[0], y: targetXyz[1], z: preZ, d: 1.0 } },
        { n: 'move_to_pose', p: { x: targetXyz[0], y: targetXyz[1], z: targetXyz[2], d: 0.7 } },
        { n: 'close_gripper',p: { d: 0.6 } },
        { n: 'verify_grasp', p: {} },
        { n: 'retract',      p: { lift: 0.10, d: 0.7 } },
      ];
    }
    return [];
  }

  // ─── 执行 ───
  async _tween(target, duration) {
    return new Promise(resolve => {
      this.robot.tweenTo(target, duration, () => resolve({ ok: true }));
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
      await new Promise(resolve => {
        this.robot.tweenTo({
          joint1: curJoints['joint1'],
          joint2: PRESETS.arm_uplift.joint2,
          joint3: PRESETS.arm_uplift.joint3,
          joint4: PRESETS.arm_uplift.joint4,
          joint5: 0,
        }, 1.0, () => resolve());
      });
    }

    // 2. 路径规划 (避开工作台占地)
    const waypoints = this._planChassisPath(x, y);
    if (waypoints.length > 1) {
      this._log(`[chassis] 路径规划: ${waypoints.length}个航点 (绕行工作台)`);
    }

    // 3. 沿航点平滑移动 (每段独立动画)
    const root = this.robot.root;
    for (let i = 0; i < waypoints.length; i++) {
      const wp = waypoints[i];
      const isLast = i === waypoints.length - 1;
      const targetX = wp.x;
      const targetY = wp.y;
      const targetYaw = isLast ? yaw : Math.atan2(targetY - root.position.y, targetX - root.position.x);
      const segDur = duration / waypoints.length;
      const startX = root.position.x;
      const startY = root.position.y;
      const startYaw = root.rotation.z;
      const t0 = performance.now();
      const ms = segDur * 1000;
      await new Promise(resolve => {
        const step = () => {
          const t = Math.min(1, (performance.now() - t0) / ms);
          const e = t * t * (3 - 2 * t); // smoothstep
          root.position.x = startX + (targetX - startX) * e;
          root.position.y = startY + (targetY - startY) * e;
          root.rotation.z = startYaw + (targetYaw - startYaw) * e;
          if (t < 1) requestAnimationFrame(step);
          else resolve();
        };
        step();
      });
    }
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

    if (needLift && p.d > 0.8) {
      // 先抬到安全高度 (用 IK 求解上方点)
      const liftTarget = new THREE.Vector3(tcpNow.x, tcpNow.y, safeZ);
      this.ik.solve(liftTarget); // 会修改 robot 关节
      const liftJoints = {};
      ARM_JOINT_NAMES.forEach(j => liftJoints[j] = this.robot.getJoint(j));
      this.robot.applyJointState(ARM_JOINT_NAMES, ARM_JOINT_NAMES.map(j => cur[j]));
      // 抬升 (短时间)
      await this._tween(liftJoints, Math.min(0.5, p.d * 0.4));
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
    if (best && bestD < 0.12) {
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

  _verifyGrasp() {
    const cls = this._target?.class;
    const orig = this._target?.xyz;
    if (!cls || !orig) return { ok: true };
    const objs = this._perceive();
    const stillThere = objs.some(o =>
      o.class === cls &&
      Math.hypot(o.xyz[0]-orig[0], o.xyz[1]-orig[1], o.xyz[2]-orig[2]) < 0.04);
    return { ok: !stillThere, reason: stillThere ? 'object_still_on_table' : 'object_lifted' };
  }

  _verifyPlace() {
    const cls = this._target?.class;
    if (!cls || !this._slotXyz) return { ok: true };
    const objs = this._perceive();
    const placed = objs.some(o =>
      o.class === cls &&
      Math.hypot(o.xyz[0]-this._slotXyz[0], o.xyz[1]-this._slotXyz[1]) < 0.06);
    return { ok: placed, reason: placed ? 'object_in_slot' : 'object_not_in_slot' };
  }

  async _executeSkill(skill) {
    switch (skill.n) {
      case 'go_preset':
        return this._tween({ ...PRESETS[skill.p.name] }, skill.p.d);
      case 'move_base_to':
        this._log(`[chassis] 导航至 (${skill.p.x.toFixed(2)}, ${skill.p.y.toFixed(2)})`);
        return this._moveChassisTo(skill.p.x, skill.p.y, skill.p.yaw || 0, skill.p.d || 2.0);
      case 'move_to_pose':
        return this._moveToPose(skill.p);
      case 'open_gripper':
        await this._tween({ ...GRIPPER_POSES.open }, skill.p.d);
        this._release();
        return { ok: true };
      case 'close_gripper':
        await this._tween({ ...GRIPPER_POSES.close }, skill.p.d);
        this._graspNearest();
        return { ok: true };
      case 'retract': {
        const tcp = this.robot.getGripperTCP();
        return this._moveToPose({ x: tcp.x, y: tcp.y, z: tcp.z + skill.p.lift, d: skill.p.d });
      }
      case 'verify_grasp':
        await this._delay(200);
        return this._verifyGrasp();
      case 'verify_place':
        await this._delay(200);
        return this._verifyPlace();
      default:
        return { ok: false };
    }
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

  // ─── 主入口 (自动选择 LLM 模式 / 正则模式) ───
  async run(instruction) {
    if (this._busy) return;
    this._busy = true;

    this._log(`>>> 指令: ${instruction}`);
    this._setStatus('running', instruction);

    // LLM 模式: 有 API Key + executor 时走大模型决策
    if (this._llmAgent && this._llmAgent.apiKey) {
      await this._runWithLLM(instruction);
      return;
    }

    // 正则模式: 回退到规则化 NLU + 规划
    await this._runWithRegex(instruction);
  }

  /** 设置 LLM 智能体 (由 main.js 注入) */
  setLLMAgent(llmAgent) { this._llmAgent = llmAgent; }

  // ─── LLM 模式: 大模型决策 + MCP 工具调用 ───
  async _runWithLLM(instruction) {
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
      this._log(`[LLM] 异常: ${e.message}, 回退正则模式`);
      await this._runWithRegex(instruction);
    }
    this._busy = false;
  }

  // ─── 正则模式 (回退) ───
  async _runWithRegex(instruction) {
    // 1. NLU (正则解析)
    const intent = parseInstruction(instruction);
    this._log(`[nlu] 意图=${intent.action} tool=${intent.target.tool} hint=${intent.target.position_hint} slot=${intent.destination?.slot}`);
    await this._delay(300);

    // 2. 感知
    const objs = this._perceive();
    this._log(`[perceive] ${objs.length} 物体: ${objs.map(o => o.class).join(', ')}`);
    await this._delay(300);

    // 3. 选目标 + 规划
    const target = this._selectTarget(objs, intent);
    if (!target) {
      this._log('[plan] 未找到目标工具');
      this._setStatus('failed', instruction);
      this._busy = false;
      return;
    }
    this._target = { class: target.class, xyz: [...target.xyz] };
    const slot = intent.destination?.slot;
    this._slotXyz = slot ? BIN_SLOTS[slot] : null;
    const plan = this._buildPlan(intent.action, target.xyz, this._slotXyz);
    this._log(`[plan] ${plan.length} 步, 目标=${target.class}@[${target.xyz.map(v=>v.toFixed(2))}]`);
    await this._delay(300);

    // 4. 执行 + 验证 + 重试 (4级恢复策略)
    let retryCount = 0;
    const maxRetries = 2;       // 本地重试 (换目标/偏移/换格子)
    const maxChassisRetries = 2; // 底盘移动重试 (超限后触发)
    const hardLimit = maxRetries + maxChassisRetries;

    while (retryCount <= hardLimit) {
      let failed = false;
      let failSkill = '';
      for (let i = 0; i < plan.length; i++) {
        const skill = plan[i];
        const r = await this._executeSkill(skill);
        this._log(`[exec ${i}] ${skill.n} → ok=${r.ok}`);
        if (!r.ok) {
          if (skill.n === 'verify_grasp' || skill.n === 'verify_place' || skill.n === 'move_to_pose') {
            failed = true;
            failSkill = skill.n;
            break;
          }
        }
      }
      if (!failed) {
        this._log(`<<< 结果: done`);
        this._setStatus('done', instruction);
        this._busy = false;
        return;
      }
      retryCount++;
      if (retryCount > hardLimit) {
        this._log(`[replan] 重试+底盘移动均超限 → 失败`);
        this._log(`<<< 结果: failed`);
        this._setStatus('failed', instruction);
        this._busy = false;
        return;
      }

      // ── 1~2次: 本地重试 (换目标) ──
      if (retryCount <= maxRetries) {
        this._log(`[replan] 第${retryCount}次重试, 重新感知`);
        const newObjs = this._perceive();
        const newTarget = this._selectTarget(newObjs, intent);
        if (newTarget) {
          this._target = { class: newTarget.class, xyz: [...newTarget.xyz] };
          const newPlan = this._buildPlan(intent.action, newTarget.xyz, this._slotXyz);
          plan.length = 0;
          plan.push(...newPlan);
        }
        await this._delay(500);
      }
      // ── 3~4次: 底盘移动 + 重规划 (新 _buildPlan 自动插入 move_base_to) ──
      else {
        const chassisRound = retryCount - maxRetries;
        this._log(`[replan] 本地重试超限 → 第${chassisRound}次底盘移动+重规划`);
        // 重新感知
        const newObjs = this._perceive();
        const newTarget = this._selectTarget(newObjs, intent) || this._target;
        this._target = { class: newTarget.class, xyz: [...newTarget.xyz] };
        // 重建计划 (_buildPlan 自动检测不可达 → 插入 move_base_to 到目标旁和料箱旁)
        const newPlan = this._buildPlan(intent.action, newTarget.xyz, this._slotXyz);
        plan.length = 0;
        plan.push(...newPlan);
        await this._delay(500);
      }
    }
    this._busy = false;
  }
}
