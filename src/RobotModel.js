/**
 * RobotModel.js
 * 从 URDF (mini_4wd_six_arm.urdf) 精确重建运动学树，加载真实 STL 网格。
 * 这是 sim2real 的核心：仿真模型的连杆/关节/位姿与真机完全一致。
 *
 * 坐标系：ROS 标准 Z-up 右手系（X前 Y左 Z上），单位米。
 * 所有关节 origin / axis / limit 直接取自 URDF，无任何近似。
 */
import * as THREE from 'three';
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js';

// ────────────────────────── URDF 连杆定义 ──────────────────────────
// color: URDF <material><color rgba="r g b a">
const LINKS = {
  base_link:              { mesh: 'base_link.STL',              color: [1, 1, 1] },
  left_front_wheel_link:  { mesh: 'left_front_wheel_link.STL',  color: [1, 1, 1] },
  left_rear_wheel_link:   { mesh: 'left_rear_wheel_link.STL',   color: [1, 1, 1] },
  right_front_wheel_link: { mesh: 'right_front_wheel_link.STL', color: [1, 1, 1] },
  right_rear_wheel_link:  { mesh: 'right_rear_wheel_link.STL',  color: [1, 1, 1] },
  link1: { mesh: 'link1.STL', color: [0.741, 0.902, 0.957] },
  link2: { mesh: 'link2.STL', color: [0.741, 0.902, 0.957] },
  link3: { mesh: 'link3.STL', color: [0.741, 0.902, 0.957] },
  link4: { mesh: 'link4.STL', color: [0.741, 0.902, 0.957] },
  link5: { mesh: 'link5.STL', color: [0.741, 0.902, 0.957] },
  link6: { mesh: 'link6.STL', color: [1, 1, 1] },
  link7: { mesh: 'link7.STL', color: [0, 0.137, 1] },
  link8: { mesh: 'link8.STL', color: [0.749, 0.749, 0.749] },
  link9: { mesh: 'link9.STL', color: [1, 1, 1] },
  link10: { mesh: 'link10.STL', color: [1, 1, 1] },
  link11: { mesh: 'link11.STL', color: [1, 1, 1] },
};

// ────────────────────────── URDF 关节定义 ──────────────────────────
// xyz: joint origin 平移 (米)
// axis: 旋转轴 (joint frame)
// type: continuous(无限转) | revolute(有限转)
// limit: [lower, upper] rad (仅 revolute)
// parent → child
const JOINTS = [
  // 四个车轮 (continuous, 绕 Y 轴)
  { name: 'left_front_wheel_joint',  parent: 'base_link', child: 'left_front_wheel_link',  xyz: [ 0.094976,  0.076703, 0.038001], axis: [0,1,0], type: 'continuous' },
  { name: 'left_rear_wheel_joint',   parent: 'base_link', child: 'left_rear_wheel_link',   xyz: [-0.078024,  0.076703, 0.038001], axis: [0,1,0], type: 'continuous' },
  { name: 'right_front_wheel_joint', parent: 'base_link', child: 'right_front_wheel_link', xyz: [ 0.094976, -0.075297, 0.038001], axis: [0,1,0], type: 'continuous' },
  { name: 'right_rear_wheel_joint',  parent: 'base_link', child: 'right_rear_wheel_link',  xyz: [-0.078024, -0.075297, 0.038001], axis: [0,1,0], type: 'continuous' },
  // 机械臂 5 自由度 (arm group: base_link → link5)
  { name: 'joint1', parent: 'base_link', child: 'link1', xyz: [ 0.054476, 0.00070272, 0.156   ], axis: [0,0,-1], type: 'revolute', limit: [-1.57, 1.57] },
  { name: 'joint2', parent: 'link1', child: 'link2', xyz: [ 0.0005,    0.0227,    0.0315  ], axis: [0,-1,0], type: 'revolute', limit: [-1.57, 1.57] },
  { name: 'joint3', parent: 'link2', child: 'link3', xyz: [ 0,         0.001,     0.105   ], axis: [0,1,0],  type: 'revolute', limit: [-1.57, 1.57] },
  { name: 'joint4', parent: 'link3', child: 'link4', xyz: [ 0.0010378,-0.0015,    0.0975  ], axis: [0,1,0],  type: 'revolute', limit: [-0.8,  1.57] },
  { name: 'joint5', parent: 'link4', child: 'link5', xyz: [-0.027,    -0.0222,    0.0432  ], axis: [0,0,-1], type: 'revolute', limit: [-1.57, 1.57] },
  // 夹爪 (hand group: joint6~11, 从 link5 分支)
  { name: 'joint6',  parent: 'link5', child: 'link6',  xyz: [-0.005,   -0.0125,   0.036042], axis: [1,0,0],  type: 'revolute', limit: [-0.8, 0.8] },
  { name: 'joint7',  parent: 'link6', child: 'link7',  xyz: [-0.007,   -0.024099, 0.025382], axis: [1,0,0],  type: 'revolute', limit: [-0.8, 0.8] },
  { name: 'joint8',  parent: 'link5', child: 'link8',  xyz: [-0.0095,   0.0125,   0.036042], axis: [-1,0,0], type: 'revolute', limit: [-0.8, 0.8] },
  { name: 'joint9',  parent: 'link8', child: 'link9',  xyz: [-0.0025,   0.024471, 0.025023], axis: [1,0,0],  type: 'revolute', limit: [-0.8, 0.8] },
  { name: 'joint10', parent: 'link5', child: 'link10', xyz: [-0.008,   -0.032703, 0.028688], axis: [1,0,0],  type: 'revolute', limit: [-0.8, 0.8] },
  { name: 'joint11', parent: 'link5', child: 'link11', xyz: [-0.008,    0.032703, 0.028688], axis: [-1,0,0], type: 'revolute', limit: [-0.8, 0.8] },
];

// ────────────────────────── 预设位姿 (来自 SRDF group_state) ──────────────────────────
export const PRESETS = {
  arm_home:            { joint1: 0,    joint2: 0,    joint3: 0,    joint4: 0,    joint5: 0 },
  arm_uplift:          { joint1: 0,    joint2: 0.54, joint3: 1.57, joint4: 1.57, joint5: 0 },
  arm_place:           { joint1: 0,    joint2: 0.14, joint3: 1.57, joint4: 1.57, joint5: 0 },
  arm_clamp:           { joint1: 0,    joint2: -1.1, joint3: 0.66, joint4: 1,    joint5: 0 },
  arm_rotate_uplift:   { joint1: 1.57, joint2: 0.57, joint3: 1.57, joint4: 1.3,  joint5: 0 },
  arm_rotate_put:      { joint1: 1.57, joint2: -1.1, joint3: 0.66, joint4: 1,    joint5: 0 },
};

export const GRIPPER_POSES = {
  open:  { joint6:  0.45, joint7: -0.45, joint8:  0.45, joint9:  0.45, joint10:  0.45, joint11:  0.45 },
  close: { joint6: -0.45, joint7:  0.45, joint8: -0.45, joint9: -0.45, joint10: -0.45, joint11: -0.45 },
};

// 用于 ROS /joint_states 话题的关节顺序 (与 controller_joint_names 一致)
export const ALL_JOINT_NAMES = [
  'left_front_wheel_joint', 'left_rear_wheel_joint',
  'right_front_wheel_joint', 'right_rear_wheel_joint',
  'joint1', 'joint2', 'joint3', 'joint4', 'joint5',
  'joint6', 'joint7', 'joint8', 'joint9', 'joint10', 'joint11',
];

// 机械臂关节 (IK / 滑块控制用)
export const ARM_JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5'];
export const GRIPPER_JOINT_NAMES = ['joint6', 'joint7', 'joint8', 'joint9', 'joint10', 'joint11'];
export const WHEEL_JOINT_NAMES = ['left_front_wheel_joint', 'left_rear_wheel_joint', 'right_front_wheel_joint', 'right_rear_wheel_joint'];

// 夹爪抓取中心 (TCP) 相对 link5 的固定偏移 (米)
// 由 link7/link9 指尖网格最远顶点在 link5 系取中点实测得到 ≈ (-0.0012, 0, 0.128)
// IK 目标应为 TCP 而非 link5 原点, 否则夹爪会越过目标点约 6.7cm
export const GRIPPER_TCP = [-0.0012, 0, 0.128];

// SRDF 中已禁用碰撞的连杆对 (Adjacent / Never / Default)
// 取自 mini_4wd_six_arm.srdf, 这些对不需要检测
const SRDF_DISABLED = [
  ['base_link','left_front_wheel_link'],['base_link','left_rear_wheel_link'],
  ['base_link','link1'],['base_link','right_front_wheel_link'],['base_link','right_rear_wheel_link'],
  ['left_front_wheel_link','left_rear_wheel_link'],['left_front_wheel_link','link1'],
  ['left_front_wheel_link','link2'],['left_front_wheel_link','link3'],
  ['left_front_wheel_link','right_front_wheel_link'],['left_front_wheel_link','right_rear_wheel_link'],
  ['left_rear_wheel_link','link1'],['left_rear_wheel_link','link2'],['left_rear_wheel_link','link3'],
  ['left_rear_wheel_link','right_front_wheel_link'],['left_rear_wheel_link','right_rear_wheel_link'],
  ['link1','link10'],['link1','link11'],['link1','link2'],['link1','link3'],
  ['link1','link5'],['link1','link6'],['link1','link7'],['link1','link8'],['link1','link9'],
  ['link1','right_front_wheel_link'],['link1','right_rear_wheel_link'],
  ['link10','link11'],['link10','link2'],['link10','link3'],['link10','link4'],
  ['link10','link5'],['link10','link7'],['link10','link8'],['link10','link9'],
  ['link11','link2'],['link11','link3'],['link11','link4'],['link11','link5'],
  ['link11','link6'],['link11','link9'],
  ['link2','link3'],['link2','link5'],['link2','link6'],['link2','link7'],
  ['link2','link8'],['link2','link9'],['link2','right_front_wheel_link'],['link2','right_rear_wheel_link'],
  ['link3','link4'],['link3','link5'],['link3','link6'],['link3','link7'],
  ['link3','link8'],['link3','link9'],['link3','right_front_wheel_link'],['link3','right_rear_wheel_link'],
  ['link4','link5'],['link4','link6'],['link4','link7'],['link4','link8'],['link4','link9'],
  ['link5','link6'],['link5','link7'],['link5','link8'],['link5','link9'],
  ['link6','link7'],['link6','link8'],
  ['link8','link9'],
  ['right_front_wheel_link','right_rear_wheel_link'],
];

const MESH_BASE = '/meshes/';

function pairKey(a, b) { return a < b ? a + '|' + b : b + '|' + a; }

// 地面碰撞检测的连杆 (机械臂 + 夹爪, 不含底盘和车轮)
const GROUND_CHECK_LINKS = [
  'link1','link2','link3','link4','link5',
  'link6','link7','link8','link9','link10','link11',
];

export class RobotModel {
  constructor() {
    this.root = new THREE.Group();          // 整个机器人根节点 (== base_link frame)
    this.root.name = 'mini_4wd_six_arm';
    this.jointGroups = {};                  // name → THREE.Group (关节 frame)
    this.jointAxes = {};                    // name → THREE.Vector3
    this.jointLimits = {};                  // name → [lower, upper] | null
    this.jointTypes = {};                   // name → 'continuous' | 'revolute'
    this.jointValues = {};                  // name → 当前角度 (rad)
    this.linkMeshes = {};                   // linkName → THREE.Mesh
    this.loader = new STLLoader();
    this._linkByJointChild = {};            // childLink → jointName
    for (const j of JOINTS) this._linkByJointChild[j.child] = j.name;
    // 自碰撞检测
    this.collisionEnabled = true;           // 总开关
    this.collisionShrink = 0.65;            // 包围球收缩系数 (越小越宽松)
    this.boundingSpheres = {};              // linkName → { center: Vector3, radius: number }
    this.boundingBoxes = {};               // linkName → { min: Vector3, max: Vector3 }
    this._skipPairs = new Set();            // 不检测的连杆对
    this._allLinks = Object.keys(LINKS);
    this.onCollision = null;                // (pair:[a,b]) => void
    for (const [a, b] of SRDF_DISABLED) this._skipPairs.add(pairKey(a, b));

    // 关节插值 (平滑动作) — 由 tweenTo 启动, update() 每帧推进
    this._tween = null;                     // { names, from, to, elapsed, duration, onDone }
    this._lastTime = performance.now();
  }

  /** 异步加载所有 STL 网格并构建运动学树 */
  async load() {
    // 1. 并行加载全部 STL
    const geomMap = {};
    const entries = Object.entries(LINKS);
    await Promise.all(entries.map(async ([name, info]) => {
      const geo = await this.loader.loadAsync(MESH_BASE + info.mesh);
      geo.computeVertexNormals();
      geomMap[name] = geo;
    }));

    // 2. 构建连杆 mesh
    for (const [name, info] of entries) {
      const mat = new THREE.MeshPhongMaterial({
        color: new THREE.Color(info.color[0], info.color[1], info.color[2]),
        shininess: 80,
        specular: 0x444444,
      });
      const mesh = new THREE.Mesh(geomMap[name], mat);
      mesh.castShadow = true;
      mesh.receiveShadow = true;
      mesh.name = 'mesh_' + name;
      this.linkMeshes[name] = mesh;
    }

    // 3. 构建运动学树
    //    base_link 的 mesh 直接挂到 root
    this.root.add(this.linkMeshes['base_link']);

    //    为每个关节创建 Group，挂到对应父连杆的 Group 下
    //    父连杆的 "Group" = 关节 Group (如果有 joint 以该 link 为 child)
    //    若 link 是 base_link, 父 Group 就是 root
    for (const j of JOINTS) {
      const g = new THREE.Group();
      g.position.set(j.xyz[0], j.xyz[1], j.xyz[2]);
      g.name = j.name;
      this.jointGroups[j.name] = g;
      this.jointAxes[j.name] = new THREE.Vector3(j.axis[0], j.axis[1], j.axis[2]).normalize();
      this.jointTypes[j.name] = j.type;
      this.jointLimits[j.name] = j.type === 'revolute' ? j.limit : null;
      this.jointValues[j.name] = 0;

      // 找父连杆的挂载点: 若该父连杆是某个 joint 的 child, 则挂到该 joint Group;
      // 否则 (base_link) 挂到 root
      const parentJoint = this._linkByJointChild[j.parent];
      const parentGroup = parentJoint ? this.jointGroups[parentJoint] : this.root;
      parentGroup.add(g);

      // 子连杆 mesh 挂到关节 Group (visual origin = 0,0,0)
      g.add(this.linkMeshes[j.child]);
    }

    // 4. 初始化所有关节角度到 0
    for (const name of Object.keys(this.jointGroups)) {
      this._applyJoint(name);
    }
    // 5. 计算每个连杆的包围球+包围盒 (用于自碰撞和地面碰撞检测)
    this._computeBoundingSpheres();
    return this;
  }

  /**
   * 计算各连杆 STL 几何的包围球 (局部坐标), 并记录归零位时各对的距离。
   * 用于解决大体积连杆(如 base_link 车壳)包围球过大致使初始就重叠的误报:
   * 若某对在归零位已重叠, 碰撞阈值改为初始距离的 60% 而非半径和。
   */
  _computeBoundingSpheres() {
    for (const [name, mesh] of Object.entries(this.linkMeshes)) {
      const geo = mesh.geometry;
      if (!geo.boundingSphere) geo.computeBoundingSphere();
      const bs = geo.boundingSphere;
      this.boundingSpheres[name] = {
        center: bs.center.clone(),
        radius: bs.radius,
      };
      // 包围盒 (局部 AABB), 用于地面碰撞
      if (!geo.boundingBox) geo.computeBoundingBox();
      this.boundingBoxes[name] = {
        min: geo.boundingBox.min.clone(),
        max: geo.boundingBox.max.clone(),
      };
    }
    // 记录归零位 (当前全 0) 各检测对的球心世界距离
    this._homeDistances = {};
    this.root.updateMatrixWorld(true);
    const tmpA = new THREE.Vector3();
    const tmpB = new THREE.Vector3();
    const links = this._allLinks;
    for (let i = 0; i < links.length; i++) {
      const a = links[i];
      const bsA = this.boundingSpheres[a];
      if (!bsA) continue;
      for (let j = i + 1; j < links.length; j++) {
        const b = links[j];
        const key = pairKey(a, b);
        if (this._skipPairs.has(key)) continue;
        const bsB = this.boundingSpheres[b];
        if (!bsB) continue;
        tmpA.copy(bsA.center).applyMatrix4(this.linkMeshes[a].matrixWorld);
        tmpB.copy(bsB.center).applyMatrix4(this.linkMeshes[b].matrixWorld);
        this._homeDistances[key] = tmpA.distanceTo(tmpB);
      }
    }
  }

  /**
   * 自碰撞检测: 检查所有非禁用连杆对的包围球是否重叠。
   * 动态阈值: 若归零位已重叠(大车壳 vs 近臂连杆), 用初始距离×60%;
   *          否则用 (半径A+半径B)×shrink。
   * @returns {string[]|null} 碰撞连杆对 [a, b], 无碰撞返回 null
   */
  _checkSelfCollision() {
    this.root.updateMatrixWorld(true);
    const tmpA = new THREE.Vector3();
    const tmpB = new THREE.Vector3();
    const links = this._allLinks;
    const shrink = this.collisionShrink;

    for (let i = 0; i < links.length; i++) {
      const a = links[i];
      const bsA = this.boundingSpheres[a];
      if (!bsA) continue;
      const meshA = this.linkMeshes[a];

      for (let j = i + 1; j < links.length; j++) {
        const b = links[j];
        const key = pairKey(a, b);
        if (this._skipPairs.has(key)) continue;
        const bsB = this.boundingSpheres[b];
        if (!bsB) continue;

        tmpA.copy(bsA.center).applyMatrix4(meshA.matrixWorld);
        tmpB.copy(bsB.center).applyMatrix4(this.linkMeshes[b].matrixWorld);

        const dist = tmpA.distanceTo(tmpB);
        const sphereThresh = (bsA.radius + bsB.radius) * shrink;
        const home = this._homeDistances[key];

        // 归零位就重叠的对: 需要比初始再近 40% 才算碰撞
        let threshold;
        if (home !== undefined && home < sphereThresh) {
          threshold = home * 0.6;
        } else {
          threshold = sphereThresh;
        }

        if (dist < threshold) {
          return [a, b];
        }
      }
    }
    return null;
  }

  /**
   * 地面碰撞检测: 检查机械臂+夹爪各连杆的包围盒最低点是否低于地面 z=0。
   * 用包围盒 8 顶点变换到世界坐标取最小 Z, 比包围球更精确。
   * @returns {string|null} 碰撞连杆名, 无碰撞返回 null
   */
  _checkGroundCollision() {
    this.root.updateMatrixWorld(true);
    const corner = new THREE.Vector3();
    for (const name of GROUND_CHECK_LINKS) {
      const bb = this.boundingBoxes[name];
      if (!bb) continue;
      const mesh = this.linkMeshes[name];
      const mins = [bb.min.x, bb.min.y, bb.min.z];
      const maxs = [bb.max.x, bb.max.y, bb.max.z];
      let minZ = Infinity;
      for (let i = 0; i < 8; i++) {
        corner.set(
          (i & 1) ? maxs[0] : mins[0],
          (i & 2) ? maxs[1] : mins[1],
          (i & 4) ? maxs[2] : mins[2],
        );
        corner.applyMatrix4(mesh.matrixWorld);
        if (corner.z < minZ) minZ = corner.z;
      }
      if (minZ < 0) return name;
    }
    return null;
  }

  /** 综合: 自碰撞 + 地面碰撞 */
  _checkCollision() {
    const self = this._checkSelfCollision();
    if (self) return self;
    const ground = this._checkGroundCollision();
    if (ground) return ['ground', ground];
    return null;
  }
  _applyJoint(name) {
    const g = this.jointGroups[name];
    const axis = this.jointAxes[name];
    const v = this.jointValues[name];
    const q = new THREE.Quaternion().setFromAxisAngle(axis, v);
    g.quaternion.copy(q);
  }

  /**
   * 设置单个关节角度 (自动 clamp + 自碰撞检测)
   * @returns {boolean} true=移动成功, false=因碰撞被拒绝
   */
  setJoint(name, rad) {
    if (!(name in this.jointGroups)) return true;
    const lim = this.jointLimits[name];
    if (lim) rad = Math.max(lim[0], Math.min(lim[1], rad));

    // 车轮(continuous)不检测碰撞
    if (this.jointTypes[name] === 'continuous' || !this.collisionEnabled) {
      this.jointValues[name] = rad;
      this._applyJoint(name);
      return true;
    }

    // 试探性更新
    const oldVal = this.jointValues[name];
    this.jointValues[name] = rad;
    this._applyJoint(name);

    // 碰撞检测 (自碰撞 + 地面碰撞)
    const col = this._checkCollision();
    if (col) {
      this.jointValues[name] = oldVal;
      this._applyJoint(name);
      if (this.onCollision) this.onCollision(col);
      return false;
    }
    return true;
  }

  /**
   * 批量设置关节 (一次性更新 + 单次碰撞检测)
   * @returns {boolean} true=全部成功, false=因碰撞全部回退
   */
  setJoints(map) {
    const oldValues = {};
    for (const [k, v] of Object.entries(map)) {
      if (!(k in this.jointGroups)) continue;
      const lim = this.jointLimits[k];
      let val = v;
      if (lim) val = Math.max(lim[0], Math.min(lim[1], val));
      oldValues[k] = this.jointValues[k];
      this.jointValues[k] = val;
      this._applyJoint(k);
    }

    if (this.collisionEnabled) {
      const col = this._checkCollision();
      if (col) {
        for (const [k, v] of Object.entries(oldValues)) {
          this.jointValues[k] = v;
          this._applyJoint(k);
        }
        if (this.onCollision) this.onCollision(col);
        return false;
      }
    }
    return true;
  }

  /** 获取关节当前角度 */
  getJoint(name) {
    return this.jointValues[name] ?? 0;
  }

  /** 获取全部关节角度 (按 ALL_JOINT_NAMES 顺序, 用于 ROS 发布) */
  getJointStateArray() {
    return ALL_JOINT_NAMES.map(n => this.jointValues[n] ?? 0);
  }

  /** 从 ROS /joint_states 数据更新 (sim ← real, 真机状态直接应用不做碰撞检测) */
  applyJointState(names, positions) {
    const wasEnabled = this.collisionEnabled;
    this.collisionEnabled = false;
    for (let i = 0; i < names.length; i++) {
      const n = names[i];
      if (n in this.jointGroups) {
        const lim = this.jointLimits[n];
        const v = positions[i];
        this.jointValues[n] = lim ? Math.max(lim[0], Math.min(lim[1], v)) : v;
        this._applyJoint(n);
      }
    }
    this.collisionEnabled = wasEnabled;
  }

  /**
   * 启动关节平滑插值: 从当前角度过渡到 targetMap (缓动 ease-in-out)。
   * 新调用会替换进行中的插值。碰撞检测在插值期间关闭以保证流畅。
   */
  tweenTo(targetMap, duration = 1.0, onDone = null) {
    const names = Object.keys(targetMap);
    const from = names.map(n => this.jointValues[n] ?? 0);
    const to = names.map(n => {
      const v = targetMap[n];
      const lim = this.jointLimits[n];
      return lim ? Math.max(lim[0], Math.min(lim[1], v)) : v;
    });
    this._tween = { names, from, to, elapsed: 0, duration: Math.max(0.05, duration), onDone };
  }

  cancelTween() { this._tween = null; }
  get isTweening() { return !!this._tween; }

  /** 每帧推进插值 (由渲染循环调用) */
  update() {
    const now = performance.now();
    const dt = Math.min((now - this._lastTime) / 1000, 0.05);
    this._lastTime = now;
    if (!this._tween) return;

    const t = this._tween;
    t.elapsed += dt;
    let a = t.duration > 0 ? t.elapsed / t.duration : 1;
    if (a > 1) a = 1;
    // ease-in-out
    const e = a < 0.5 ? 2 * a * a : 1 - Math.pow(-2 * a + 2, 2) / 2;
    const vals = new Array(t.names.length);
    for (let i = 0; i < t.names.length; i++) {
      vals[i] = t.from[i] + (t.to[i] - t.from[i]) * e;
    }
    this.applyJointState(t.names, vals);
    if (a >= 1) {
      const cb = t.onDone;
      this._tween = null;
      if (cb) cb();
    }
  }

  /** 获取末端 (link5 原点) 世界坐标 — 用于 IK 与显示 */
  getEndEffectorPosition() {
    const g = this.jointGroups['joint5'];
    if (!g) return new THREE.Vector3();
    const p = new THREE.Vector3();
    g.getWorldPosition(p);
    return p;
  }

  /**
   * 获取夹爪抓取中心 (TCP) 世界坐标。
   * TCP = link5 原点 + link5 旋转 × GRIPPER_TCP 偏移 (指尖最远点中点)。
   * IK 目标应为 TCP 而非 link5, 否则夹爪会越过目标点。
   */
  getGripperTCP() {
    const g = this.jointGroups['joint5'];
    if (!g) return new THREE.Vector3();
    const p = new THREE.Vector3();
    const q = new THREE.Quaternion();
    g.getWorldPosition(p);
    g.getWorldQuaternion(q);
    const tcp = new THREE.Vector3(GRIPPER_TCP[0], GRIPPER_TCP[1], GRIPPER_TCP[2]).applyQuaternion(q);
    return p.add(tcp);
  }

  /** 获取末端世界四元数 */
  getEndEffectorQuaternion() {
    const g = this.jointGroups['joint5'];
    if (!g) return new THREE.Quaternion();
    const q = new THREE.Quaternion();
    g.getWorldQuaternion(q);
    return q;
  }

  /** 应用预设位姿 (机械臂) */
  applyPreset(name) {
    const p = PRESETS[name];
    if (p) this.setJoints(p);
  }

  /** 应用夹爪开合 */
  applyGripper(name) {
    const p = GRIPPER_POSES[name];
    if (p) this.setJoints(p);
  }

  /** 获取关节限位信息 (供 UI 滑块) */
  getJointInfo(name) {
    return {
      type: this.jointTypes[name],
      limit: this.jointLimits[name],
      value: this.jointValues[name],
    };
  }
}
