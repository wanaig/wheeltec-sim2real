/**
 * SceneLayouts.js
 * 工业工具场景布局 + 工具3D模型工厂 + 数据集标注信息。
 *
 * 三大功能:
 *  1. createToolMesh(cls) — 构建5种工业工具的3D模型 (从MockAgent提取)
 *  2. LAYOUTS — 有序摆放 / 混杂摆放 / 随机生成 三种场景布局
 *  3. TOOL_BBOX / TOOL_CLASS_ID — 供 DatasetGenerator 计算 YOLO 标注
 */
import * as THREE from 'three';

// ─────────────── 工具元数据 ───────────────

export const TOOL_CLASSES = ['screwdriver', 'wrench', 'nut', 'roller', 'screw'];

export const TOOL_CN = {
  screwdriver: '螺丝刀', wrench: '扳手', nut: '螺母',
  roller: '滚柱', screw: '螺丝',
};

// YOLO 类别 ID (与训练 data/instruction_dataset_300.jsonl 对齐)
export const TOOL_CLASS_ID = {
  screwdriver: 0, wrench: 1, nut: 2, roller: 3, screw: 4,
};

// 工具模型统一缩放因子 (与缩小后的工作台尺寸匹配)
const TOOL_SCALE = 0.5;

// 工具 3D 包围盒 (X宽 × Y长 × Z高), 供边界框投影计算
export const TOOL_BBOX = {
  screwdriver: { x: 0.016, y: 0.0565, z: 0.016 },
  wrench:      { x: 0.012, y: 0.047, z: 0.0035 },
  nut:         { x: 0.014, y: 0.014, z: 0.004 },
  roller:      { x: 0.018, y: 0.0255, z: 0.018 },
  screw:       { x: 0.007, y: 0.013, z: 0.002 },
};

// 工具最大半径 (Z方向), 用于放置时防穿模: z = benchTopZ + radius
export const TOOL_MAX_R = {
  screwdriver: 0.008, wrench: 0.002, nut: 0.007, roller: 0.009, screw: 0.0035,
};

// ─────────────── 材质 ───────────────

const MAT = {
  metal:   new THREE.MeshStandardMaterial({ color: 0xA8A8B8, metalness: 0.85, roughness: 0.18 }),
  steel:   new THREE.MeshStandardMaterial({ color: 0x6A6A7A, metalness: 0.80, roughness: 0.25 }),
  brass:   new THREE.MeshStandardMaterial({ color: 0xC8A830, metalness: 0.70, roughness: 0.30 }),
  plastic: new THREE.MeshStandardMaterial({ color: 0x2255CC, metalness: 0.0,  roughness: 0.35 }),
  dark:    new THREE.MeshStandardMaterial({ color: 0x181818, metalness: 0.50, roughness: 0.40 }),
};

// ─────────────── 工具 3D 模型工厂 ───────────────

export function createToolMesh(cls) {
  let g;
  switch (cls) {
    case 'screwdriver': g = createScrewdriver(); break;
    case 'wrench':      g = createWrench(); break;
    case 'nut':         g = createNut(); break;
    case 'roller':      g = createRoller(); break;
    case 'screw':       g = createScrew(); break;
    default: g = new THREE.Group();
  }
  g.scale.setScalar(TOOL_SCALE);
  return g;
}

function createScrewdriver() {
  const g = new THREE.Group();
  const handle = new THREE.Mesh(
    new THREE.CylinderGeometry(0.014, 0.016, 0.05, 20), MAT.plastic);
  handle.position.y = 0.025; g.add(handle);
  for (let i = 0; i < 3; i++) {
    const groove = new THREE.Mesh(
      new THREE.TorusGeometry(0.015, 0.0015, 6, 16),
      new THREE.MeshStandardMaterial({ color: 0x113388, roughness: 0.3 }));
    groove.position.y = 0.012 + i * 0.012;
    groove.rotation.x = Math.PI / 2;
    g.add(groove);
  }
  const shaft = new THREE.Mesh(
    new THREE.CylinderGeometry(0.003, 0.003, 0.055, 12), MAT.metal);
  shaft.position.y = -0.027; g.add(shaft);
  const tip = new THREE.Mesh(
    new THREE.ConeGeometry(0.003, 0.008, 8), MAT.metal);
  tip.position.y = -0.059; g.add(tip);
  return g;
}

function createWrench() {
  const g = new THREE.Group();
  const handle = new THREE.Mesh(
    new THREE.BoxGeometry(0.007, 0.07, 0.003), MAT.steel);
  g.add(handle);
  const ring1 = new THREE.Mesh(
    new THREE.TorusGeometry(0.012, 0.0035, 8, 20), MAT.steel);
  ring1.position.y = 0.044; g.add(ring1);
  const ring2 = new THREE.Mesh(
    new THREE.TorusGeometry(0.010, 0.0035, 8, 20), MAT.steel);
  ring2.position.y = -0.044; g.add(ring2);
  return g;
}

function createNut() {
  const g = new THREE.Group();
  const nut = new THREE.Mesh(
    new THREE.CylinderGeometry(0.014, 0.014, 0.008, 6), MAT.brass);
  g.add(nut);
  const hole = new THREE.Mesh(
    new THREE.CylinderGeometry(0.006, 0.006, 0.009, 12), MAT.dark);
  g.add(hole);
  return g;
}

function createRoller() {
  const g = new THREE.Group();
  const cyl = new THREE.Mesh(
    new THREE.CylinderGeometry(0.016, 0.016, 0.045, 24), MAT.steel);
  g.add(cyl);
  for (const y of [-0.023, 0.023]) {
    const cap = new THREE.Mesh(
      new THREE.CylinderGeometry(0.018, 0.018, 0.003, 24), MAT.metal);
    cap.position.y = y; g.add(cap);
  }
  return g;
}

function createScrew() {
  const g = new THREE.Group();
  const head = new THREE.Mesh(
    new THREE.CylinderGeometry(0.007, 0.007, 0.004, 16), MAT.metal);
  head.position.y = 0.014; g.add(head);
  for (const r of [0, Math.PI / 2]) {
    const slot = new THREE.Mesh(
      new THREE.BoxGeometry(0.010, 0.0015, 0.0015), MAT.dark);
    slot.position.y = 0.017;
    slot.rotation.z = r;
    g.add(slot);
  }
  const shaft = new THREE.Mesh(
    new THREE.CylinderGeometry(0.0025, 0.0025, 0.022, 10), MAT.metal);
  shaft.position.y = -0.001; g.add(shaft);
  return g;
}

// ─────────────── 场景布局 ───────────────

/**
 * 每个布局返回工具实例数组: [{ class, xyz, rot }]
 * xyz 为世界坐标, z = benchTopZ + TOOL_MAX_R[cls] (贴台面不穿模)
 * rot 为绕Z轴旋转弧度
 *
 * 双工作台布局:
 *   左台 (料箱): Y center=-0.30, Y range [-0.49, -0.11], benchD=0.38
 *   过道:        Y range [-0.11, +0.11] (22cm, 机械臂小车通道)
 *   右台 (工具): Y center=+0.30, Y range [+0.11, +0.49], benchD=0.38
 *   两台 X 相同: benchCx=0.30, benchW=0.15 (X range [0.225, 0.375])
 *
 * 工具全部放在右台 (Y > +0.11)
 */

const BENCH_CX = 0.30, BENCH_W = 0.15, BENCH_D = 0.38;

// 工具区 (右台) Y/X 范围
const TOOL_Y_MIN = 0.15, TOOL_Y_MAX = 0.45;
const TOOL_X_MIN = 0.255, TOOL_X_MAX = 0.345;

/**
 * 布局1: 有序摆放 (ordered)
 * 工具在右区整齐排列, 间距均匀, 角度统一
 */
function layoutOrdered(benchTopZ) {
  const tools = [];
  const startX = 0.26, gapX = 0.02;
  const classes = ['screw', 'nut', 'screwdriver', 'wrench', 'roller'];
  for (let i = 0; i < classes.length; i++) {
    const cls = classes[i];
    const x = startX + i * gapX;
    const y = (TOOL_Y_MIN + TOOL_Y_MAX) / 2;  // 右区居中
    const z = benchTopZ + TOOL_MAX_R[cls];
    tools.push({ class: cls, xyz: [x, y, z], rot: 0 });
  }
  return tools;
}

/**
 * 布局2: 混杂摆放 (cluttered)
 * 工具随机散落在右区, 随机角度, 部分重叠
 */
function layoutCluttered(benchTopZ) {
  const tools = [];
  const classes = ['screwdriver', 'wrench', 'nut', 'roller', 'screw'];
  for (const cls of classes) {
    const count = 1 + Math.floor(Math.random() * 2);  // 1或2个
    for (let i = 0; i < count; i++) {
      const x = TOOL_X_MIN + Math.random() * (TOOL_X_MAX - TOOL_X_MIN);
      const y = TOOL_Y_MIN + Math.random() * (TOOL_Y_MAX - TOOL_Y_MIN);
      const rot = Math.random() * Math.PI * 2;
      const z = benchTopZ + TOOL_MAX_R[cls];
      tools.push({ class: cls, xyz: [x, y, z], rot });
    }
  }
  return tools;
}

/**
 * 布局3: 多工具组合 (mixed)
 * 有序+散乱混合, 更多工具实例
 */
function layoutMixed(benchTopZ) {
  const tools = [];
  // 左列有序: 螺丝和螺母
  const small = ['screw', 'nut', 'screw', 'nut'];
  for (let i = 0; i < small.length; i++) {
    const cls = small[i];
    tools.push({
      class: cls,
      xyz: [0.26, TOOL_Y_MIN + i * 0.08, benchTopZ + TOOL_MAX_R[cls]],
      rot: (Math.random() - 0.5) * 0.5,
    });
  }
  // 右侧散乱: 螺丝刀、扳手、滚柱
  const big = ['screwdriver', 'wrench', 'roller'];
  for (let i = 0; i < big.length; i++) {
    const cls = big[i];
    const x = 0.26 + Math.random() * 0.08;
    const y = TOOL_Y_MIN + 0.04 + i * 0.12 + (Math.random() - 0.5) * 0.04;
    const rot = Math.random() * Math.PI * 2;
    tools.push({ class: cls, xyz: [x, y, benchTopZ + TOOL_MAX_R[cls]], rot });
  }
  return tools;
}

/**
 * 布局4: 随机生成 (random)
 * 随机数量(3~8), 随机类别, 随机位置, 随机角度
 */
function layoutRandom(benchTopZ, opts = {}) {
  const count = opts.count || (3 + Math.floor(Math.random() * 6));
  const tools = [];
  for (let i = 0; i < count; i++) {
    const cls = TOOL_CLASSES[Math.floor(Math.random() * TOOL_CLASSES.length)];
    const x = TOOL_X_MIN + Math.random() * (TOOL_X_MAX - TOOL_X_MIN);
    const y = TOOL_Y_MIN + Math.random() * (TOOL_Y_MAX - TOOL_Y_MIN);
    const rot = Math.random() * Math.PI * 2;
    const z = benchTopZ + TOOL_MAX_R[cls];
    tools.push({ class: cls, xyz: [x, y, z], rot });
  }
  return tools;
}

export const LAYOUTS = {
  ordered: layoutOrdered,
  cluttered: layoutCluttered,
  mixed: layoutMixed,
  random: layoutRandom,
};

export const LAYOUT_NAMES = ['ordered', 'cluttered', 'mixed', 'random'];

export const LAYOUT_CN = {
  ordered: '有序摆放', cluttered: '混杂摆放',
  mixed: '混合场景', random: '随机生成',
};

/** 生成指定布局的工具列表 */
export function generateLayout(name, benchTopZ, opts) {
  const fn = LAYOUTS[name] || layoutCluttered;
  return fn(benchTopZ, opts);
}

export { MAT, BENCH_CX, BENCH_W, BENCH_D };
