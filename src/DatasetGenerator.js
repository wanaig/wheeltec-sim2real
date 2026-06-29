/**
 * DatasetGenerator.js
 * 工业场景数据集生成器。
 *
 * 功能:
 *   1. 多视角渲染 — 从俯视/侧视/斜视等角度截图
 *   2. YOLO标注   — 3D包围盒投影到2D屏幕, 输出归一化边界框
 *   3. 批量生成   — 多布局 × 多视角 = N张图像+标注
 *   4. 导出下载   — 打包为 images/*.jpg + labels/*.txt + dataset.yaml
 *
 * 用法:
 *   const gen = new DatasetGenerator(sceneSetup, mockAgent);
 *   await gen.generate({ layouts: ['ordered','cluttered'], viewsPerLayout: 8 });
 */
import * as THREE from 'three';
import {
  LAYOUTS, LAYOUT_NAMES, LAYOUT_CN, generateLayout,
  createToolMesh, TOOL_CN, TOOL_CLASS_ID, TOOL_BBOX,
} from './SceneLayouts.js';

// 多视角相机预设 (position, lookAt) — 适配双工作台(总Y跨度~1.0m)
const VIEW_PRESETS = [
  { name: 'top',       pos: [0.30, 0.00, 1.40], look: [0.30, 0.00, 0.06] },
  { name: 'front',     pos: [0.30, -1.00, 0.55], look: [0.30, 0.00, 0.10] },
  { name: 'left',      pos: [-0.50, -0.30, 0.55], look: [0.25, -0.30, 0.10] },
  { name: 'right',     pos: [1.10, +0.30, 0.55], look: [0.35, +0.30, 0.10] },
  { name: 'front-l',   pos: [0.10, -0.90, 0.60], look: [0.25, -0.10, 0.10] },
  { name: 'front-r',   pos: [0.60, -0.90, 0.60], look: [0.35, +0.10, 0.10] },
  { name: 'back-l',    pos: [0.10, 0.90, 0.60], look: [0.25, -0.10, 0.10] },
  { name: 'back-r',    pos: [0.60, 0.90, 0.60], look: [0.35, +0.10, 0.10] },
];

export class DatasetGenerator {
  constructor(sceneSetup, mockAgent) {
    this.scene = sceneSetup;
    this.mockAgent = mockAgent;
    this._progressCb = null;
  }

  onProgress(cb) { this._progressCb = cb; }

  _log(msg) {
    if (this._progressCb) this._progressCb(msg);
  }

  /**
   * 生成完整数据集
   * @param {Object} opts
   *   layouts: ['ordered','cluttered','mixed','random'] — 使用哪些布局
   *   viewsPerLayout: int — 每个布局渲染几个视角 (从VIEW_PRESETS取前N个)
   *   randomCount: int — random布局每次生成几个工具
   *   randomSeeds: int — random布局重复几次 (数据增强)
   *   imgWidth: int — 图像宽度
   *   imgHeight: int — 图像高度
   */
  async generate(opts = {}) {
    const layouts = opts.layouts || LAYOUT_NAMES;
    const viewsPerLayout = opts.viewsPerLayout || 8;
    const randomSeeds = opts.randomSeeds || 3;
    const imgW = opts.imgWidth || 640;
    const imgH = opts.imgHeight || 480;

    const benchTopZ = this.mockAgent._benchTopZ || 0.060;
    const samples = [];  // [{imageName, imageData, annotations}]

    let total = 0;
    for (const layout of layouts) {
      const seeds = layout === 'random' ? randomSeeds : 1;
      total += seeds * Math.min(viewsPerLayout, VIEW_PRESETS.length);
    }
    let done = 0;

    for (const layoutName of layouts) {
      const seeds = layoutName === 'random' ? randomSeeds : 1;
      for (let s = 0; s < seeds; s++) {
        // 生成布局
        const tools = generateLayout(layoutName, benchTopZ, { count: opts.randomCount });
        // 切换MockAgent场景
        this.mockAgent.applyLayout(tools);
        this._log(`[${LAYOUT_CN[layoutName]}] 种子${s + 1}/${seeds} → ${tools.length}个工具`);

        // 多视角渲染
        const views = VIEW_PRESETS.slice(0, Math.min(viewsPerLayout, VIEW_PRESETS.length));
        for (let vi = 0; vi < views.length; vi++) {
          const view = views[vi];
          const imgName = `${layoutName}_s${s}_${view.name}`;
          const result = this._captureView(view, tools, imgW, imgH);
          samples.push({ name: imgName, ...result });
          done++;
          this._log(`  视角 ${vi + 1}/${views.length} (${view.name}) ✓  标注${result.annotations.length}个`);
          // 让出主线程, 避免UI卡死
          await new Promise(r => setTimeout(r, 10));
        }
      }
    }

    this._log(`\n数据集生成完毕: ${samples.length}张图像`);
    this._exportAll(samples);
    return samples;
  }

  /**
   * 从指定视角渲染并计算标注
   */
  _captureView(view, tools, imgW, imgH) {
    const cam = this.scene.camera;
    const renderer = this.scene.renderer;
    const scene = this.scene.scene;

    // 保存原相机状态
    const origPos = cam.position.clone();
    const origAspect = cam.aspect;
    const origFov = cam.fov;

    // 设置渲染视角
    cam.position.set(...view.pos);
    cam.lookAt(new THREE.Vector3(...view.look));
    cam.aspect = imgW / imgH;
    cam.updateProjectionMatrix();

    // 设置渲染分辨率
    const origSize = new THREE.Vector2();
    renderer.getSize(origSize);
    renderer.setSize(imgW, imgH, false);

    // 渲染
    renderer.render(scene, cam);

    // 读取像素
    const gl = renderer.getContext();
    const pixels = new Uint8Array(imgW * imgH * 4);
    gl.readPixels(0, 0, imgW, imgH, gl.RGBA, gl.UNSIGNED_BYTE, pixels);

    // 翻转Y轴 (WebGL底左原点 → 图像顶左原点)
    const flipped = new Uint8Array(imgW * imgH * 4);
    for (let y = 0; y < imgH; y++) {
      const srcRow = (imgH - 1 - y) * imgW * 4;
      const dstRow = y * imgW * 4;
      flipped.set(pixels.subarray(srcRow, srcRow + imgW * 4), dstRow);
    }

    // 转 JPEG DataURL (用临时canvas)
    const cv = document.createElement('canvas');
    cv.width = imgW; cv.height = imgH;
    const ctx = cv.getContext('2d');
    const imgData = new ImageData(new Uint8ClampedArray(flipped), imgW, imgH);
    ctx.putImageData(imgData, 0, 0);
    const dataURL = cv.toDataURL('image/jpeg', 0.92);

    // 计算每个工具的2D边界框 (YOLO格式)
    const annotations = [];
    for (const t of tools) {
      const bbox = this._projectBBox(t.xyz, t.rot, TOOL_BBOX[t.class], cam, imgW, imgH);
      if (bbox) {
        annotations.push({
          classId: TOOL_CLASS_ID[t.class],
          className: t.class,
          ...bbox,
        });
      }
    }

    // 恢复原相机和渲染器
    cam.position.copy(origPos);
    cam.aspect = origAspect;
    cam.fov = origFov;
    cam.updateProjectionMatrix();
    renderer.setSize(origSize.x, origSize.y, false);

    return { imageData: dataURL, annotations };
  }

  /**
   * 3D包围盒 → 2D屏幕投影 → YOLO归一化边界框
   * 返回 {x, y, w, h} (归一化 0~1) 或 null (不在视野内)
   */
  _projectBBox(xyz, rot, bbox, camera, imgW, imgH) {
    // 包围盒8个角点 (工具局部坐标, 考虑旋转)
    const hx = bbox.x / 2, hy = bbox.y / 2, hz = bbox.z / 2;
    const corners = [
      [-hx, -hy, -hz], [hx, -hy, -hz], [hx, hy, -hz], [-hx, hy, -hz],
      [-hx, -hy,  hz], [hx, -hy,  hz], [hx, hy,  hz], [-hx, hy,  hz],
    ];
    const cosR = Math.cos(rot), sinR = Math.sin(rot);
    const worldCorners = corners.map(([x, y, z]) => {
      // Z轴旋转
      const rx = x * cosR - y * sinR;
      const ry = x * sinR + y * cosR;
      return new THREE.Vector3(xyz[0] + rx, xyz[1] + ry, xyz[2] + z);
    });

    // 投影到屏幕
    const projected = [];
    for (const c of worldCorners) {
      const p = c.clone().project(camera);
      // NDC (-1~1) → 像素 (0~imgW/H)
      const px = (p.x + 1) / 2 * imgW;
      const py = (1 - p.y) / 2 * imgH;
      const inFront = p.z < 1;  // z<1 在相机前方
      if (inFront) projected.push({ x: px, y: py });
    }

    if (projected.length === 0) return null;  // 完全在视野外

    // 取所有可见角点的min/max
    let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
    for (const p of projected) {
      minX = Math.min(minX, p.x);
      maxX = Math.max(maxX, p.x);
      minY = Math.min(minY, p.y);
      maxY = Math.max(maxY, p.y);
    }

    // 裁剪到图像边界
    minX = Math.max(0, minX);
    maxX = Math.min(imgW, maxX);
    minY = Math.max(0, minY);
    maxY = Math.min(imgH, maxY);

    const w = maxX - minX, h = maxY - minY;
    if (w < 2 || h < 2) return null;  // 太小, 忽略

    // YOLO格式: x_center, y_center, width, height (归一化)
    return {
      x: ((minX + maxX) / 2) / imgW,
      y: ((minY + maxY) / 2) / imgH,
      w: w / imgW,
      h: h / imgH,
    };
  }

  /**
   * 导出全部数据 (触发浏览器下载)
   */
  _exportAll(samples) {
    // 1. 生成 data.yaml (YOLO训练配置)
    const yaml = this._genYaml();
    // 2. 逐个下载图像和标注
    this._log('正在下载文件...');
    for (const s of samples) {
      // 图像
      this._download(s.imageData, `images/${s.name}.jpg`);
      // 标注
      const txt = s.annotations.map(a =>
        `${a.classId} ${a.x.toFixed(6)} ${a.y.toFixed(6)} ${a.w.toFixed(6)} ${a.h.toFixed(6)}`
      ).join('\n');
      this._downloadText(txt, `labels/${s.name}.txt`);
    }
    // data.yaml
    this._downloadText(yaml, `data.yaml`);
    this._log(`已下载 ${samples.length} 张图像 + 标注 + data.yaml`);
  }

  _genYaml() {
    const lines = [
      '# YOLO 工业工具检测数据集',
      `# 生成时间: ${new Date().toISOString()}`,
      `# 图像数: (见images目录)`,
      '',
      'path: ./dataset',
      'train: images',
      'val: images',
      '',
      'names:',
    ];
    for (const [cls, id] of Object.entries(TOOL_CLASS_ID)) {
      lines.push(`  ${id}: ${cls}`);
    }
    return lines.join('\n');
  }

  _download(dataURL, filename) {
    const a = document.createElement('a');
    a.href = dataURL;
    a.download = filename;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
  }

  _downloadText(text, filename) {
    const blob = new Blob([text], { type: 'text/plain' });
    const url = URL.createObjectURL(blob);
    this._download(url, filename);
    setTimeout(() => URL.revokeObjectURL(url), 1000);
  }

  /**
   * 仅预览单张 (不下载), 返回 {imageData, annotations}
   * 用于UI预览
   */
  preview(layoutName, viewIndex = 0, imgW = 640, imgH = 480) {
    const benchTopZ = this.mockAgent._benchTopZ || 0.060;
    const tools = generateLayout(layoutName, benchTopZ);
    this.mockAgent.applyLayout(tools);
    const view = VIEW_PRESETS[viewIndex] || VIEW_PRESETS[0];
    return this._captureView(view, tools, imgW, imgH);
  }
}
