/**
 * SceneSetup.js
 * Three.js 场景基础设施：渲染器、相机(正/侧/自由)、灯光、地面网格、坐标轴。
 * 坐标系保持 ROS 标准 Z-up，camera.up = (0,0,1)。
 */
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';

export class SceneSetup {
  constructor(canvas) {
    this.canvas = canvas;

    // ── 渲染器 ──
    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.setSize(window.innerWidth, window.innerHeight);
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;

    // ── 场景 ──
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x1a1a2e);

    // ── 相机 (Z-up, 透视) ──
    this.camera = new THREE.PerspectiveCamera(
      50, window.innerWidth / window.innerHeight, 0.01, 100
    );
    this.camera.up.set(0, 0, 1); // ROS: Z 朝上
    this.camera.position.set(1.2, -1.0, 0.8);
    this.camera.lookAt(0, 0, 0.15);

    // ── 控制器 ──
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.12;
    this.controls.target.set(0, 0, 0.15);
    this.controls.minDistance = 0.15;
    this.controls.maxDistance = 8;
    this.controls.minPolarAngle = 0.1;          // 防止正上方万向锁
    this.controls.maxPolarAngle = Math.PI / 2 - 0.02;  // 不穿越地面

    this._buildLights();
    this._buildGround();
    this._buildAxes();

    window.addEventListener('resize', () => this.onResize());
  }

  _buildLights() {
    // 环境光
    this.scene.add(new THREE.AmbientLight(0xffffff, 0.55));

    // 主平行光 (带阴影)
    const dir = new THREE.DirectionalLight(0xffffff, 0.9);
    dir.position.set(1.5, -1.5, 2.5);
    dir.castShadow = true;
    dir.shadow.mapSize.set(2048, 2048);
    dir.shadow.camera.near = 0.1;
    dir.shadow.camera.far = 10;
    dir.shadow.camera.left = -2.5;
    dir.shadow.camera.right = 2.5;
    dir.shadow.camera.top = 2.5;
    dir.shadow.camera.bottom = -2.5;
    dir.shadow.bias = -0.0005;
    this.scene.add(dir);

    // 补光
    const fill = new THREE.DirectionalLight(0x88aaff, 0.3);
    fill.position.set(-1, 1, 1);
    this.scene.add(fill);
  }

  _buildGround() {
    // 地面平面 (接收阴影)
    // Z-up 坐标系: PlaneGeometry 默认在 XY 平面, 即水平地面, 无需旋转
    const groundGeo = new THREE.PlaneGeometry(6, 6);
    const groundMat = new THREE.MeshPhongMaterial({
      color: 0x222233,
      shininess: 10,
      side: THREE.DoubleSide,
    });
    this.ground = new THREE.Mesh(groundGeo, groundMat);
    this.ground.receiveShadow = true;
    this.scene.add(this.ground);

    // 网格线 (5cm 细网格 + 50cm 粗网格)
    // GridHelper 默认在 XZ 平面 (Y-up 惯例), Z-up 需旋转 PI/2 到 XY 水平面
    const grid = new THREE.GridHelper(6, 120, 0x4488ff, 0x2a2a3e);
    grid.rotation.x = Math.PI / 2;
    this.scene.add(grid);

    const grid2 = new THREE.GridHelper(6, 12, 0x66aaff, 0x66aaff);
    grid2.rotation.x = Math.PI / 2;
    this.scene.add(grid2);
  }

  _buildAxes() {
    // 世界坐标系轴 (红=X, 绿=Y, 蓝=Z), 长度 0.3m
    const axes = new THREE.AxesHelper(0.3);
    axes.position.set(0, 0, 0.001);
    this.scene.add(axes);
  }

  /** 添加物体到场景 */
  add(obj) { this.scene.add(obj); }

  /** 移除 */
  remove(obj) { this.scene.remove(obj); }

  onResize() {
    this.camera.aspect = window.innerWidth / window.innerHeight;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(window.innerWidth, window.innerHeight);
  }

  /** 渲染一帧 */
  render() {
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  }
}
