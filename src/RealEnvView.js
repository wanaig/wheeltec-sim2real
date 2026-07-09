/**
 * RealEnvView.js — 真实环境 RGB-D 点云数字孪生
 *
 * 订阅 Astra S 的彩色图 + 8-bit 深度图 (通过 rosbridge),
 * 在浏览器端生成彩色 3D 点云, 放入 Three.js 场景。
 * 点云位置 = 相机光学坐标系 → 世界坐标系 (base_link), 由 RobotModel FK 计算。
 *
 * 数据流:
 *   /camera/color/image_raw/compressed  (JPEG, 彩色)
 *   /industrial_tools/depth_viz/compressed  (JPEG, 8-bit 深度, depth_m = pixel / 100)
 *   /camera/color/camera_info  (内参 fx/fy/cx/cy)
 *      ↓ 浏览器解码 + 逐像素 back-project → 3D 点 (光学坐标系)
 *      ↓ RobotModel.getCameraOpticalPose() → 世界坐标系
 *   THREE.Points (彩色点云, 实时更新)
 *
 * 用法 (main.js):
 *   const realEnv = new RealEnvView(scene, robot, ros);
 *   // 渲染循环: realEnv.update();
 */
import * as THREE from 'three';
import ROSLIB from 'roslib';

const DEPTH_SCALE = 100;      // 8-bit pixel → meters: depth_m = pixel / 100
const MIN_DEPTH_M = 0.05;
const MAX_DEPTH_M = 2.50;
const DOWNSAMPLE_STEP = 4;    // 每 4 像素取 1 点 (640×480 → ~19200 点)
const POINT_SIZE = 0.003;

export class RealEnvView {
  /**
   * @param {THREE.Scene} scene
   * @param {RobotModel} robot  — 用于 FK 计算相机世界位姿
   * @param {RosBridge} ros     — ROS 连接
   * @param {Object} mount      — 相机挂载参数 {x,y,z,roll,pitch,yaw}
   */
  constructor(scene, robot, ros, mount) {
    this.scene = scene;
    this.robot = robot;
    this.ros = ros;
    this.mount = mount || { x: 0.05, y: 0, z: 0.03, roll: 0, pitch: -0.3, yaw: 0 };

    this.points = null;
    this._colorImg = new Image();
    this._depthImg = new Image();
    this._colorCanvas = document.createElement('canvas');
    this._depthCanvas = document.createElement('canvas');
    this._colorCtx = this._colorCanvas.getContext('2d', { willReadFrequently: true });
    this._depthCtx = this._depthCanvas.getContext('2d', { willReadFrequently: true });

    this.fx = 0; this.fy = 0; this.cx = 0; this.cy = 0;
    this._colorReady = false;
    this._depthReady = false;
    this._intrinsicsReady = false;
    this._started = false;

    this._subColor = null;
    this._subDepth = null;
    this._subInfo = null;
  }

  /** 在 ROS 连接后调用 (update() 自动检测连接状态并启动) */
  _start() {
    if (this._started || !this.ros.connected || !this.ros.ros) return;
    const isROS2 = this.ros.version === 'ros2';
    const mtCompressed = isROS2 ? 'sensor_msgs/msg/CompressedImage' : 'sensor_msgs/CompressedImage';
    const mtCameraInfo = isROS2 ? 'sensor_msgs/msg/CameraInfo' : 'sensor_msgs/CameraInfo';

    this._subColor = new ROSLIB.Topic({
      ros: this.ros.ros,
      name: '/camera/color/image_raw/compressed',
      messageType: mtCompressed,
      throttle_rate: 100,
      queue_length: 1,
    });
    this._subColor.subscribe((msg) => this._onColor(msg));

    this._subDepth = new ROSLIB.Topic({
      ros: this.ros.ros,
      name: '/industrial_tools/depth_viz/compressed',
      messageType: mtCompressed,
      throttle_rate: 100,
      queue_length: 1,
    });
    this._subDepth.subscribe((msg) => this._onDepth(msg));

    this._subInfo = new ROSLIB.Topic({
      ros: this.ros.ros,
      name: '/camera/color/camera_info',
      messageType: mtCameraInfo,
      throttle_rate: 5000,
      queue_length: 1,
    });
    this._subInfo.subscribe((msg) => this._onCameraInfo(msg));

    this._started = true;
  }

  _stop() {
    if (this._subColor) { try { this._subColor.unsubscribe(); } catch (e) {} this._subColor = null; }
    if (this._subDepth) { try { this._subDepth.unsubscribe(); } catch (e) {} this._subDepth = null; }
    if (this._subInfo) { try { this._subInfo.unsubscribe(); } catch (e) {} this._subInfo = null; }
    this._started = false;
    this._colorReady = false;
    this._depthReady = false;
  }

  _onCameraInfo(msg) {
    if (msg.k && msg.k.length >= 9 && msg.k[0] > 0) {
      this.fx = msg.k[0];
      this.fy = msg.k[4];
      this.cx = msg.k[2];
      this.cy = msg.k[5];
      this._intrinsicsReady = true;
      if (this._subInfo) { try { this._subInfo.unsubscribe(); } catch (e) {} }
    }
  }

  _onColor(msg) {
    const fmt = (msg.format || 'jpeg').toLowerCase();
    const mime = fmt.includes('png') ? 'image/png' : 'image/jpeg';
    this._colorImg.onload = () => {
      if (this._colorCanvas.width !== this._colorImg.width)
        this._colorCanvas.width = this._colorImg.width;
      if (this._colorCanvas.height !== this._colorImg.height)
        this._colorCanvas.height = this._colorImg.height;
      this._colorCtx.drawImage(this._colorImg, 0, 0);
      this._colorReady = true;
    };
    this._colorImg.src = 'data:' + mime + ';base64,' + msg.data;
  }

  _onDepth(msg) {
    const fmt = (msg.format || 'jpeg').toLowerCase();
    const mime = fmt.includes('png') ? 'image/png' : 'image/jpeg';
    this._depthImg.onload = () => {
      if (this._depthCanvas.width !== this._depthImg.width)
        this._depthCanvas.width = this._depthImg.width;
      if (this._depthCanvas.height !== this._depthImg.height)
        this._depthCanvas.height = this._depthImg.height;
      this._depthCtx.drawImage(this._depthImg, 0, 0);
      this._depthReady = true;
    };
    this._depthImg.src = 'data:' + mime + ';base64,' + msg.data;
  }

  /** 每帧调用: 生成/更新点云 */
  update() {
    if (!this._started && this.ros.connected) this._start();
    if (!this._started || !this._colorReady || !this._depthReady || !this._intrinsicsReady) return;

    const w = this._colorCanvas.width;
    const h = this._colorCanvas.height;
    if (w === 0 || h === 0) return;
    if (this._depthCanvas.width !== w || this._depthCanvas.height !== h) return;

    const colorData = this._colorCtx.getImageData(0, 0, w, h).data;
    const depthData = this._depthCtx.getImageData(0, 0, w, h).data;

    const positions = [];
    const colors = [];
    const step = DOWNSAMPLE_STEP;

    for (let v = 0; v < h; v += step) {
      for (let u = 0; u < w; u += step) {
        const idx = (v * w + u) * 4;
        const depthPx = depthData[idx];
        if (depthPx === 0) continue;
        const d = depthPx / DEPTH_SCALE;
        if (d < MIN_DEPTH_M || d > MAX_DEPTH_M) continue;

        // 相机光学坐标系: X 右, Y 下, Z 前
        const x = (u - this.cx) * d / this.fx;
        const y = (v - this.cy) * d / this.fy;
        const z = d;

        positions.push(x, y, z);
        colors.push(
          colorData[idx] / 255,
          colorData[idx + 1] / 255,
          colorData[idx + 2] / 255
        );
      }
    }

    if (positions.length === 0) return;

    if (!this.points) {
      const geo = new THREE.BufferGeometry();
      const mat = new THREE.PointsMaterial({
        size: POINT_SIZE,
        vertexColors: true,
        sizeAttenuation: true,
      });
      this.points = new THREE.Points(geo, mat);
      this.scene.add(this.points);
    }

    this.points.geometry.setAttribute('position',
      new THREE.Float32BufferAttribute(positions, 3));
    this.points.geometry.setAttribute('color',
      new THREE.Float32BufferAttribute(colors, 3));
    this.points.geometry.computeBoundingSphere();

    // 用相机光学坐标系世界位姿定位点云
    const pose = this.robot.getCameraOpticalPose(this.mount);
    this.points.position.copy(pose.position);
    this.points.quaternion.copy(pose.quaternion);
  }

  /** 隐藏/显示点云 */
  setVisible(visible) {
    if (this.points) this.points.visible = visible;
  }

  /** 销毁 */
  dispose() {
    this._stop();
    if (this.points) {
      this.scene.remove(this.points);
      this.points.geometry.dispose();
      this.points.material.dispose();
      this.points = null;
    }
  }
}
