/**
 * ChassisController.js
 * 仿真模式下的底盘运动学控制 (4WD 差速转向)。
 *
 * - 键盘 WASD / 方向键 驱动虚拟小车在场景中移动
 * - 底盘方向按钮在无 ROS 时驱动仿真, 有 ROS 时同时发 cmd_vel
 * - 轮子根据线速度自动旋转 (视觉反馈)
 * - 连接 ROS 且开启同步时, 位姿由 /odom 覆盖
 */
import * as THREE from 'three';
import { WHEEL_JOINT_NAMES } from './RobotModel.js';

export class ChassisController {
  constructor(robot, ros) {
    this.robot = robot;
    this.ros = ros;

    // 运动参数
    this.maxLinear = 0.35;     // m/s
    this.maxAngular = 1.2;     // rad/s
    this.accel = 1.5;          // 线加速度 m/s²
    this.angAccel = 4.0;       // 角加速度 rad/s²
    this.damping = 0.85;       // 松开按键后衰减
    this.wheelRadius = 0.038;  // 轮半径 (URDF wheel joint z=0.038)

    // 当前速度
    this.v = 0;  // 线速度
    this.w = 0;  // 角速度

    // 输入状态
    this.keys = { forward: false, backward: false, left: false, right: false };

    // 是否由 odom 接管 (ROS 同步时)
    this.odomOverride = false;
    this._odomX = 0; this._odomY = 0; this._odomYaw = 0;

    // 自主导航 (IK 远距目标时先移动底盘至站位)
    this._navGoal = null;        // { x, y, yaw }
    this._navActive = false;
    this._navDist = 0;           // 剩余水平距离 (供 UI 显示)
    this._navTime = 0;           // 已用时间 (超时保护)
    this.onNavArrived = null;    // () => void  到达后回调

    this._lastTime = performance.now();
    this._lastCmdSent = 0;    // cmd_vel 节流 (20Hz, 避免 WebSocket 过载)
    this._bindKeys();
  }

  _bindKeys() {
    window.addEventListener('keydown', (e) => {
      // 输入框聚焦时不拦截
      if (e.target.tagName === 'INPUT') return;
      switch (e.key) {
        case 'ArrowUp':    this.keys.forward = true; e.preventDefault(); break;
        case 'ArrowDown':  this.keys.backward = true; e.preventDefault(); break;
        case 'ArrowLeft':  this.keys.left = true; e.preventDefault(); break;
        case 'ArrowRight': this.keys.right = true; e.preventDefault(); break;
      }
    });
    window.addEventListener('keyup', (e) => {
      switch (e.key) {
        case 'ArrowUp':    this.keys.forward = false; break;
        case 'ArrowDown':  this.keys.backward = false; break;
        case 'ArrowLeft':  this.keys.left = false; break;
        case 'ArrowRight': this.keys.right = false; break;
      }
    });
  }

  /** 按钮调用: 设定目标方向 (瞬时) */
  drive(forward, backward, left, right) {
    this.keys.forward = forward;
    this.keys.backward = backward;
    this.keys.left = left;
    this.keys.right = right;
  }

  /** 自动导航至指定底盘位姿 (世界系); 到达后触发 onNavArrived */
  navigateTo(x, y, yaw) {
    this._navGoal = { x, y, yaw };
    this._navActive = true;
    this._navDist = 0;
    this._navTime = 0;
    // 清空键入, 避免与导航速度冲突
    this.keys = { forward: false, backward: false, left: false, right: false };
  }

  /** 取消导航 */
  cancelNav() {
    this._navActive = false;
    this._navGoal = null;
    this.onNavArrived = null;
    this.v = 0;
    this.w = 0;
    if (this.ros.publishEnabled && this.ros.connected) this.ros.sendCmdVel(0, 0);
  }

  get navActive() { return this._navActive; }

  /** 计算导航目标速度 (比例控制器 + 制动限速, 抑制超调) */
  _computeNavVel(dt) {
    const pos = this.robot.root.position;
    const rot = this.robot.root.rotation.z;
    const g = this._navGoal;
    const dx = g.x - pos.x, dy = g.y - pos.y;
    const dist = Math.hypot(dx, dy);
    this._navDist = dist;

    const posTol = 0.015;  // 位置容差 1.5cm
    const yawTol = 0.035;  // 姿态容差 ~2°
    let targetV = 0, targetW = 0;
    let arrived = false;

    const norm = (a) => Math.atan2(Math.sin(a), Math.cos(a));
    const clampW = (w) => Math.max(-this.maxAngular, Math.min(this.maxAngular, w));

    if (dist > posTol) {
      const heading = Math.atan2(dy, dx);
      const yawErr = norm(heading - rot);
      const ay = Math.abs(yawErr);
      // 制动限速: 保证能在剩余距离内停下 (v² ≤ 2·a·d)
      const brakeV = Math.sqrt(2 * this.accel * Math.max(dist, 0)) * 0.9;
      let vmax = Math.min(this.maxLinear * 0.55, dist * 1.0, brakeV);
      if (dist < 0.12) vmax = Math.min(vmax, dist * 0.5);  // 末端蠕行
      if (ay > 2.5) {
        // 目标在车后方 (如过近需后退): 保持朝向倒车, 避免掉头
        const revYawErr = norm(heading + Math.PI - rot);
        targetV = -vmax;
        targetW = clampW(revYawErr * 2.0);
      } else if (ay > 0.25) {
        // 偏差大, 原地转向
        targetW = clampW(Math.sign(yawErr) * Math.min(this.maxAngular, ay * 2.0));
      } else {
        // 朝目标前进
        targetV = vmax;
        targetW = clampW(yawErr * 2.0);
      }
    } else {
      // 已到位, 对齐目标朝向
      const finalYawErr = norm(g.yaw - rot);
      if (Math.abs(finalYawErr) < yawTol) {
        arrived = true;
      } else {
        targetW = clampW(Math.sign(finalYawErr) * Math.min(this.maxAngular, Math.abs(finalYawErr) * 2.0));
      }
    }

    // 超时保护 (30s)
    this._navTime += dt;
    if (this._navTime > 30) arrived = true;

    return { v: targetV, w: targetW, arrived };
  }

  /** ROS odom 回调 (真机位姿覆盖仿真) */
  setOdom(x, y, yaw) {
    this._odomX = x; this._odomY = y; this._odomYaw = yaw;
    this.odomOverride = true;
  }
  clearOdom() { this.odomOverride = false; }

  /** 每帧更新 */
  update() {
    const now = performance.now();
    const dt = Math.min((now - this._lastTime) / 1000, 0.05);
    this._lastTime = now;

    // ── 1. 处理输入 → 目标速度 ──
    let targetV = 0, targetW = 0;
    if (this._navActive) {
      // 自主导航: 由比例控制器给出目标速度
      const res = this._computeNavVel(dt);
      targetV = res.v;
      targetW = res.w;
      if (res.arrived) {
        this._navActive = false;
        this.v = 0;
        this.w = 0;
        const cb = this.onNavArrived;
        this.onNavArrived = null;
        this._navGoal = null;
        if (this.ros.publishEnabled && this.ros.connected) this.ros.sendCmdVel(0, 0);
        if (cb) cb();
      }
    } else {
      if (this.keys.forward)  targetV += this.maxLinear;
      if (this.keys.backward) targetV -= this.maxLinear;
      if (this.keys.left)     targetW += this.maxAngular;
      if (this.keys.right)    targetW -= this.maxAngular;
    }

    // ── 2. 平滑加速/减速 ──
    this.v += THREE.MathUtils.clamp(targetV - this.v, -this.accel * dt, this.accel * dt);
    this.w += THREE.MathUtils.clamp(targetW - this.w, -this.angAccel * dt, this.angAccel * dt);

    // 松手时衰减归零
    if (targetV === 0) this.v *= Math.pow(this.damping, dt * 60);
    if (targetW === 0) this.w *= Math.pow(this.damping, dt * 60);
    if (Math.abs(this.v) < 0.001) this.v = 0;
    if (Math.abs(this.w) < 0.001) this.w = 0;

    // ── 3. 位姿更新 (差速运动学) ──
    if (!this.odomOverride) {
      // 仿真自行计算
      const yaw = this.robot.root.rotation.z;
      const newX = this.robot.root.position.x + this.v * Math.cos(yaw) * dt;
      const newY = this.robot.root.position.y + this.v * Math.sin(yaw) * dt;
      const newYaw = yaw + this.w * dt;
      this.robot.root.position.set(newX, newY, 0);
      this.robot.root.rotation.z = newYaw;
    } else {
      // 真机 odom 覆盖
      this.robot.root.position.set(this._odomX, this._odomY, 0);
      this.robot.root.rotation.z = this._odomYaw;
    }

    // ── 4. 轮子自旋 (视觉反馈) ──
    // 4WD: 所有轮子同向自旋, 角速度 = v / r; 转向时内侧外侧仍有差异但简化处理
    const wheelRate = this.v / this.wheelRadius;
    for (const name of WHEEL_JOINT_NAMES) {
      const cur = this.robot.getJoint(name);
      // continuous 关节无限位, 直接累加
      this.robot.setJoint(name, cur + wheelRate * dt);
    }

    // ── 5. 有 ROS 且开启发送 → 同时下发真机 (节流 20Hz) ──
    if (this.ros.publishEnabled && this.ros.connected) {
      const now = performance.now();
      if (now - this._lastCmdSent > 50) {
        this._lastCmdSent = now;
        this.ros.sendCmdVel(this.v, this.w);
      }
    }

    return { v: this.v, w: this.w };
  }
}
