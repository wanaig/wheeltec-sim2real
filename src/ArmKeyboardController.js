/**
 * ArmKeyboardController.js
 * 机械臂关节 + 夹爪的键盘控制。
 *
 * 按键映射 (底盘用方向键, 不冲突):
 *   1 / Q  → joint1  底座偏航  (减 / 增)
 *   2 / W  → joint2  肩部俯仰  (增 / 减)
 *   3 / E  → joint3  肘部      (减 / 增)
 *   4 / R  → joint4  腕部俯仰  (减 / 增)
 *   5 / T  → joint5  腕部滚转  (减 / 增)
 *   6 / Y  → 夹爪     (开 / 闭)
 *   0      → 归零位 (preset arm_home)
 *
 * 按住持续旋转, 速度 0.6 rad/s; 松开停止。
 */
import { ARM_JOINT_NAMES } from './RobotModel.js';

const KEY_MAP = {
  // 数字键减, 字母键增 (键盘上垂直相邻)
  '1': { joint: 'joint1', dir: -1 }, q: { joint: 'joint1', dir:  1 },
  '2': { joint: 'joint2', dir:  1 }, w: { joint: 'joint2', dir: -1 },
  '3': { joint: 'joint3', dir: -1 }, e: { joint: 'joint3', dir:  1 },
  '4': { joint: 'joint4', dir: -1 }, r: { joint: 'joint4', dir:  1 },
  '5': { joint: 'joint5', dir: -1 }, t: { joint: 'joint5', dir:  1 },
  '6': { gripper: true, dir:  1 },   y: { gripper: true, dir: -1 },
};

export class ArmKeyboardController {
  constructor(robot, ros) {
    this.robot = robot;
    this.ros = ros;
    this.speed = 0.6;          // rad/s
    this.gripSpeed = 0.5;      // 夹爪 /s
    this.gripValue = 0;        // 当前夹爪主值 [-0.8, 0.8]
    this.keys = {};            // key → bool
    this._lastTime = performance.now();
    this._publishAccum = 0;
    this.onAfterUpdate = null; // () => void  每帧更新后回调 (同步滑块)
    this._bindKeys();
  }

  _bindKeys() {
    window.addEventListener('keydown', (e) => {
      if (e.target.tagName === 'INPUT' || e.target.tagName === 'TEXTAREA') return;
      const k = e.key.toLowerCase();
      if (KEY_MAP[k]) {
        this.keys[k] = true;
        this.robot.cancelTween();   // 键盘接管, 取消自动动画
        e.preventDefault();
      }
      // 归零
      if (k === '0') {
        const home = { joint1:0, joint2:0, joint3:0, joint4:0, joint5:0,
                       joint6:0, joint7:0, joint8:0, joint9:0, joint10:0, joint11:0 };
        this.gripValue = 0;
        this.robot.tweenTo(home, 1.0, this.onAfterUpdate);
        this.ros.sendArmCommand(home);
      }
    });
    window.addEventListener('keyup', (e) => {
      const k = e.key.toLowerCase();
      if (KEY_MAP[k]) this.keys[k] = false;
    });
  }

  update() {
    const now = performance.now();
    const dt = Math.min((now - this._lastTime) / 1000, 0.05);
    this._lastTime = now;

    let changed = false;
    const cmdMap = {};

    for (const [k, info] of Object.entries(KEY_MAP)) {
      if (!this.keys[k]) continue;

      if (info.gripper) {
        // 夹爪
        this.gripValue += info.dir * this.gripSpeed * dt;
        this.gripValue = Math.max(-0.8, Math.min(0.8, this.gripValue));
        this.robot.setJoints({
          joint6:  this.gripValue,
          joint7: -this.gripValue,
          joint8:  this.gripValue,
          joint9:  this.gripValue,
          joint10: this.gripValue,
          joint11: this.gripValue,
        });
        cmdMap.joint6  = this.gripValue;
        cmdMap.joint7  = -this.gripValue;
        cmdMap.joint8  = this.gripValue;
        cmdMap.joint9  = this.gripValue;
        cmdMap.joint10 = this.gripValue;
        cmdMap.joint11 = this.gripValue;
        changed = true;
      } else {
        // 机械臂关节
        const name = info.joint;
        const cur = this.robot.getJoint(name);
        const info2 = this.robot.getJointInfo(name);
        const lim = info2.limit;
        let next = cur + info.dir * this.speed * dt;
        if (lim) next = Math.max(lim[0], Math.min(lim[1], next));
        this.robot.setJoint(name, next);
        cmdMap[name] = next;
        changed = true;
      }
    }

    if (changed) {
      if (this.onAfterUpdate) this.onAfterUpdate();
      // 节流发布到真机 (10Hz)
      this._publishAccum += dt;
      if (this._publishAccum >= 0.1) {
        this._publishAccum = 0;
        this.ros.sendArmCommand(cmdMap);
      }
    }
  }
}
