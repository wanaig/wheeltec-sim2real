/**
 * IK.js
 * 逆运动学求解器 — CCD (Cyclic Coordinate Descent) 算法
 * 针对 5-DOF 机械臂链: joint1 → joint2 → joint3 → joint4 → joint5
 * 末端 = link5 原点 (与 SRDF end_effector 一致)
 *
 * CCD 原理: 从最末端关节向基座迭代, 每次旋转一个关节使末端逼近目标,
 * 多次迭代收敛。每步只做平面投影旋转, 计算量极小, 适合浏览器实时求解。
 */
import * as THREE from 'three';
import { ARM_JOINT_NAMES, GRIPPER_TCP } from './RobotModel.js';

export class IKSolver {
  constructor(robotModel) {
    this.robot = robotModel;
    this.chain = ARM_JOINT_NAMES;              // ['joint1'...'joint5'] (用于结果输出)
    this.posChain = ARM_JOINT_NAMES.slice(0, 4); // 位置求解链 joint1-4
    this.eeJoint = 'joint5';                   // 末端关节 (link5 frame)
    this.maxIter = 200;             // 最大迭代次数
    this.tolerance = 0.025;         // 收敛阈值 25mm
    // 夹爪 TCP (抓取中心) 相对 link5 的固定偏移 — IK 目标为夹爪指尖, 非 link5 原点
    this._tcp = new THREE.Vector3(GRIPPER_TCP[0], GRIPPER_TCP[1], GRIPPER_TCP[2]);
    this._eeQ = new THREE.Quaternion();
  }

  /** 末端 (夹爪 TCP) 世界坐标 */
  _eeWorld(out) {
    const g = this.robot.jointGroups[this.eeJoint];
    g.getWorldPosition(out);
    g.getWorldQuaternion(this._eeQ);
    out.add(this._tcp.clone().applyQuaternion(this._eeQ));
    return out;
  }

  /**
   * 求解 IK: 给定世界坐标目标, 求关节角使末端到达 target
   * @param {THREE.Vector3} target 目标世界坐标 (米)
   * @returns {{solved:boolean, error:number, iterations:number, joints:Object}}
   */
  solve(target) {
    const robot = this.robot;
    const chain = this.chain;
    const tmpV = new THREE.Vector3();
    const pivot = new THREE.Vector3();
    const ee = new THREE.Vector3();
    const parentQ = new THREE.Quaternion();
    const worldAxis = new THREE.Vector3();
    const v1 = new THREE.Vector3();
    const v2 = new THREE.Vector3();
    const v1p = new THREE.Vector3();
    const v2p = new THREE.Vector3();
    const crossVec = new THREE.Vector3();

    let error = Infinity;
    let iter = 0;

    // joint5 为腕部滚转: TCP 几乎在其旋转轴上, 参与 CCD 会使投影≈0 而产生随机大角度
    // (夹爪被甩进目标点穿模). 故固定为中性 0, 位置仅由 joint1-4 求解.
    robot.setJoint('joint5', 0);

    for (iter = 0; iter < this.maxIter; iter++) {
      // 当前末端位置 (夹爪 TCP)
      this._eeWorld(ee);
      error = ee.distanceTo(target);
      if (error < this.tolerance) break;

      // 从末端关节向基座迭代 (CCD) — 仅 joint1-4
      for (let i = this.posChain.length - 1; i >= 0; i--) {
        const jname = this.posChain[i];
        const jg = robot.jointGroups[jname];
        const axis = robot.jointAxes[jname];

        // 关节枢轴点 (世界坐标)
        jg.getWorldPosition(pivot);

        // 世界旋转轴 = 父节点世界旋转 * 局部轴 (origin rpy=0)
        const parent = jg.parent;
        parent.getWorldQuaternion(parentQ);
        worldAxis.copy(axis).applyQuaternion(parentQ).normalize();

        // 当前末端位置 (更新, 因为前面的关节可能已变)
        this._eeWorld(ee);

        // v1 = 末端 - 枢轴, v2 = 目标 - 枢轴
        v1.copy(ee).sub(pivot);
        v2.copy(target).sub(pivot);

        // 投影到垂直于 worldAxis 的平面
        const d1 = v1.dot(worldAxis);
        const d2 = v2.dot(worldAxis);
        v1p.copy(v1).addScaledVector(worldAxis, -d1);
        v2p.copy(v2).addScaledVector(worldAxis, -d2);

        if (v1p.lengthSq() < 1e-12 || v2p.lengthSq() < 1e-12) continue;

        // 有符号夹角: atan2( (v1p×v2p)·axis, v1p·v2p )
        crossVec.crossVectors(v1p, v2p);
        const sin = crossVec.dot(worldAxis);
        const cos = v1p.dot(v2p);
        let delta = Math.atan2(sin, cos);

        // 应用旋转 (限制在关节限位内)
        const current = robot.jointValues[jname];
        const lim = robot.jointLimits[jname];
        let newVal = current + delta;
        if (lim) newVal = Math.max(lim[0], Math.min(lim[1], newVal));

        // 实际旋转量 (考虑 clamp)
        const actualDelta = newVal - current;
        // setJoint 返回 false 表示碰撞被拒绝, 跳过该关节
        const moved = robot.setJoint(jname, newVal);
        if (!moved || Math.abs(actualDelta) < 1e-8) continue;
      }
    }

    // 最终误差
    this._eeWorld(ee);
    error = ee.distanceTo(target);

    const joints = {};
    for (const n of chain) joints[n] = robot.jointValues[n];

    return {
      solved: error < this.tolerance,
      error: error,
      iterations: iter,
      joints: joints,
      endEffector: { x: ee.x, y: ee.y, z: ee.z },
    };
  }
}
