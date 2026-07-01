/**
 * UIController.js
 * 绑定 DOM 控件 → 机器人模型 / IK / ROS, 实现 sim2real 交互层。
 */
import * as THREE from 'three';
import {
  ARM_JOINT_NAMES, GRIPPER_JOINT_NAMES,
  PRESETS, GRIPPER_POSES, ALL_JOINT_NAMES,
} from './RobotModel.js';

export class UIController {
  constructor({ robot, ik, ros, scene, chassis, armKey, cameras }) {
    this.robot = robot;
    this.ik = ik;
    this.ros = ros;
    this.chassis = chassis;
    this.armKey = armKey;
    this.scene = scene;
    this.cameras = cameras || [];
    this.ikMarker = null;
    this._targetBox = null;     // 随机生成的目标色块 (测试 IK 用)
    this.onJointChange = null;   // (jointMap) => void  用于发布到真机
  }

  init() {
    this._buildArmSliders();
    this._buildExtraSliders();
    this._wireRos();
    this._wirePresets();
    this._wireGripper();
    this._wireIK();
    this._wireRandomTarget();
    this._buildIKMarker();
    this._buildKeyboardHelp();
    this._buildCollisionToggle();
    this._wireCameras();
    // 机械臂键盘控制后同步滑块显示
    if (this.armKey) {
      this.armKey.onAfterUpdate = () => this._syncAllSliders();
    }
    // 碰撞反馈: 临时高亮警告
    this.robot.onCollision = (pair) => {
      const el = document.getElementById('collision-warn');
      if (el) {
        const msg = pair[0] === 'ground'
          ? `⚠ 地面碰撞: ${pair[1]} 穿入地面`
          : `⚠ 自碰撞: ${pair[0]} ↔ ${pair[1]}`;
        el.textContent = msg;
        el.classList.add('active');
        clearTimeout(this._colTimer);
        this._colTimer = setTimeout(() => el.classList.remove('active'), 800);
      }
    };
  }

  _buildCollisionToggle() {
    // 碰撞警告条 (悬浮于 3D 视口顶部)
    const warn = document.createElement('div');
    warn.id = 'collision-warn';
    warn.className = 'collision-warn';
    warn.textContent = '';
    document.getElementById('app').appendChild(warn);
  }

  // ─────────── 实时相机画面 ───────────
  _wireCameras() {
    // 折叠按钮
    const collapseBtn = document.getElementById('camera-collapse');
    const panel = document.getElementById('camera-panel');
    if (collapseBtn) {
      collapseBtn.addEventListener('click', () => {
        panel.classList.toggle('collapsed');
        collapseBtn.textContent = panel.classList.contains('collapsed') ? '展开' : '折叠';
      });
    }
    // 两个相机槽
    this.cameras.forEach((cam, idx) => {
      const topicInput = document.querySelector(`.cam-topic[data-cam="${idx}"]`);
      const kindSel = document.querySelector(`.cam-kind[data-cam="${idx}"]`);
      const enCb = document.querySelector(`.cam-en input[data-cam="${idx}"]`);
      if (topicInput && kindSel) {
        const apply = () => cam.configure(topicInput.value.trim(), kindSel.value);
        topicInput.addEventListener('change', apply);
        kindSel.addEventListener('change', apply);
        // 初始配置 (默认值已在 HTML 中)
        cam.configure(topicInput.value.trim(), kindSel.value);
      }
      if (enCb) {
        enCb.addEventListener('change', () => {
          if (enCb.checked) cam.enable();
          else cam.disable();
        });
        if (enCb.checked) cam.enable();
      }
    });
  }

  // ─────────── 键盘操作帮助面板 ───────────
  _buildKeyboardHelp() {
    const help = document.createElement('div');
    help.className = 'panel-section';
    help.innerHTML = `
      <h3>键盘操作</h3>
      <div class="kbd-grid">
        <div class="kbd-sep">底盘 (方向键)</div>
        <div class="kbd-row"><span class="kbd-group"><kbd>↑</kbd><kbd>↓</kbd></span><span>前进 / 后退</span></div>
        <div class="kbd-row"><span class="kbd-group"><kbd>←</kbd><kbd>→</kbd></span><span>左转 / 右转</span></div>
        <div class="kbd-sep">机械臂关节 (数字减 / 字母增)</div>
        <div class="kbd-row"><span class="kbd-group"><kbd>1</kbd><kbd>Q</kbd></span><span>joint1 底座偏航</span></div>
        <div class="kbd-row"><span class="kbd-group"><kbd>2</kbd><kbd>W</kbd></span><span>joint2 肩部俯仰</span></div>
        <div class="kbd-row"><span class="kbd-group"><kbd>3</kbd><kbd>E</kbd></span><span>joint3 肘部</span></div>
        <div class="kbd-row"><span class="kbd-group"><kbd>4</kbd><kbd>R</kbd></span><span>joint4 腕部俯仰</span></div>
        <div class="kbd-row"><span class="kbd-group"><kbd>5</kbd><kbd>T</kbd></span><span>joint5 腕部滚转</span></div>
        <div class="kbd-sep">夹爪</div>
        <div class="kbd-row"><span class="kbd-group"><kbd>6</kbd><kbd>Y</kbd></span><span>夹爪闭合 / 张开</span></div>
        <div class="kbd-row"><span class="kbd-group"><kbd>0</kbd></span><span>机械臂归零位</span></div>
      </div>
    `;
    document.getElementById('panel-left').appendChild(help);
  }

  // ─────────── 机械臂关节滑块 ───────────
  _buildArmSliders() {
    const container = document.getElementById('arm-sliders');
    for (const name of ARM_JOINT_NAMES) {
      const info = this.robot.getJointInfo(name);
      const lower = info.limit ? info.limit[0] : -3.14;
      const upper = info.limit ? info.limit[1] : 3.14;
      const step = 0.01;

      const row = document.createElement('div');
      row.className = 'slider-row';
      row.innerHTML = `
        <label class="slider-label">${name}</label>
        <input type="range" class="slider" min="${lower}" max="${upper}" step="${step}" value="0" data-joint="${name}"/>
        <span class="slider-val" data-val="${name}">0.00</span>
      `;
      container.appendChild(row);

      const slider = row.querySelector('.slider');
      const valSpan = row.querySelector('.slider-val');
      slider.addEventListener('input', () => {
        this.robot.cancelTween();   // 手动操作中断自动动画
        const v = parseFloat(slider.value);
        this.robot.setJoint(name, v);
        valSpan.textContent = v.toFixed(2);
        this._updateEE();
        if (this.onJointChange) this.onJointChange({ [name]: v });
      });
    }

    // 夹爪主控 (joint6 联动)
    const gripRow = document.createElement('div');
    gripRow.className = 'slider-row';
    gripRow.innerHTML = `
      <label class="slider-label">joint6</label>
      <input type="range" class="slider" min="-0.8" max="0.8" step="0.01" value="0" data-joint="gripper"/>
      <span class="slider-val" data-val="gripper">0.00</span>
    `;
    container.appendChild(gripRow);
    const gripSlider = gripRow.querySelector('.slider');
    const gripVal = gripRow.querySelector('.slider-val');
    gripSlider.addEventListener('input', () => {
      this.robot.cancelTween();   // 手动操作中断自动动画
      const v = parseFloat(gripSlider.value);
      // joint6 为主, 其余夹爪关节按比例联动
      const ratio = v / 0.8;
      this.robot.setJoints({
        joint6:  v,
        joint7: -v,
        joint8:  v,
        joint9:  v,
        joint10: v,
        joint11: v,
      });
      gripVal.textContent = v.toFixed(2);
      if (this.onJointChange) {
        this.onJointChange({ joint6:v, joint7:-v, joint8:v, joint9:v, joint10:v, joint11:v });
      }
    });
  }

  // ─────────── 底盘控制 ───────────
  _buildExtraSliders() {
    const container = document.getElementById('extra-sliders');

    // 底盘移动 (仿真驱动 + 真机 cmd_vel)
    const cmdRow = document.createElement('div');
    cmdRow.className = 'slider-row';
    cmdRow.innerHTML = `
      <label class="slider-label">chassis</label>
      <button id="btn-fwd" class="mini-btn hold-btn">↑</button>
      <button id="btn-bwd" class="mini-btn hold-btn">↓</button>
      <button id="btn-left" class="mini-btn hold-btn">↺</button>
      <button id="btn-right" class="mini-btn hold-btn">↻</button>
      <button id="btn-stop" class="mini-btn">■</button>
    `;
    container.appendChild(cmdRow);

    // 按住驱动, 松开停止 (同时驱动仿真 + 真机)
    const hold = (btn, fwd, bwd, lft, rgt) => {
      const press = (e) => { e.preventDefault(); this.chassis.drive(fwd, bwd, lft, rgt); };
      const release = () => { this.chassis.drive(false, false, false, false); };
      btn.addEventListener('mousedown', press);
      btn.addEventListener('mouseup', release);
      btn.addEventListener('mouseleave', release);
      btn.addEventListener('touchstart', press, { passive: false });
      btn.addEventListener('touchend', release);
    };
    hold(document.getElementById('btn-fwd'),   true,  false, false, false);
    hold(document.getElementById('btn-bwd'),   false, true,  false, false);
    hold(document.getElementById('btn-left'),  false, false, true,  false);
    hold(document.getElementById('btn-right'), false, false, false, true);
    document.getElementById('btn-stop').onclick = () => this.chassis.drive(false, false, false, false);

    // 速度显示
    const spdRow = document.createElement('div');
    spdRow.className = 'slider-row';
    spdRow.innerHTML = `
      <label class="slider-label">speed</label>
      <span class="slider-val" id="chassis-speed" style="min-width:auto;flex:1">v=0.00 m/s  w=0.00 rad/s</span>
    `;
    container.appendChild(spdRow);

    // 键盘提示
    const hint = document.createElement('div');
    hint.className = 'info-box';
    hint.style.marginTop = '6px';
    hint.innerHTML = '键盘方向键 ↑↓←→ 驱动底盘';
    container.appendChild(hint);
  }

  // ─────────── ROS 连接 ───────────
  _wireRos() {
    const btnC = document.getElementById('btn-connect');
    const btnD = document.getElementById('btn-disconnect');
    const urlInput = document.getElementById('ros-url');
    const versionSel = document.getElementById('ros-version');
    const statusEl = document.getElementById('ros-status');
    const cbSync = document.getElementById('cb-sync');
    const cbPub = document.getElementById('cb-publish');

    // 初始化版本到 RosBridge, 并在切换时同步 (连接中切换需重连)
    this.ros.setVersion(versionSel.value);
    versionSel.addEventListener('change', () => {
      this.ros.setVersion(versionSel.value);
      // 版本切换后图像话题的消息类型字符串也变了, 重建已启用的相机订阅
      (this.cameras || []).forEach(c => { if (c.enabled) c.rebuild(); });
      // 交互型智能体面板 (订阅 /agent/*) 同样需重建消息类型
      if (this.ros._agentPanel) this.ros._agentPanel.rebuild();
      // 已连接则断开, 提示用户重连以重建话题的消息类型
      if (this.ros.connected) {
        this.ros.disconnect();
        statusEl.textContent = '已切换 ROS 版本, 请重新连接';
        statusEl.className = 'status disconnected';
      }
    });

    this.ros.onStatus = (connected) => {
      statusEl.textContent = connected ? 'ROS 已连接' : 'ROS 未连接';
      statusEl.className = 'status ' + (connected ? 'connected' : 'disconnected');
      btnC.disabled = connected;
      btnD.disabled = !connected;
      cbSync.disabled = !connected;
      cbPub.disabled = !connected;
      if (!connected) { cbSync.checked = false; cbPub.checked = false; }
      // 连接状态变化 → 已启用的相机重建订阅
      (this.cameras || []).forEach(c => { if (c.enabled) c.onRosStateChanged(); });
      // 交互型智能体面板随连接状态重建订阅
      if (this.ros._agentPanel) this.ros._agentPanel.rebuild();
    };

    btnC.addEventListener('click', async () => {
      const url = urlInput.value.trim();
      if (!url) return;
      statusEl.textContent = '连接中...';
      statusEl.className = 'status connecting';
      try {
        await this.ros.connect(url);
      } catch (e) {
        statusEl.textContent = '连接失败';
        statusEl.className = 'status disconnected';
        console.error('ROS connect error:', e);
      }
    });

    btnD.addEventListener('click', () => this.ros.disconnect());

    cbSync.addEventListener('change', () => {
      this.ros.setSyncEnabled(cbSync.checked);
      // 同步模式下禁用机械臂手动滑块 (真机驱动), 底盘仍可键盘控制
      document.querySelectorAll('#arm-sliders .slider:not([data-joint="gripper"])').forEach(s => {
        s.disabled = cbSync.checked;
      });
      if (!cbSync.checked) this.chassis.clearOdom();
    });

    cbPub.addEventListener('change', () => {
      this.ros.setPublishEnabled(cbPub.checked);
    });

    // 真机 → 仿真: 实时更新模型 + 滑块
    this.ros.onJointState = (names, positions) => {
      // 本地关节动画进行中时, 不被真机状态覆盖, 避免抖动
      if (this.robot.isTweening) return;
      this.robot.applyJointState(names, positions);
      // 更新滑块显示
      for (let i = 0; i < names.length; i++) {
        const n = names[i];
        const sl = document.querySelector(`.slider[data-joint="${n}"]`);
        const vl = document.querySelector(`.slider-val[data-val="${n}"]`);
        if (sl) sl.value = positions[i];
        if (vl) vl.textContent = positions[i].toFixed(2);
      }
      this._updateEE();
    };

    // 底盘里程计 → ChassisController 接管 (真机位姿覆盖仿真)
    this.ros.onOdom = (x, y, yaw) => {
      this.chassis.setOdom(x, y, yaw);
    };

    // 仿真 → 真机: 滑块/预设变更时发布
    this.onJointChange = (jointMap) => {
      this.ros.sendArmCommand(jointMap);
    };
  }

  // ─────────── 预设位姿 ───────────
  _wirePresets() {
    document.querySelectorAll('.preset-btn').forEach(btn => {
      btn.addEventListener('click', () => {
        const name = btn.dataset.preset;
        const pose = PRESETS[name];
        // 平滑过渡到预设位姿 (动画), 同时发布到真机
        this._animateToPose(pose, 1.2);
      });
    });
  }

  // ─────────── 夹爪开合 ───────────
  _wireGripper() {
    document.querySelectorAll('.grip-btn').forEach(btn => {
      btn.addEventListener('click', () => {
        const g = btn.dataset.grip;  // 'open' | 'close'
        // 平滑过渡 (夹爪动作较快)
        this._animateToPose(GRIPPER_POSES[g], 0.5);
      });
    });
  }

  // ─────────── 逆运动学 ───────────
  _wireIK() {
    document.getElementById('btn-ik-solve').addEventListener('click', () => {
      const x = parseFloat(document.getElementById('ik-x').value);
      const y = parseFloat(document.getElementById('ik-y').value);
      const z = parseFloat(document.getElementById('ik-z').value);
      const target = new THREE.Vector3(x, y, z);
      // 显示目标点
      this._showIKMarker(target);
      this._approachAndSolve(target);
    });

    document.getElementById('btn-ik-marker').addEventListener('click', () => {
      const x = parseFloat(document.getElementById('ik-x').value);
      const y = parseFloat(document.getElementById('ik-y').value);
      const z = parseFloat(document.getElementById('ik-z').value);
      this._showIKMarker(new THREE.Vector3(x, y, z));
    });
  }

  // ─────────── 随机目标色块 (测试 IK) ───────────
  _wireRandomTarget() {
    document.getElementById('btn-random-target').addEventListener('click', () => {
      this._generateRandomTarget();
    });
  }

  /**
   * 在机械臂前方工作空间内随机生成一个色块作为 IK 目标:
   * 创建彩色方块放入场景, 并把坐标填入 IK 输入框、显示目标点。
   */
  _generateRandomTarget() {
    // 以底盘当前位姿为参考, 在前方/侧方随机生成, 色块贴地放置
    const car = this.robot.root.position;
    const yaw = this.robot.root.rotation.z;
    const fwd  = 0.15 + Math.random() * 0.25;   // 前方 0.15~0.40 m
    const lat  = (Math.random() - 0.5) * 0.60;  // 侧方 ±0.30 m
    const size = 0.03;
    const z    = size / 2;                       // 方块底面在地面上 (z=0), 中心 z=size/2
    const cosY = Math.cos(yaw), sinY = Math.sin(yaw);
    const x = car.x + fwd * cosY - lat * sinY;
    const y = car.y + fwd * sinY + lat * cosY;

    // 填入 IK 输入框
    document.getElementById('ik-x').value = x.toFixed(3);
    document.getElementById('ik-y').value = y.toFixed(3);
    document.getElementById('ik-z').value = z.toFixed(3);

    const target = new THREE.Vector3(x, y, z);
    this._showIKMarker(target);

    // 生成彩色方块 (替换上一个)
    if (this._targetBox) {
      this.scene.remove(this._targetBox);
      this._targetBox.geometry.dispose();
      this._targetBox.material.dispose();
    }
    const geo = new THREE.BoxGeometry(size, size, size);
    const hue = Math.random();
    const color = new THREE.Color().setHSL(hue, 0.75, 0.55);
    const mat = new THREE.MeshStandardMaterial({ color: color, roughness: 0.45, metalness: 0.1 });
    this._targetBox = new THREE.Mesh(geo, mat);
    this._targetBox.position.set(x, y, z);
    this._targetBox.castShadow = false;
    this.scene.add(this._targetBox);

    const info = document.getElementById('ik-info');
    if (info) info.innerHTML = `<span class="ok">● 已生成目标色块</span> · 位置 x=${x.toFixed(2)} y=${y.toFixed(2)} z=${z.toFixed(2)} m · 点击"求解 IK"抓取`;
  }

  /**
   * 先尝试原地 IK; 若不可达 (过近/过远/朝向不正) → 自动移动底盘至合适站位, 到达后重试。
   * 不再用距离阈值猜测可达性, 而以 IK 是否收敛为准, 避免过近时误判。
   */
  _approachAndSolve(target) {
    const DESIRED_DIST = 0.28;  // 期望 3D 站位距离 (舒适抓取, 位于臂展可达范围)
    const info = document.getElementById('ik-info');
    const norm = (a) => Math.atan2(Math.sin(a), Math.cos(a));

    // 1. 先尝试原地 IK (试探后自动复位, 不改变当前姿态)
    const result = this._attemptIK(target);
    if (result.solved) {
      info.innerHTML = `<span class="ok">✓ 求解成功</span> · 误差 ${result.error.toFixed(4)}m · ${result.iterations} 次迭代`;
      this._graspFromResult(result);
      return;
    }

    // 2. 原地不可达 → 计算站位, 移动底盘 (过近时目标在车后方 → 倒车后退)
    const armBase = new THREE.Vector3();
    this.robot.jointGroups['joint1'].getWorldPosition(armBase);
    const dx = target.x - armBase.x;
    const dy = target.y - armBase.y;
    const dz = target.z - armBase.z;
    const yaw = this.robot.root.rotation.z;

    // 根据目标 z 自适应 standoff, 使 3D 距离 ≈ DESIRED_DIST
    let standoff = Math.sqrt(Math.max(0, DESIRED_DIST * DESIRED_DIST - dz * dz));
    standoff = Math.max(0.15, standoff);

    // 方向: 从目标指向当前臂座 (从当前所在侧靠近; 过近时即后退方向)
    let dirX = armBase.x - target.x;
    let dirY = armBase.y - target.y;
    let dl = Math.hypot(dirX, dirY);
    if (dl < 1e-6) { dirX = Math.cos(yaw); dirY = Math.sin(yaw); dl = 1; }
    dirX /= dl; dirY /= dl;

    const armGoalX = target.x + dirX * standoff;
    const armGoalY = target.y + dirY * standoff;
    const goalYaw = Math.atan2(target.y - armGoalY, target.x - armGoalX);

    // 臂座在 base_link 中的 XY 偏移 (URDF joint1 origin)
    const offX = 0.054476, offY = 0.00070272;
    const cosG = Math.cos(goalYaw), sinG = Math.sin(goalYaw);
    const chassisGoalX = armGoalX - (cosG * offX - sinG * offY);
    const chassisGoalY = armGoalY - (sinG * offX + cosG * offY);

    info.innerHTML = `<span class="warn">↻ 原地不可达 (误差 ${result.error.toFixed(2)}m), 移动底盘至抓取站位…</span>`;
    this.chassis.navigateTo(chassisGoalX, chassisGoalY, goalYaw);
    this.chassis.onNavArrived = () => {
      // 到达站位后重试 (此时应收敛)
      const r = this._attemptIK(target);
      const el = document.getElementById('ik-info');
      el.innerHTML = r.solved
        ? `<span class="ok">✓ 求解成功</span> · 误差 ${r.error.toFixed(4)}m · ${r.iterations} 次迭代`
        : `<span class="warn">△ 未完全收敛</span> · 误差 ${r.error.toFixed(4)}m · ${r.iterations} 次迭代`;
      this._graspFromResult(r);
    };
  }

  /** 试探 IK: 求解后立即复位到起点, 不改变当前姿态; 返回结果 */
  _attemptIK(target) {
    const startNames = ARM_JOINT_NAMES.slice();
    const startVals = startNames.map(n => this.robot.getJoint(n));
    const result = this.ik.solve(target);
    this.robot.applyJointState(startNames, startVals);  // 复位, 撤销求解中的关节改动
    return result;
  }

  /** 由 IK 结果执行抓取: 平滑过渡到解 → 闭合夹爪 (同步发布真机) */
  _graspFromResult(result) {
    this.ros.sendArmCommand(result.joints);
    this.robot.tweenTo(result.joints, 1.2, () => {
      this._syncAllSlidersFromRobot();
      // 臂到位, 闭合夹爪
      this.ros.sendArmCommand(GRIPPER_POSES.close);
      this.robot.tweenTo(GRIPPER_POSES.close, 0.5, () => {
        this._syncAllSlidersFromRobot();
        const el = document.getElementById('ik-info');
        if (el) el.innerHTML += ' · <span class="ok">夹爪已闭合</span>';
      });
    });
  }

  _buildIKMarker() {
    const geo = new THREE.SphereGeometry(0.012, 24, 16);
    const mat = new THREE.MeshBasicMaterial({ color: 0xff3333 });
    this.ikMarker = new THREE.Mesh(geo, mat);
    this.ikMarker.visible = false;
    this.scene.add(this.ikMarker);

    // 目标点连线
    const lineGeo = new THREE.BufferGeometry();
    lineGeo.setAttribute('position', new THREE.Float32BufferAttribute([0,0,0,0,0,0], 3));
    const lineMat = new THREE.LineDashedMaterial({ color: 0xff3333, dashSize: 0.02, gapSize: 0.01 });
    this.ikLine = new THREE.Line(lineGeo, lineMat);
    this.ikLine.visible = false;
    this.scene.add(this.ikLine);
  }

  _showIKMarker(target) {
    this.ikMarker.position.copy(target);
    this.ikMarker.visible = true;
    // 虚线从夹爪指尖 (TCP) 到目标
    const ee = this.robot.getGripperTCP();
    const arr = this.ikLine.geometry.attributes.position.array;
    arr[0] = ee.x; arr[1] = ee.y; arr[2] = ee.z;
    arr[3] = target.x; arr[4] = target.y; arr[5] = target.z;
    this.ikLine.geometry.attributes.position.needsUpdate = true;
    this.ikLine.computeLineDistances();
    this.ikLine.visible = true;
  }

  // ─────────── 工具方法 ───────────
  _syncSliders() {
    for (const name of ARM_JOINT_NAMES) {
      const v = this.robot.getJoint(name);
      const sl = document.querySelector(`.slider[data-joint="${name}"]`);
      const vl = document.querySelector(`.slider-val[data-val="${name}"]`);
      if (sl) sl.value = v;
      if (vl) vl.textContent = v.toFixed(2);
    }
  }

  /** 从模型真实角度同步全部滑块 (含夹爪 joint6), 用于动画期间/结束后 */
  _syncAllSlidersFromRobot() {
    this._syncSliders();
    const v = this.robot.getJoint('joint6');
    const sl = document.querySelector('.slider[data-joint="gripper"]');
    const vl = document.querySelector('.slider-val[data-val="gripper"]');
    if (sl) sl.value = v;
    if (vl) vl.textContent = v.toFixed(2);
  }

  /** 平滑过渡到指定位姿 (动画), 并发布到真机 */
  _animateToPose(pose, duration = 1.0) {
    this.robot.tweenTo(pose, duration, () => this._syncAllSlidersFromRobot());
    this.ros.sendArmCommand(pose);
  }

  /** 同步全部滑块 (含夹爪), 供键盘控制后调用 */
  _syncAllSliders() {
    this._syncSliders();
    // 夹爪滑块
    if (this.armKey) {
      const v = this.armKey.gripValue;
      const sl = document.querySelector('.slider[data-joint="gripper"]');
      const vl = document.querySelector('.slider-val[data-val="gripper"]');
      if (sl) sl.value = v;
      if (vl) vl.textContent = v.toFixed(2);
    }
  }

  _updateEE() {
    const p = this.robot.getEndEffectorPosition();
    const q = this.robot.getEndEffectorQuaternion();
    const euler = new THREE.Euler().setFromQuaternion(q, 'ZYX');
    const el = document.getElementById('ee-info');
    el.innerHTML = `
      <b>link5 世界位姿</b><br>
      位置: x=${p.x.toFixed(3)} y=${p.y.toFixed(3)} z=${p.z.toFixed(3)} m<br>
      姿态: R=${THREE.MathUtils.radToDeg(euler.x).toFixed(1)}°
            P=${THREE.MathUtils.radToDeg(euler.y).toFixed(1)}°
            Y=${THREE.MathUtils.radToDeg(euler.z).toFixed(1)}°
    `;
  }

  /** 每帧调用: 更新末端显示 + IK 标记线 + 底盘速度 */
  tick() {
    this._updateEE();
    // 底盘速度显示
    const spdEl = document.getElementById('chassis-speed');
    if (spdEl && this.chassis) {
      spdEl.textContent = `v=${this.chassis.v.toFixed(2)} m/s  w=${this.chassis.w.toFixed(2)} rad/s`;
    }
    // 自主导航进度
    if (this.chassis && this.chassis._navActive) {
      const info = document.getElementById('ik-info');
      if (info) info.innerHTML = `<span class="warn">↻ 自动靠近目标中… 剩余 ${this.chassis._navDist.toFixed(2)} m</span>`;
    }
    // 关节动画期间, 滑块跟随插值实时更新
    if (this.robot.isTweening) {
      this._syncAllSlidersFromRobot();
    }
    if (this.ikLine.visible) {
      const ee = this.robot.getGripperTCP();
      const arr = this.ikLine.geometry.attributes.position.array;
      arr[0] = ee.x; arr[1] = ee.y; arr[2] = ee.z;
      this.ikLine.geometry.attributes.position.needsUpdate = true;
      this.ikLine.computeLineDistances();
    }
  }
}
