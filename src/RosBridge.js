/**
 * RosBridge.js
 * sim2real 桥接层：通过 rosbridge_server (WebSocket) 与真机 ROS 通信。
 *
 * 数据流:
 *   真机 → /joint_states → 仿真镜像 (实时同步)
 *   仿真 → voice_joint_states → 真机执行 (关节指令)
 *   仿真 → cmd_vel → 真机底盘 (移动指令)
 *
 * 依赖: roslibjs (npm roslib), 真机需启动 rosbridge_websocket
 *   roslaunch rosbridge_server rosbridge_websocket.launch
 */
import ROSLIB from 'roslib';
import { ALL_JOINT_NAMES } from './RobotModel.js';

// ROS1 与 ROS2 的消息类型字符串差异:
//   ROS1: "sensor_msgs/JointState"          时间戳 {secs, nsecs}
//   ROS2: "sensor_msgs/msg/JointState"      时间戳 {sec, nanosec}
// rosbridge_suite 在两个发行版上各自有对应版本, roslibjs 前端可复用,
// 只需按版本切换 messageType 与 stamp 字段命名即可。
const MSG_TYPES = {
  ros1: {
    JointState: 'sensor_msgs/JointState',
    Twist: 'geometry_msgs/Twist',
    Odometry: 'nav_msgs/Odometry',
  },
  ros2: {
    JointState: 'sensor_msgs/msg/JointState',
    Twist: 'geometry_msgs/msg/Twist',
    Odometry: 'nav_msgs/msg/Odometry',
  },
};

export class RosBridge {
  constructor() {
    this.ros = null;
    this.connected = false;
    this.syncEnabled = false;
    this.publishEnabled = false;
    this.version = 'ros2';            // 'ros1' | 'ros2' (默认 ROS2)
    this.onStatus = null;       // (connected: boolean) => void
    this.onJointState = null;   // (names, positions) => void

    this._jointSub = null;
    this._armCmdPub = null;
    this._fakeCtrlPub = null;
    this._cmdVelPub = null;
    this._odomSub = null;
    this.onOdom = null;         // (x, y, theta) => void
    this.getCurrentJoints = null; // () => {joint1..joint6} 当前关节状态, 用于补全发送帧
  }

  /** 切换 ROS 版本 (连接前调用; 已连接则需重连以重建话题) */
  setVersion(version) {
    if (version !== 'ros1' && version !== 'ros2') return;
    this.version = version;
  }

  /** 当前版本的消息类型 */
  _mt(kind) {
    return MSG_TYPES[this.version][kind];
  }

  /** 连接到 rosbridge */
  connect(url) {
    return new Promise((resolve, reject) => {
      this.ros = new ROSLIB.Ros({ url });

      this.ros.on('connection', () => {
        this.connected = true;
        this._setupTopics();
        this._notifyStatus();
        resolve();
      });

      this.ros.on('error', (e) => {
        this.connected = false;
        this._notifyStatus();
        reject(e);
      });

      this.ros.on('close', () => {
        this.connected = false;
        this.syncEnabled = false;
        this._notifyStatus();
      });
    });
  }

  /** 断开 */
  disconnect() {
    if (this._jointSub) { this._jointSub.unsubscribe(); this._jointSub = null; }
    if (this._odomSub) { this._odomSub.unsubscribe(); this._odomSub = null; }
    if (this.ros) { this.ros.close(); this.ros = null; }
    this.connected = false;
    this._notifyStatus();
  }

  _notifyStatus() {
    if (this.onStatus) this.onStatus(this.connected);
  }

  /** 连接成功后创建话题 */
  _setupTopics() {
    // 订阅 /joint_states (真机 → 仿真)
    this._jointSub = new ROSLIB.Topic({
      ros: this.ros,
      name: '/joint_states',
      messageType: this._mt('JointState'),
      throttle_rate: 50,   // 20Hz 限速, 防止浏览器过载
    });

    // 发布 voice_joint_states (仿真 → 真机机械臂, 即时指令)
    this._armCmdPub = new ROSLIB.Topic({
      ros: this.ros,
      name: 'voice_joint_states',
      messageType: this._mt('JointState'),
      queue_size: 10,
    });

    // 发布 move_group/fake_controller_joint_states (喂给 joint_state_publisher)
    // joint_state_publisher 的 source_list 读此话题, 合并后发布 /joint_states
    // 这样 /joint_states 上只有 joint_state_publisher 一个发布者, 携带我们的值而非零值
    // (ROS1 场景; ROS2 若用 joint_state_publisher/gz 等价物同样适用)
    this._fakeCtrlPub = new ROSLIB.Topic({
      ros: this.ros,
      name: 'move_group/fake_controller_joint_states',
      messageType: this._mt('JointState'),
      queue_size: 10,
    });

    // 发布 cmd_vel (仿真 → 真机底盘)
    this._cmdVelPub = new ROSLIB.Topic({
      ros: this.ros,
      name: 'cmd_vel',
      messageType: this._mt('Twist'),
      queue_size: 10,
    });

    // 订阅 /odom (底盘里程计, 用于同步底盘位姿)
    this._odomSub = new ROSLIB.Topic({
      ros: this.ros,
      name: '/odom',
      messageType: this._mt('Odometry'),
      throttle_rate: 100,
    });
  }

  /** 开启/关闭 真机→仿真 实时同步 */
  setSyncEnabled(enabled) {
    this.syncEnabled = enabled;
    if (enabled && this.connected && this._jointSub) {
      this._jointSub.subscribe((msg) => {
        if (this.onJointState) this.onJointState(msg.name, msg.position);
      });
      this._odomSub.subscribe((msg) => {
        if (this.onOdom) {
          const p = msg.pose.pose.position;
          const q = msg.pose.pose.orientation;
          const yaw = Math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z));
          this.onOdom(p.x, p.y, yaw);
        }
      });
    } else {
      if (this._jointSub) this._jointSub.unsubscribe();
      if (this._odomSub) this._odomSub.unsubscribe();
    }
  }

  /** 开启/关闭 仿真→真机 指令发送 */
  setPublishEnabled(enabled) {
    this.publishEnabled = enabled;
  }

  /** 生成毫秒精度时间戳, 避免 robot_state_publisher "rosbag looped" 警告
   *  ROS1: {secs, nsecs}  ROS2: {sec, nanosec} (builtin_interfaces/Time) */
  _stamp() {
    const now = Date.now();
    const secs = Math.floor(now / 1000);
    const nsecVal = (now % 1000) * 1e6;
    if (this.version === 'ros2') {
      return { sec: secs, nanosec: nsecVal };
    }
    return { secs: secs, nsecs: nsecVal };
  }

  /**
   * 发送机械臂关节目标角度 (仿真 → 真机)
   * 真机 voice_joint_states 回调按【顺序】读 6 个 position: [joint1..joint5, joint6(夹爪)],
   * 完全忽略 name 字段。故无论传入什么 jointMap, 此处都补全成完整 6 元数组按固定顺序发送,
   * 未指定的关节用当前状态保持 —— 否则只发 1 个关节会被真机当作 joint1, 或夹爪值错位到
   * joint1 导致机械臂猛甩。
   * @param {Object} jointMap  可只含部分关节, 例 {joint3:0.5} 或 {joint6:-0.45,...}
   */
  sendArmCommand(jointMap) {
    if (!this.publishEnabled || !this.connected || !this._armCmdPub) return;
    const order = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'];
    const cur = this.getCurrentJoints ? this.getCurrentJoints() : {};
    const positions = order.map(n =>
      (jointMap && jointMap[n] != null) ? jointMap[n]
      : (cur[n] != null ? cur[n] : 0)
    );
    const stamp = this._stamp();
    // 1. voice_joint_states (即时指令, 真机直接执行)
    this._armCmdPub.publish(new ROSLIB.Message({
      header: { stamp, frame_id: '' },
      name: order,
      position: positions,
      velocity: [],
      effort: [],
    }));
    // 2. move_group/fake_controller_joint_states (喂给 joint_state_publisher)
    //    joint_state_publisher 合并后发布 /joint_states, 消除零值竞争
    if (this._fakeCtrlPub) {
      this._fakeCtrlPub.publish(new ROSLIB.Message({
        header: { stamp, frame_id: '' },
        name: order,
        position: positions,
        velocity: [],
        effort: [],
      }));
    }
  }

  /**
   * 发送全部关节角度 (按 ALL_JOINT_NAMES 顺序输入, 但真机只接受 joint1..joint6)
   * @param {number[]} positions  ALL_JOINT_NAMES 顺序 (index 4..9 = joint1..joint6)
   */
  sendAllJoints(positions) {
    if (!this.publishEnabled || !this.connected || !this._armCmdPub) return;
    const order = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'];
    const six = (positions && positions.length >= 10) ? positions.slice(4, 10) : order.map(() => 0);
    const stamp = this._stamp();
    this._armCmdPub.publish(new ROSLIB.Message({
      header: { stamp, frame_id: '' },
      name: order,
      position: six,
      velocity: [],
      effort: [],
    }));
    if (this._fakeCtrlPub) {
      this._fakeCtrlPub.publish(new ROSLIB.Message({
        header: { stamp, frame_id: '' },
        name: order,
        position: six,
        velocity: [],
        effort: [],
      }));
    }
  }

  /**
   * 持续发布关节状态到 move_group/fake_controller_joint_states
   * joint_state_publisher 读此话题并合并发布 /joint_states, 使其携带我们的值而非零值
   * @param {number[]} allPositions  ALL_JOINT_NAMES 顺序 (15 个值)
   */
  publishJointStates(allPositions) {
    if (!this.publishEnabled || !this.connected || !this._fakeCtrlPub) return;
    const stamp = this._stamp();
    this._fakeCtrlPub.publish(new ROSLIB.Message({
      header: { stamp, frame_id: '' },
      name: ALL_JOINT_NAMES,
      position: allPositions,
      velocity: [],
      effort: [],
    }));
  }

  /**
   * 发送底盘速度指令 (仿真 → 真机)
   * @param {number} linear  线速度 m/s (X 前进)
   * @param {number} angular 角速度 rad/s (Z 转向)
   */
  sendCmdVel(linear, angular) {
    if (!this.publishEnabled || !this.connected || !this._cmdVelPub) return;
    this._cmdVelPub.publish(new ROSLIB.Message({
      linear:  { x: linear,  y: 0, z: 0 },
      angular: { x: 0, y: 0, z: angular },
    }));
  }

  /** 调用 MoveIt move_group 规划并执行 (可选, 需真机运行 move_group) */
  sendMoveGroupTarget(jointMap) {
    if (!this.connected) return Promise.reject('not connected');
    // 使用 FollowJointTrajectory action 简化版: 发送 JointTrajectory
    const trajPub = new ROSLIB.Topic({
      ros: this.ros,
      name: '/move_group/display_planned_path',
      // moveit_msgs 在 ROS2 同样带 /msg/ 前缀
      messageType: this.version === 'ros2'
        ? 'moveit_msgs/msg/DisplayTrajectory'
        : 'moveit_msgs/DisplayTrajectory',
      queue_size: 1,
    });
    const names = Object.keys(jointMap);
    // Duration 字段: ROS2 {sec, nanosec}, ROS1 {secs, nsecs}
    const duration = this.version === 'ros2'
      ? { sec: 2, nanosec: 0 }
      : { secs: 2, nsecs: 0 };
    const points = [{
      positions: names.map(n => jointMap[n]),
      time_from_start: duration,
    }];
    trajPub.publish(new ROSLIB.Message({
      trajectory_start: {},
      trajectories: [{
        joint_trajectory: {
          header: { frame_id: '' },
          joint_names: names,
          points,
        },
      }],
    }));
  }
}
