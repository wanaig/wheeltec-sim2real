# WHEELTEC mini_4wd_six_arm · Sim2Real 数字孪生

基于 **Vite + Three.js** 构建的 WHEELTEC R550A 6 自由度机械臂(四驱底盘)三维数字孪生系统，直接从 URDF 解析运动学树并加载真实 STL 网格，通过 roslibjs 与真机 ROS 双向通信，实现 sim2real。

**同时支持 ROS1 (Melodic/Noetic) 与 ROS2 (Foxy/Humble/Jazzy)**：UI 左上角「ROS 版本」下拉框切换，前端自动适配消息类型字符串 (`sensor_msgs/JointState` ↔ `sensor_msgs/msg/JointState`) 与时间戳字段 (`secs/nsecs` ↔ `sec/nanosec`)。配套 ROS2 串口桥接包见 [`ros2/`](./ros2)。

## 快速开始

```bash
cd wheeltec-sim2real
npm install
npm run dev          # 浏览器打开 http://localhost:5173
```

生产构建：
```bash
npm run build && npm run preview
```

## 机器人结构 (来自 URDF)

```
base_link (底盘)
├── 4× 车轮 (continuous, 绕 Y 轴)
└── joint1 (Z 轴旋转, 底座偏航)          ┐
    └── joint2 (Y 轴, 肩部俯仰)          │
        └── joint3 (Y 轴, 肘部)          │  5-DOF 机械臂
            └── joint4 (Y 轴, 腕部俯仰)  │  (MoveIt arm group)
                └── joint5 (Z 轴, 腕部滚转) ┘
                    ├── joint6/7  (夹爪指 1)
                    ├── joint8/9  (夹爪指 2)
                    └── joint10/11(夹爪指 3)
```

- 16 个真实 STL 网格 (SolidWorks 导出)
- 关节 origin / axis / limit 100% 取自 `mini_4wd_six_arm.urdf`
- 预设位姿取自 SRDF group_state (归零/举起/夹取/侧举/侧放)

## Sim2Real 架构

```
┌─────────────────┐     WebSocket      ┌──────────────────┐
│  浏览器数字孪生  │ ◄─────────────────► │  真机 ROS        │
│  (Three.js)     │   rosbridge_server  │  (melodic)       │
└────────┬────────┘                     └────────┬─────────┘
         │                                       │
         │  真机 → 仿真 (实时镜像)               │
         │  /joint_states  (sensor_msgs/JointState)
         │  /odom          (nav_msgs/Odometry)
         │                                       │
         │  仿真 → 真机 (指令下发)               │
         │  voice_joint_states (sensor_msgs/JointState)
         │  cmd_vel            (geometry_msgs/Twist)
```

### 真机端准备

#### ROS2 (推荐, Foxy/Humble/Jazzy)

1. 安装 rosbridge_server 与 serial 依赖：
```bash
sudo apt install ros-<distro>-rosbridge-server python3-serial
# <distro> = foxy / humble / jazzy
```
2. 构建配套串口桥接包 [`ros2/wheeltec_sim2real_bridge`](./ros2) (由 ROS1 `wheeltec_arm_six` 移植, 协议逐字节一致)：
```bash
cd ros2
colcon build --symlink-install
source install/setup.bash
```
3. 启动 (串口桥接 + rosbridge 一条命令)：
```bash
# 真机
ros2 launch wheeltec_sim2real_bridge sim2real_bridge.launch.py
# 无硬件闭环测试 (mock 节点回显指令, 跑通全流程)
ros2 launch wheeltec_sim2real_bridge sim2real_bridge.launch.py mock:=true
```
4. 赋予串口权限 (真机)：
```bash
sudo chmod 666 /dev/wheeltec_controller
```

#### ROS1 (Melodic/Noetic, 兼容)

在小车 ROS 端启动 rosbridge：
```bash
# 安装 (首次)
sudo apt install ros-melodic-rosbridge-server
# 启动
roslaunch rosbridge_server rosbridge_websocket.launch
```
确保 `base_serial.launch` 已启动 (机械臂串口节点 + MoveIt)：
```bash
roslaunch wheeltec_arm_pick base_serial.launch
```

### 连接步骤

1. 浏览器左上角「ROS 版本」选择 ROS2 (或 ROS1)
2. 输入小车 IP 的 WebSocket 地址 (默认 `ws://192.168.0.20:9090`)
3. 点击「连接」
4. 勾选「实时同步真机状态」→ 仿真模型镜像真机 `/joint_states` + `/odom`
5. 勾选「将仿真指令发送至真机」→ 滑块/预设/IK 结果经 `voice_joint_states` + `cmd_vel` 下发真机

### 实时相机画面

底部「实时相机画面」面板提供 **双画面**, 每个 slot 可独立配置话题名与消息类型, 连接 ROS 后勾选「启用」即订阅并解码到 canvas。

| 消息类型 | ROS2 字符串 | 说明 |
|---------|------------|------|
| CompressedImage | `sensor_msgs/msg/CompressedImage` | 推荐, jpeg/png 带宽低 (~15fps) |
| Image (raw) | `sensor_msgs/msg/Image` | 原始图, 支持 rgb8/bgr8/mono8 (~10fps) |

**默认话题 (mini_4wd_six_arm 单 USB 摄像头):**
- 相机1: `/image_raw/compressed` (CompressedImage) — `usb_cam` 经 image_transport 重映射的压缩彩色流
- 相机2: `/usb_cam/image_raw/compressed` (CompressedImage) — 备用, 可改填第二个相机话题

**常见话题参考:**

| 相机 | 话题 (压缩 / 原始) |
|------|------|
| USB 摄像头 (six_arm) | `/image_raw/compressed` · `/usb_cam/image_raw` |
| Astra 深度相机 RGB (four_arm) | `/camera/rgb/image_raw/compressed` · `/camera/rgb/image_raw` |
| Astra 深度图 | `/camera/depth/image_raw/compressed` · `/camera/depth/image_raw` (mono8/16) |

> 若你的真机有两个相机, 直接把两个 slot 的话题名改成对应话题即可。版本切换 (ROS1↔ROS2) 会自动重建订阅以适配消息类型字符串。无硬件测试时 mock 节点会发布移动彩条测试图 (`/image_raw/compressed` 需装 Pillow, 否则发 raw `/usb_cam/image_raw`)。

## 功能

| 功能 | 说明 |
|------|------|
| 关节滑块 | 5 个机械臂关节 + 夹爪 + 4 车轮, 实时驱动模型 |
| 预设位姿 | SRDF 定义的 5 种姿态一键切换 |
| 夹爪开合 | 6 个夹爪关节联动 |
| 逆运动学 | CCD 算法, 输入 XYZ 目标坐标自动求解关节角 |
| 末端位姿 | 实时显示 link5 世界坐标与 RPY |
| **实时相机** | **双画面, 订阅 CompressedImage / raw Image, 支持 rgb8/bgr8/mono8** |
| ROS 同步 | /joint_states → 仿真镜像 |
| 指令下发 | 仿真 → voice_joint_states → 真机执行 |
| 底盘控制 | 方向按钮 → cmd_vel → 真机移动 |

## 文件结构

```
wheeltec-sim2real/
├── public/meshes/        # 16 个 STL 网格 (来自 URDF)
├── src/
│   ├── RobotModel.js     # URDF 运动学树 + STL 加载 (sim2real 核心)
│   ├── SceneSetup.js     # Three.js 场景/灯光/地面/相机
│   ├── RosBridge.js      # roslibjs 桥接 (ROS1/ROS2 双协议自适应)
│   ├── IKSolver.js       # CCD 逆运动学求解器
│   ├── CameraView.js     # ★ 实时相机画面 (CompressedImage/raw Image 解码)
│   ├── UIController.js   # DOM 控件绑定 (含 ROS 版本切换 + 相机面板)
│   ├── main.js           # 入口
│   └── style.css         # 样式
├── ros2/                 # ★ ROS2 串口桥接包 (sim2real 真机端)
│   └── src/wheeltec_sim2real_bridge/
│       ├── wheeltec_sim2real_bridge/
│       │   ├── serial_bridge.py     # 串口桥接 (移植自 wheeltec_arm_six.cpp)
│       │   └── mock_joint_states.py # 无硬件闭环测试节点
│       ├── launch/                  # sim2real_bridge + rosbridge 启动
│       ├── config/params.yaml       # 串口/帧ID 参数
│       └── package.xml / setup.py
├── index.html
├── vite.config.js
└── package.json
```

## ROS2 串口桥接包 (`ros2/wheeltec_sim2real_bridge`)

由 ROS1 `wheeltec_arm_pick` 的 `turn_on_robot` (wheeltec_arm_six.cpp) + `quaternion_solution.cpp` 移植为 rclpy + pyserial 节点。

**串口协议 (与 STM32 下位机逐字节一致):**
- 底盘下行帧 11B: `0x7B | 产品型号 | 使能 | vx_hi vx_lo | vy_hi vy_lo | wz_hi wz_lo | XOR | 0x7D` (速度 ×1000 有符号定点)
- 机械臂下行帧 16B: `0xAA | (j1..j6 各 hi,lo) | 模式 | XOR | 0xBB` (角度 ×1000)
- 上行帧 24B: `0x7B | 停止位 | (vx,vy,wz 各 hi,lo) | (ax,ay,az,gx,gy,gz 各 hi,lo) | 电压 hi,lo | XOR | 0x7D`
- 校验: 前 N 字节按位异或; IMU 加速度 /16384, 陀螺仪 ×0.00026644

**节点:**
| 节点 | 话题订阅 | 话题发布 | 说明 |
|------|---------|---------|------|
| `serial_bridge` | `voice_joint_states`, `cmd_vel` | `odom`, `imu`, `PowerVoltage`, `pose` | 真机串口桥接 |
| `mock_joint_states` | `voice_joint_states`, `cmd_vel` | `/joint_states`, `/odom` | 无硬件 mock, 回显指令 + 差速积分 |

> 真机不回传机械臂关节编码值 (24B 上行帧不含臂关节, 已在 STM32 固件 `usartx.c` 确认), 因此仿真镜像的 `/joint_states` 由 `mock_joint_states` (无硬件) 或 MoveIt2 的 `joint_state_publisher` (真机) 提供, 与 ROS1 行为一致。也可在 `serial_bridge` 开启 `echo_joint_states` 参数, 把 `voice_joint_states` 指令值回显到 `/joint_states` 作占位。

## 部署拓扑 (重要: 串口独占)

下位机 STM32 用**一条串口** `/dev/wheeltec_controller` 同时处理底盘帧 (0x7B) 与机械臂帧 (0xAA), 因此**同一时刻只能有一个节点打开该串口**。据你真机 ROS2 系统的现状, 选对应部署方式:

### 方式 A: 真机已有 `turn_on_wheeltec_robot` 全功能桥接 (推荐先查)
WHEELTEC ROS2 镜像通常自带 `turn_on_wheeltec_robot` 包 (即 ROS1 `wheeltec_arm_six` 的 ROS2 版), 已订阅 `cmd_vel`/`voice_joint_states` 并发布 `odom`/`imu`。此时**不要**再跑 `serial_bridge` (会争抢串口)。只需:
```bash
# 真机端 (已有底层节点运行)
ros2 launch rosbridge_server rosbridge_websocket.launch.xml port:=9090
```
浏览器选 ROS2 → 连接 → 勾选同步即可。`/joint_states` 由 MoveIt2 `joint_state_publisher` 提供; 若没跑 MoveIt2, 用 `mock_joint_states` 的关节回显分支或 `serial_bridge echo_joint_states:=true` (但后者会争串口, 仅在没有 turn_on_wheeltec_robot 时用)。

诊断真机是否已有桥接:
```bash
ros2 topic info /voice_joint_states      # 看是否有订阅者 (1 个以上 = 已有桥接)
ros2 node list | grep -i wheeltec        # 看是否有 turn_on_wheeltec_robot 节点
```

### 方式 B: 真机 ROS2 无桥接 (从 D:\a ROS1 源码迁移的场景)
用本包 `serial_bridge` 作为完整底盘+机械臂桥接 (替代 ROS1 `wheeltec_arm_six`):
```bash
ros2 launch wheeltec_sim2real_bridge sim2real_bridge.launch.py
# 关节回显 (无 MoveIt2 时, 让仿真有 /joint_states 镜像):
#   编辑 config/params.yaml 设 echo_joint_states: true, 或命令行:
ros2 run wheeltec_sim2real_bridge serial_bridge --ros-args -p echo_joint_states:=true
```

### 方式 C: 无硬件联调
```bash
ros2 launch wheeltec_sim2real_bridge sim2real_bridge.launch.py mock:=true
```
`mock_joint_states` 回显 `voice_joint_states`→`/joint_states` + 差速积分 `cmd_vel`→`/odom` + 测试彩条相机画面, 浏览器端可跑通完整 sim2real 闭环。

## ROS 话题对照

| 方向 | 话题 | 类型 (ROS1 / ROS2) | 用途 |
|------|------|------|------|
| 真机→仿真 | `/joint_states` | sensor_msgs/JointState · sensor_msgs/msg/JointState | 关节状态镜像 |
| 真机→仿真 | `/odom` | nav_msgs/Odometry · nav_msgs/msg/Odometry | 底盘位姿同步 |
| 仿真→真机 | `voice_joint_states` | sensor_msgs/JointState · sensor_msgs/msg/JointState | 机械臂目标关节角 |
| 仿真→真机 | `cmd_vel` | geometry_msgs/Twist · geometry_msgs/msg/Twist | 底盘速度指令 |

> `voice_joint_states` 是 WHEELTEC 系统中 `preset.cpp` / `voice_control.cpp` 使用的机械臂控制话题, 串口节点订阅此话题将指令下发至下位机。ROS2 版 `wheeltec_sim2real_bridge/serial_bridge.py` 沿用同一话题名与 6 值顺序 `[joint1..joint5, joint6(夹爪)]`, 帧格式 (0xAA 帧头 / XOR 校验 / ×1000 定点) 与 STM32 下位机完全一致。
