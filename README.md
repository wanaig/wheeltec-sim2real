# WHEELTEC mini_4wd_six_arm · Sim2Real 数字孪生

基于 **Vite + Three.js** 构建的 WHEELTEC R550A 6 自由度机械臂(四驱底盘)三维数字孪生系统,直接从 URDF 解析运动学树并加载真实 STL 网格,通过 roslibjs 与真机 ROS 双向通信,实现 sim2real。

在 sim2real 之上,内置一套 **LLM 智能体自主作业系统**:大模型(GPT-4o / DeepSeek / 通义千问 / 智谱 / 本地 Ollama 等 OpenAI 兼容 API)通过 function calling 调用 9 个 MCP 工具(`perceive / move_base / plan_arm_motion / grasp / release / retract / verify / get_robot_state / get_scene_info`),在工业双工作台场景中完成「自然语言指令 → 感知 → 规划 → 抓取 → 放置 → 验证」全流程,含 MoveIt2 风格的多起始 IK + 碰撞检测 + 经由点轨迹规划、A* 底盘路径规划与 4 级失败恢复。无 API Key 时自动回退到正则 NLU 模式。

**同时支持 ROS1 (Melodic/Noetic) 与 ROS2 (Foxy/Humble/Jazzy)**:UI 左上角「ROS 版本」下拉框切换,前端自动适配消息类型字符串 (`sensor_msgs/JointState` ↔ `sensor_msgs/msg/JointState`) 与时间戳字段 (`secs/nsecs` ↔ `sec/nanosec`)。配套 ROS2 串口桥接包见 [`ros2/`](./ros2),真机部署完整流程见 [`ros2/DEPLOY.md`](./ros2/DEPLOY.md)。

## 快速开始

```bash
cd wheeltec-sim2real
npm install
npm run dev          # 浏览器打开 http://localhost:5173
```

生产构建:
```bash
npm run build && npm run preview
```

启动后即可在浏览器内使用:键盘控制机器人、跑 LLM 智能体自主作业、生成 YOLO 数据集,无需连接真机。连接真机见下文「真机端准备」。

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

## 键盘控制

仿真与真机共用一套键位(底盘用方向键,机械臂用数字/字母键,互不冲突):

| 键 | 功能 |
|---|---|
| `↑` `↓` `←` `→` | 底盘前进 / 后退 / 左转 / 右转 (4WD 差速) |
| `1` / `Q` | joint1 底座偏航 (减 / 增) |
| `2` / `W` | joint2 肩部俯仰 (增 / 减) |
| `3` / `E` | joint3 肘部 (减 / 增) |
| `4` / `R` | joint4 腕部俯仰 (减 / 增) |
| `5` / `T` | joint5 腕部滚转 (减 / 增) |
| `6` / `Y` | 夹爪 (开 / 闭) |
| `0` | 一键归零位 |

按住持续运动(关节 0.6 rad/s,夹爪 0.5/s),松开停止;任意键接管时自动取消 IK/tween 动画。连接 ROS 且勾选「将仿真指令发送至真机」时,键盘操作会以 ~10Hz 节流下发 `voice_joint_states` + `cmd_vel`。

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

1. 安装 rosbridge_server 与 serial 依赖:
```bash
sudo apt install ros-<distro>-rosbridge-server python3-serial
# <distro> = foxy / humble / jazzy
```
2. 构建配套串口桥接包 [`ros2/wheeltec_sim2real_bridge`](./ros2) (由 ROS1 `wheeltec_arm_six` 移植, 协议逐字节一致):
```bash
cd ros2
colcon build --symlink-install
source install/setup.bash
```
3. 启动 (URDF/TF + 串口桥接 + rosbridge 一条命令):
```bash
# 真机
ros2 launch wheeltec_sim2real_bridge bringup.launch.py echo_joint_states:=true
# 无硬件闭环测试 (mock 节点回显指令, 跑通全流程)
ros2 launch wheeltec_sim2real_bridge bringup.launch.py mock:=true
```
4. 赋予串口权限 (真机):
```bash
sudo chmod 666 /dev/wheeltec_controller
```

> 旧版 `sim2real_bridge.launch.py` 仍可用(仅串口/mock + rosbridge,不含 URDF/TF 发布);`bringup.launch.py` 为推荐的总启动。完整部署到小车的步骤(scp 传输、依赖安装、串口 udev、故障排查)见 [`ros2/DEPLOY.md`](./ros2/DEPLOY.md)。

#### ROS1 (Melodic/Noetic, 兼容)

在小车 ROS 端启动 rosbridge:
```bash
# 安装 (首次)
sudo apt install ros-melodic-rosbridge-server
# 启动
roslaunch rosbridge_server rosbridge_websocket.launch
```
确保 `base_serial.launch` 已启动 (机械臂串口节点 + MoveIt):
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
| Image (raw) | `sensor_msgs/msg/Image` | 原始图, 支持 rgb8/bgr0/mono8 (~10fps) |

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

## LLM 智能体自主作业系统

浏览器内置一套完整的「自然语言 → 自主作业」系统,无需真机即可演示,也可下写真机执行。入口在画面右侧「🤖 交互型智能体」面板。

### 双模式 (自动切换)

- **LLM 模式**(有 API Key):大模型决策 → MCP 工具调用 → MoveIt2 规划 → 执行 → 结果回传 → 继续决策,循环直到完成或达到最大轮数(默认 30 轮)。
- **正则模式**(无 API Key):正则 NLU 解析意图(动作/工具/位置/格子)→ 仿真感知 → 规划 → IK 执行 → 失败重试。

在面板填写 `API Base / API Key / Model` 后保存即启用 LLM 模式(配置持久化到 localStorage),可点「验证大模型连接」测试。兼容 OpenAI / DeepSeek / 通义千问 / 智谱 / 本地 Ollama 等所有 OpenAI 格式 API。

### MCP 工具集 (9 个, OpenAI function calling 格式)

| 工具 | 功能 |
|------|------|
| `perceive` | 感知场景中所有可见工具/零件,返回类别、世界坐标、距离、可达性;不可达时附带 `suggested_chassis` |
| `get_scene_info` | 返回双工作台位置、料箱格子坐标、臂展范围、碰撞盒、安全高度 |
| `get_robot_state` | 返回底盘位姿、臂关节角、夹爪状态、TCP 世界坐标 |
| `move_base` | 底盘导航到世界坐标(x,y,yaw),先自动收回机械臂到安全高度再 A* 路径规划避障行驶 |
| `plan_arm_motion` | MoveIt2 风格规划并执行臂运动到目标 XYZ:多起始点 IK + 关节限位 + 环境碰撞检测 + 抬升/水平/下降经由点轨迹,一次调用完成规划+执行 |
| `grasp` | 闭合夹爪,抓取距离 TCP 最近(12cm 内)的工具 |
| `release` | 张开夹爪,在 TCP 位置放置当前抓取的工具 |
| `retract` | 收回机械臂:多策略垂直抬升(逆序/多解 IK/后退脱离)到安全高度,抓取/放置后必须调用 |
| `verify` | 验证抓取(type=grasp,工具是否离开原位)或放置(type=place,工具是否入指定格子) |

### 工业场景与作业流程

3D 场景为双工作台布局:左台料箱(3 格)、右台工具散乱区,中间为机械臂小车大作业区(底盘可按 footprint 避障自由选最近站位)。

典型 `pick_and_place` 流程:感知 → 检查可达性(不可达则按 `suggested_chassis` 调用 `move_base`)→ `plan_arm_motion` 到目标正上方再垂直下降 → `grasp` → `retract` → 移动到料箱 → `plan_arm_motion` 到格子 → `release` → `retract` → `verify`。

失败恢复策略:IK 不可达 / 轨迹碰撞 → 按 `suggested_chassis` 换站位重试;retract 卡死 → 先 `release` 或 `move_base` 脱离;同一目标失败 2 次后换同类目标;最多 4 次重试。

### 智能体 ROS 话题 (真机联调时)

`AgentPanel` 通过 roslibjs 订阅/发布,版本自适应:

| 方向 | 话题 | 类型 | 用途 |
|------|------|------|------|
| 订阅 | `/agent/status` | std_msgs/String (JSON `{status, info}`) | 顶部大字幕横幅(运行中/完成/失败) |
| 订阅 | `/agent/log` | std_msgs/String | 逐步日志 + 阶段字幕(感知/决策/执行/重试) |
| 发布 | `/agent/instruction` | std_msgs/String | 自然语言指令输入 |

无 ROS 连接时,面板自动切到 Mock 模式,直接驱动浏览器内 `MockAgent` 跑全流程,日志/字幕/状态全部本地回显,适合演示视频录制。

### 场景布局切换与数据集生成

面板下方提供:

- **4 种布局**一键切换:有序摆放 / 混杂摆放 / 混合场景 / 随机生成
- **重置场景** / **模拟放置失败**(下次 release 偏移,演示失败恢复)
- **生成数据集**按钮:多布局 × 多视角(8 个相机预设:俯视/正视/左侧/右侧/四斜视)× random 多种子,渲染 640×480 图像,3D 包围盒投影生成 YOLO 归一化标注,打包下载 `images/*.jpg + labels/*.txt + data.yaml`,可直接用于 YOLO 训练。类别 ID 与 `SceneLayouts.js` 的 `TOOL_CLASS_ID` 对齐(0:螺丝刀 1:扳手 2:螺母 3:滚柱 4:螺丝)。

## 功能

| 功能 | 说明 |
|------|------|
| 关节滑块 | 5 个机械臂关节 + 夹爪 + 4 车轮, 实时驱动模型 |
| 预设位姿 | SRDF 定义的 5 种姿态一键切换 |
| 夹爪开合 | 6 个夹爪关节联动 |
| 逆运动学 | CCD 算法, 输入 XYZ 目标坐标自动求解关节角 |
| 末端位姿 | 实时显示 link5 世界坐标与 RPY |
| **键盘控制** | **方向键底盘 + 1-5/Q-T 关节 + 6/Y 夹爪 + 0 归零, 仿真/真机共用** |
| **实时相机** | **双画面, 订阅 CompressedImage / raw Image, 支持 rgb8/bgr8/mono8** |
| **LLM 智能体** | **OpenAI 兼容 API + 9 个 MCP 工具 + function calling 自主作业** |
| **MoveIt2 规划仿真** | **多起始 IK + 环境碰撞检测 + 经由点轨迹 + 关节限位** |
| **工业场景** | **双工作台 + 料箱 + 5 种工具 3D 模型 + 4 种布局** |
| **A* 底盘导航** | **4WD 差速运动学 + 路径规划避障 + 比例控制制动** |
| **数据集生成** | **多视角渲染 + YOLO 标注 + 批量下载, 直接用于训练** |
| ROS 同步 | /joint_states → 仿真镜像 |
| 指令下发 | 仿真 → voice_joint_states → 真机执行 |
| 底盘控制 | 方向按钮 / 键盘 → cmd_vel → 真机移动 |

## 文件结构

```
wheeltec-sim2real/
├── public/meshes/        # 16 个 STL 网格 (来自 URDF)
├── src/
│   ├── RobotModel.js          # URDF 运动学树 + STL 加载 (sim2real 核心)
│   ├── SceneSetup.js          # Three.js 场景/灯光/地面/相机
│   ├── RosBridge.js           # roslibjs 桥接 (ROS1/ROS2 双协议自适应)
│   ├── IKSolver.js            # CCD 逆运动学求解器
│   ├── ChassisController.js   # ★ 底盘运动学 (4WD 差速 + 键盘 + A* 导航 + odom 覆盖)
│   ├── ArmKeyboardController.js # ★ 机械臂/夹爪键盘控制 (1-5/Q-T/6/Y/0)
│   ├── CameraView.js          # 实时相机画面 (CompressedImage/raw Image 解码)
│   ├── LLMAgent.js            # ★ 大模型智能体 (OpenAI 兼容 API + function calling 循环)
│   ├── MCPTools.js            # ★ MCP 工具集 + 执行器 (9 工具, MoveIt2 规划仿真)
│   ├── MockAgent.js           # ★ 浏览器内全流程智能体 (LLM/正则双模式 + 工业场景 + A* + 4级恢复)
│   ├── SceneLayouts.js        # ★ 5 种工具 3D 模型 + 4 种场景布局 + YOLO 标注元数据
│   ├── DatasetGenerator.js    # ★ 多视角渲染 + YOLO 标注 + 批量下载
│   ├── AgentPanel.js          # ★ 交互型智能体面板 (字幕 + 日志 + 指令 + LLM 配置)
│   ├── UIController.js        # DOM 控件绑定 (含 ROS 版本切换 + 相机面板)
│   ├── main.js                # 入口, 装配所有模块
│   └── style.css              # 样式
├── ros2/                 # ROS2 工作空间 (sim2real 真机端)
│   ├── README.md              # 包组成 / 串口协议 / 构建运行
│   ├── DEPLOY.md              # ★ 完整部署到小车流程 (scp/依赖/串口/故障排查)
│   └── src/
│       ├── mini_4wd_six_arm/              # URDF 模型包
│       │   ├── urdf/mini_4wd_six_arm.urdf
│       │   ├── meshes/*.STL               # 16 个真实网格
│       │   ├── launch/display.launch.py   # robot_state_publisher [+ jsp + rviz2]
│       │   └── config/joint_names_*.yaml
│       └── wheeltec_sim2real_bridge/      # 串口桥接包 (移植自 ROS1 wheeltec_arm_six.cpp)
│           ├── wheeltec_sim2real_bridge/
│           │   ├── serial_bridge.py       # 真机串口桥接: cmd_vel/voice_joint_states→STM32, 串口→odom/imu
│           │   └── mock_joint_states.py   # 无硬件 mock (回显 /joint_states + odom + 测试相机)
│           ├── launch/
│           │   ├── bringup.launch.py      # ★ 总启动: URDF/TF + 串口/mock + rosbridge 一条命令
│           │   ├── sim2real_bridge.launch.py
│           │   └── rosbridge_websocket.launch.py
│           ├── config/params.yaml         # 串口/帧ID 参数
│           └── package.xml / setup.py
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

### 方式 B: 真机 ROS2 无桥接 (从 ROS1 源码迁移的场景)
用本包 `serial_bridge` 作为完整底盘+机械臂桥接 (替代 ROS1 `wheeltec_arm_six`):
```bash
ros2 launch wheeltec_sim2real_bridge bringup.launch.py echo_joint_states:=true
# 关节回显 (无 MoveIt2 时, 让仿真有 /joint_states 镜像):
#   编辑 config/params.yaml 设 echo_joint_states: true, 或命令行:
ros2 run wheeltec_sim2real_bridge serial_bridge --ros-args -p echo_joint_states:=true
```

### 方式 C: 无硬件联调
```bash
ros2 launch wheeltec_sim2real_bridge bringup.launch.py mock:=true
```
`mock_joint_states` 回显 `voice_joint_states`→`/joint_states` + 差速积分 `cmd_vel`→`/odom` + 测试彩条相机画面, 浏览器端可跑通完整 sim2real 闭环。

> 完整的传输工作空间到小车、安装依赖、串口 udev 规则、启动验证、故障排查等步骤见 [`ros2/DEPLOY.md`](./ros2/DEPLOY.md)。

## ROS 话题对照

| 方向 | 话题 | 类型 (ROS1 / ROS2) | 用途 |
|------|------|------|------|
| 真机→仿真 | `/joint_states` | sensor_msgs/JointState · sensor_msgs/msg/JointState | 关节状态镜像 |
| 真机→仿真 | `/odom` | nav_msgs/Odometry · nav_msgs/msg/Odometry | 底盘位姿同步 |
| 仿真→真机 | `voice_joint_states` | sensor_msgs/JointState · sensor_msgs/msg/JointState | 机械臂目标关节角 |
| 仿真→真机 | `cmd_vel` | geometry_msgs/Twist · geometry_msgs/msg/Twist | 底盘速度指令 |
| 仿真↔智能体 | `/agent/status` `/agent/log` `/agent/instruction` | std_msgs/String · std_msgs/msg/String | 智能体状态/日志/指令 |

> `voice_joint_states` 是 WHEELTEC 系统中 `preset.cpp` / `voice_control.cpp` 使用的机械臂控制话题, 串口节点订阅此话题将指令下发至下位机。ROS2 版 `wheeltec_sim2real_bridge/serial_bridge.py` 沿用同一话题名与 6 值顺序 `[joint1..joint5, joint6(夹爪)]`, 帧格式 (0xAA 帧头 / XOR 校验 / ×1000 定点) 与 STM32 下位机完全一致。
