# WHEELTEC mini_4wd_six_arm · Sim2Real 数字孪生

基于 **Vite + Three.js** 构建的 WHEELTEC R550A 6 自由度机械臂(四驱底盘)三维数字孪生系统,直接从 URDF 解析运动学树并加载真实 STL 网格,通过 roslibjs 与真机 ROS 双向通信,实现 sim2real。

 在 sim2real 之上,内置一套 **LLM 智能体自主作业系统**:基于 **LangGraph** `StateGraph`(`agent ↔ tools` 循环)编排大模型(GPT-4o / DeepSeek / 通义千问 / 智谱 / 本地 Ollama 等 OpenAI 兼容 API)通过 function calling 调用 9 个 MCP 工具(`perceive / move_base / plan_arm_motion / grasp / release / retract / verify / get_robot_state / get_scene_info`),在工业双工作台场景中完成「自然语言指令 → 感知 → 规划 → 抓取 → 放置 → 验证」全流程,含 MoveIt2 风格的多起始 IK + 碰撞检测 + 经由点轨迹规划、A* 底盘路径规划与 4 级失败恢复。无 API Key 时自动回退到正则 NLU 模式。原 `LLMAgent.js`(手写 fetch 循环)保留作 fallback。

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

**默认话题 (双相机: 外部 USB 摄像头 + 手眼 Astra S):**
- 手眼相机: `/camera/color/image_raw/compressed` (CompressedImage) — Orbbec Astra S 经 ROS1 `astra_camera` 驱动 + `ros1_bridge` 桥接进 ROS2 的彩色压缩流 (见下文「手眼相机 (Astra S)」)
- 外部相机: `/eye_to_hand/image_raw/compressed` (CompressedImage) — `dual_camera_bridge` 读取 `/dev/RgbCam` (USB 摄像头) 发布的压缩彩色流

**常见话题参考:**

| 相机 | 话题 (压缩 / 原始) |
|------|------|
| 手眼 Astra S (经桥接) | `/camera/color/image_raw/compressed` · `/camera/color/image_raw` |
| USB 摄像头 (外部) | `/eye_to_hand/image_raw/compressed` · `/eye_to_hand/image_raw` |
| Astra 深度图 | `/camera/depth/image_raw/compressed` · `/camera/depth/image_raw` (mono8/16) |

> 若你的真机有两个相机, 直接把两个 slot 的话题名改成对应话题即可。版本切换 (ROS1↔ROS2) 会自动重建订阅以适配消息类型字符串。无硬件测试时 mock 节点会发布移动彩条测试图 (`/image_raw/compressed` 需装 Pillow, 否则发 raw `/usb_cam/image_raw`)。

### 手眼相机 (Astra S) — ROS1 驱动 + ros1_bridge 桥接

手眼相机是装在机械臂末端的 **Orbbec Astra S** 深度相机。它**没有 UVC RGB 接口**, OpenCV/V4L2 读不了, 必须用 ROS1 `astra_camera` 驱动 (走 OpenNI2/libPS1080) 才能取 RGB。当前 ROS2 bringup 跑的是 foxy, 因此用 **ros1_bridge** 把 ROS1 的 Astra RGB 压缩流桥接进 ROS2:

```
Astra S --(OpenNI2)--> astra_camera(ROS1) --> /camera/color/image_raw
     --> image_transport republish --> /camera/color/image_raw/compressed (ROS1)
     --> ros1_bridge dynamic_bridge --bridge-all-1to2-topics
     --> /camera/color/image_raw/compressed (ROS2)
     --> rosbridge_websocket --> 浏览器前端 (手眼相机槽位)
```

**部署 (在小车上一次性配置):**
```bash
# 1. 安装 ros1_bridge (仅首次)
sudo apt install ros-foxy-ros1-bridge
# 2. 传输 ros2 工作空间到小车并构建 (脚本随包安装到 share/scripts/)
#    (见 ros2/DEPLOY.md 的完整传输/构建流程, 需 --symlink-install)
```

**运行 (推荐: 集成进 bringup, 一条命令全起):**
```bash
# 小车上 — ROS2 主体 + 外部相机 + Astra 手眼相机, 一次启动
ros2 launch wheeltec_sim2real_bridge bringup.launch.py astra_hand:=true echo_joint_states:=true
# Ctrl+C 退出时, launch 会自动清理 Astra 链路的全部子进程 (roscore/astra/republish/bridge)
```
启动后浏览器前端「手眼相机」槽位 (默认话题 `/camera/color/image_raw/compressed`, CompressedImage) 即可实时显示 Astra 的 RGB 画面;「外部相机」槽位显示 `/eye_to_hand/...`。

**运行 (备选: 手动独立启动, 不走 launch):**
```bash
# 先启动 ROS2 bringup (不含 astra_hand), 再单独跑脚本
ros2 launch wheeltec_sim2real_bridge bringup.launch.py echo_joint_states:=true
~/start_astra_hand_camera.sh start    # 或: ros2/src/wheeltec_sim2real_bridge/scripts/start_astra_hand_camera.sh start
~/start_astra_hand_camera.sh status   # 查看状态 + ROS2 话题
~/start_astra_hand_camera.sh stop     # 停止
```

> **Astra S 已知问题:** Astra S 在 Jetson USB2.0 上偶尔会卡死 (astra 日志报 `USB transfer timeout`), 软件复位无法恢复, 需**物理拔插 Astra 的 USB 线或重启小车**。启动脚本 (supervise 模式含一次 USB 软件复位 + astra 两次重试 + 彩色流健康检查) 失败时会明确提示并退出, 不影响 bringup 其它节点。日志在 `/tmp/astra_hand_{roscore,astra,republish,bridge}.log`。

## 完整操作流程 (PC 传文件 → 小车启动 → 浏览器连接)

以下从零开始, 把电脑端的 `ros2/` 工作空间部署到 WHEELTEC 小车并启动 sim2real 双向通信 + 双相机画面。

### 前提

- 小车 IP (本文以 `10.103.12.250` 为例, 用户 `wheeltec`, 密码 `dongguan`)
- 小车已装 ROS2 Foxy + ROS1 Noetic (WHEELTEC Jetson 镜像自带)
- 小车已装 ros1_bridge (首次部署时执行一次): `sudo apt install ros-foxy-ros1-bridge`
- PC 已装 Node.js + npm

### 步骤 1: PC 端传输 ros2/ 到小车 (覆盖旧文件)

在 PC 项目根目录 `wheeltec-sim2real/` 下执行:

```bash
# 方式一: 只传改动的桥接包 (快, 推荐)
scp -r ros2/src/wheeltec_sim2real_bridge wheeltec@10.103.12.250:~/ros2/src/

# 方式二: 传整个 ros2/ 工作空间 (含 URDF 模型包 + 桥接包, 首次部署用)
ssh wheeltec@10.103.12.250 "rm -rf ~/ros2"     # 先删旧工作空间, 避免嵌套
scp -r ros2 wheeltec@10.103.12.250:~/           # ros2/ → ~/ros2/
```

> **避坑:** 如果 `~/ros2/` 已存在, 直接 `scp -r ros2 wheeltec@IP:~/ros2/` 会产生嵌套 `~/ros2/ros2/`, 导致 `colcon build` 报 `Duplicate package names`。要么先删 (`rm -rf ~/ros2`) 再传, 要么只传 `src/` 下的包目录。若已嵌套, 删掉即可: `rm -rf ~/ros2/ros2`。

### 步骤 2: 小车端构建

```bash
ssh wheeltec@10.103.12.250
cd ~/ros2
colcon build --symlink-install --packages-select wheeltec_sim2real_bridge
```

> 首次构建或改了 URDF/STL 时, 用 `colcon build --symlink-install` (不带 `--packages-select`) 构建全部包。

### 步骤 3: 小车端启动

```bash
# 首次需 source ROS2 环境 (或写入 ~/.bashrc 永久生效)
source /opt/ros/foxy/setup.bash
source ~/ros2/install/setup.bash

# 永久生效 (可选, 以后开终端自动 source)
echo 'source /opt/ros/foxy/setup.bash' >> ~/.bashrc
echo 'source ~/ros2/install/setup.bash' >> ~/.bashrc

# 启动! URDF/TF + 串口桥接 + rosbridge + 外部相机 + Astra 手眼相机, 一条命令
ros2 launch wheeltec_sim2real_bridge bringup.launch.py astra_hand:=true echo_joint_states:=true
```

启动后终端应看到: `rosbridge_websocket` (端口 9090 就绪)、`serial_bridge` (串口已打开)、`dual_camera_bridge` (外部相机发布中)、`astra_hand_camera` (Astra 彩色流就绪)。`Ctrl+C` 退出时 launch 自动清理全部子进程。

> **串口权限:** 若 serial_bridge 报串口打不开, 执行 `sudo chmod 666 /dev/wheeltec_controller`。
>
> **Astra 卡死:** 若 astra 日志报 `USB transfer timeout`, 物理拔插 Astra 的 USB 线后重新 launch。
>
> **端口占用:** 启动前确认没有旧的 bringup 在跑 (会占 9090 端口和串口), 有则先 `Ctrl+C` 停掉。

### 步骤 4: PC 端启动前端并连接

```bash
cd wheeltec-sim2real
npm install        # 首次
npm run dev        # 浏览器打开 http://localhost:5173
```

浏览器内操作:

1. 左上角「ROS 版本」→ 选 **ROS2**
2. WebSocket 地址 → 填 `ws://10.103.12.250:9090`
3. 点击「连接」→ 状态变绿
4. 勾选「实时同步真机状态」→ 仿真模型镜像真机 `/joint_states` + `/odom`
5. 勾选「将仿真指令发送至真机」→ 滑块/键盘/IK 指令下发 `voice_joint_states` + `cmd_vel`
6. 底部「实时相机画面」→ 两个 slot 分别勾选「启用」:
   - 手眼相机 (默认 `/camera/color/image_raw/compressed`, CompressedImage) → Astra RGB 画面
   - 外部相机 (默认 `/eye_to_hand/image_raw/compressed`, CompressedImage) → USB 摄像头画面

### 常用启动参数

| 参数 | 默认 | 说明 |
|------|------|------|
| `astra_hand` | `false` | `true` 时拉起 Astra 手眼相机桥接链路 (ROS1 驱动 + ros1_bridge, 含 RGB+depth 流) |
| `rgbd_detector` | `false` | `true` 时启动 YOLO+RGB-D 工业检测节点 (需 `astra_hand:=true` 提供 depth 流) |
| `locate_anything` | `false` | `true` 时启动 LocateAnything 外置推理客户端 (**与 `rgbd_detector` 二选一**)。Jetson 把 RGB+自然语言 prompt 发到外置 RTX 上的 `server.py`, 收 2D bbox 后本地 depth+TF 做 2D→3D 反投影, 发同样的 `/industrial_tools/annotations`。需 `astra_hand:=true` + 外置机运行 `locate_anything_server/server.py` |
| `la_server_url` | `http://192.168.0.100:8765` | 外置机 LocateAnything HTTP 服务地址 |
| `la_http_timeout` | `3.0` | 外置机 HTTP 请求超时 (秒) |
| `la_default_prompt` | `''` | 默认 prompt (空=不检测, 等前端发 `/locate/query`) |
| `locate_query_topic` | `/locate/query` | 自然语言 prompt 输入话题 (前端 LLM `perceive(query)` 发布) |
| `yolo_model` | `yolov8n.pt` | YOLO 模型文件 (COCO 预训练)。需 `pip install ultralytics`; 未安装时自动回退到 `yolov5s.pt` |
| `yolo_conf` | `0.45` | YOLO 置信度阈值 |
| `yolo_device` | `cuda:0` | YOLO 推理设备 (`cuda:0` / `cpu`) |
| `yolo_imgsz` | `640` | YOLO 推理图像尺寸 |
| `echo_joint_states` | `false` | `true` 时 serial_bridge 把 `voice_joint_states` 指令值回显到 `/joint_states` (无 MoveIt2 时让仿真有关节镜像) |
| `mock` | `false` | `true` 时用 mock 节点替代串口 (无硬件联调) |
| `hand_cam_x/y/z` | `0.05/0/0.03` | 手眼相机挂载位置 (link5→camera_link 静态 TF, 需 `astra_hand:=true`) |
| `hand_cam_roll/pitch/yaw` | `0/-0.3/0` | 手眼相机挂载姿态 (pitch 负值=向下看桌面) |

```bash
# 不带手眼相机 (仅外部相机)
ros2 launch wheeltec_sim2real_bridge bringup.launch.py echo_joint_states:=true
# 真机 + Astra 手眼 + YOLO RGB-D 工业检测 (完整工业感知)
ros2 launch wheeltec_sim2real_bridge bringup.launch.py astra_hand:=true rgbd_detector:=true echo_joint_states:=true
# 真机 + Astra + LocateAnything 外置推理 (开放词汇 grounding, 跑在外置 RTX 上, 见下节)
ros2 launch wheeltec_sim2real_bridge bringup.launch.py astra_hand:=true locate_anything:=true \
  la_server_url:=http://192.168.0.100:8765 echo_joint_states:=true
# 无硬件闭环测试
ros2 launch wheeltec_sim2real_bridge bringup.launch.py mock:=true
```

### YOLO + RGB-D 工业检测与 sim2real 数字孪生

基于手眼 Astra S 的 RGB-D 流 + YOLO 目标检测,实时检测桌面物体,标注类别与世界坐标,并**同步真实画面为 3D 彩色点云**到 Three.js 场景,实现 sim2real 数字孪生闭环。

**架构: ROS2 + OpenCV + YOLO + RGB-D → Three.js 彩色点云数字孪生**

```
Astra S (eye-in-hand)
  │ RGB /camera/color/image_raw/compressed        ─┐
  │ Depth /camera/depth/image_raw/compressed       │ ROS1→ROS2 (dynamic_bridge)
  │ CameraInfo /camera/color/camera_info           ─┘
  ▼
rgbd_tool_detector (YOLO 推理: bounding box + class + confidence
                     → depth ROI 中值 → 3D 坐标 → TF 变换到 base_link)
  │ YOLO 后端自动检测: ultralytics(YOLOv8) 或 torch.hub(YOLOv5, 本地 ~/yolov5-pytorch)
  │ TF 变换: camera_color_optical_frame → base_link (世界坐标)
  ▼
/industrial_tools/annotations (JSON: class, confidence, position_world_m, depth_m, bbox_xywh, ...)
/industrial_tools/debug_image/compressed (带 YOLO 框+标签+世界坐标的调试图)
/industrial_tools/depth_viz/compressed (8-bit 深度 JPEG, 供浏览器生成点云)
  │ rosbridge_websocket
  ▼
浏览器 (两条并行链路):
  ┌─────────────────────────────────────────────────────────────────┐
  │ 链路1: YOLO 检测 → 物体级数字孪生                                │
  │  AgentPanel._onToolAnnotations → MockAgent.setRealAnnotations   │
  │  → _syncRealTools(): 为每个真实检测创建 3D Mesh → this.tools    │
  │  → grasp/release/verify 全链路打通                              │
  │  → 绿色标签 = 真实检测, 黄色标签 = 虚拟布局                     │
  │  → 5s 无新检测 → 自动恢复虚拟布局                               │
  ├─────────────────────────────────────────────────────────────────┤
  │ 链路2: RGB-D 点云 → 像素级数字孪生                              │
  │  RealEnvView: 订阅 color JPEG + depth JPEG + camera_info        │
  │  → 逐像素 back-project (深度→3D坐标, 内参 fx/fy/cx/cy)         │
  │  → ~19000 彩色点 (降采样 1/16)                                  │
  │  → RobotModel.getCameraOpticalPose() FK → 世界坐标系定位        │
  │  → THREE.Points 实时渲染 (真实环境点云 = 3D 场景中的真实世界)   │
  └─────────────────────────────────────────────────────────────────┘
  ▼
Three.js 3D 场景 = 真实环境数字孪生:
  - 机器人 URDF 模型 (关节角 = 真机实时镜像)
  - 真实 RGB-D 彩色点云 (桌面物体的真实 3D 外观)
  - YOLO 检测物体 3D Mesh (类别+世界坐标, 可抓取)
  - 虚拟工作台/料箱 (工业场景布局)
  ▼
LLM 智能体自主作业: perceive (真实类别+坐标) → plan_arm_motion (IK) → grasp → release → verify
```

**启动:**
```bash
ros2 launch wheeltec_sim2real_bridge bringup.launch.py \
  astra_hand:=true rgbd_detector:=true echo_joint_states:=true
# 自定义 YOLO 模型/参数:
ros2 launch wheeltec_sim2real_bridge bringup.launch.py \
  astra_hand:=true rgbd_detector:=true \
  yolo_model:=yolov8n.pt yolo_conf:=0.5 yolo_device:=cuda:0
```

**YOLO 后端 (自动检测):**
- **ultralytics (YOLOv8)**: `pip install ultralytics` 后自动使用,默认模型 `yolov8n.pt`
- **YOLOv5 (torch.hub)**: 未安装 ultralytics 时自动回退,从 `~/yolov5-pytorch/` 本地加载 `yolov5s.pt`
- 两者都使用 COCO 80 类预训练,后续可替换为自训练工业工具模型

**输出标注格式** (`/industrial_tools/annotations`, JSON):
```json
{
  "stamp": 1234567890.0,
  "frame_id": "camera_color_optical_frame",
  "world_frame": "base_link",
  "detector": "yolo",
  "yolo_backend": "yolov5",
  "objects": [
    {
      "id": 0,
      "class": "bottle", "class_cn": "瓶子", "class_id": 39,
      "confidence": 0.87,
      "bbox_xywh": [100, 200, 80, 120],
      "center_px": [140.0, 260.0],
      "position_camera_m": [0.12, -0.05, 0.45],
      "position_world_m": [0.35, 0.02, 0.08],
      "position_table_m": [0.35, 0.02, 0.08],
      "depth_m": 0.45,
      "industrial": true
    }
  ]
}
```

**手眼标定 TF 链:**
```
base_link → joint1 → ... → joint5 → link5
  → [static TF] camera_link (挂载偏移, 可通过 launch 参数 hand_cam_x/y/z/roll/pitch/yaw 调整)
  → [astra publish_tf] camera_color_frame → camera_color_optical_frame
```
> 默认挂载参数 (`x=0.05, z=0.03, pitch=-0.3`) 为近似值,实际安装不同时覆盖 launch 参数或做手眼标定后更新。`position_world_m` 为 `base_link` 坐标系 (与 Three.js 场景坐标系一致: X前/Y左/Z上, 单位米)。

**检测类别**: COCO 80 类预训练 (person/car/bottle/knife/scissors/...),工业相关类高亮标记 (`industrial: true`)。后续可训练自定义工业工具模型 (screwdriver/wrench/nut/screw/...) 替换 `yolo_model` 参数。

> **YOLO 推理**: 在 Jetson Orin Nano 8GB 上使用 CUDA + TensorRT,YOLOv5s 约 20+ FPS。模型文件自动从本地 `~/yolov5-pytorch/` 加载 (无需联网),或从 GitHub 下载 (首次使用)。

> **前端显示**: AgentPanel「检测物体」区域, 真实检测结果标注 `[YOLO]` 标签, 虚拟场景感知无标签。perceive 工具返回结果中 `real: true` 表示来自 YOLO+RGB-D。

### LocateAnything 外置推理 (开放词汇 grounding)

YOLO 用固定 COCO 80 类, 识别不了「蓝色方形工件」这类自然语言属性描述。`locate_anything` 链路换用 **NVIDIA LocateAnything-3B** (`nvidia/LocateAnything-3B`) 开放词汇 grounding 模型, 接收任意自然语言 prompt 输出目标 2D bbox。该模型 ~4B(BF16), Jetson Orin Nano 8GB 跑不动, 故**推理跑在外置 RTX 工控机上**, Jetson 只负责相机采集 + 2D→3D 反投影 + 运动控制。

**为什么 3D 反投影留 Jetson**: depth 流 + TF 树 (`base_link←camera`) 都在小车上, 把一帧 JPEG 发出去、收回几个 bbox, 远比把 depth + 实时 TF 同步传给外置机便宜。重模型离板, 廉价几何运算在板。

**架构 (三端分布式):**

```
PC 前端 (Vite+Three.js)              Jetson 小车 (ROS2 Foxy)           外置机 (RTX 工控机)
─────────────────────              ──────────────────────────       ─────────────────────
用户: "抓取蓝色方形工件"              Astra S RGB/depth/camera_info      LocateAnything-3B
  │                                  │                                  (nvidia/LocateAnything-3B)
  │ LLM 调 perceive(query=...)       │ /locate/query ◄── prompt          │
  └─ /locate/query ─rosbridge──────►│ locate_anything_client            │
                                     │  ── JPEG+prompt (HTTP) ───────► │ 推理: <box> 坐标
                                     │  ◄── 2D bbox ────────────────── │
                                     │ depth ROI 中值 + 内参 → camera 3D
                                     │ TF: camera→base_link → XYZ
                                     │ /industrial_tools/annotations ─►│
  ◄── annotations (rosbridge) ──────┘                                  │
  MockAgent.setRealAnnotations → 3D 场景同步
  perceive 返回真实物体 → LLM 选目标
  → plan_arm_motion → grasp → release → verify
```

**① 外置机 (RTX 工控机, Linux + NVIDIA GPU)**

```bash
cd locate_anything_server
pip install -r requirements.txt   # transformers==4.57.1 peft decord lmdb + torch(按CUDA)
# 提前下载模型 (离线部署, 约 8GB):
huggingface-cli download nvidia/LocateAnything-3B --local-dir ./models/la-3b
# 启动 (加载本地, 不联网):
BACKEND=locateanything MODEL_PATH=./models/la-3b DEVICE=cuda:0 PORT=8765 python3 server.py
# 验证: curl http://<外置机IP>:8765/health
```

后端可插拔 (`BACKEND=`): `locateanything` (生产) / `yoloworld` (ultralytics 开放词汇, 零配置验证全链路) / `groundingdino` (transformers)。`server.py` 暴露 `POST /detect {image_base64, prompt}` → `{boxes:[{bbox, class, confidence}]}`。

**② Jetson 小车** — `locate_anything_client` 节点 (继承 `rgbd_tool_detector`, 复用全部图像解码/depth→3D/TF/发布逻辑, 仅把检测步骤换成 HTTP 调外置机):

```bash
ros2 launch wheeltec_sim2real_bridge bringup.launch.py \
  astra_hand:=true locate_anything:=true \
  la_server_url:=http://<外置机IP>:8765 echo_joint_states:=true
```

**③ PC 前端** — `npm run dev`, 浏览器内 LLM 输入「抓取蓝色方形工件」→ `perceive(query)` 自动经 `AgentPanel.publishLocateQuery` 发布 `/locate/query`, 等待外置检测回传 `/industrial_tools/annotations` 后注入 `MockAgent` 完成 sim2real 闭环。

**与 YOLO 链路的差异:**

| 项 | YOLO (`rgbd_detector`) | LocateAnything (`locate_anything`) |
|----|------------------------|-------------------------------------|
| 检测器 | 本地 YOLOv8/v5 (COCO 80 类) | 外置 LocateAnything-3B (开放词汇) |
| 输入 | 无 (全场景检测) | 自然语言 prompt (`/locate/query`) |
| 输出 | bbox + 类别 + 置信度 | bbox + prompt 文本 (无 per-box conf, 填 1.0) |
| 算力位置 | Jetson 本地 GPU | 外置 RTX |
| `annotations` 话题 | 相同格式 | 相同格式 (额外带 `query` 字段) |
| 前端/工具链 | 不变 | 不变 (`perceive` 加可选 `query` 参数) |

> **License**: nvidia/LocateAnything-3B 为 NVIDIA License, **仅非商业/学术研究**。模型含自定义代码, 需 `trust_remote_code=True`, 离线目录须保留其 `.py` 文件。不支持 TensorRT。硬件支持 Ampere/Hopper/Blackwell/Lovelace (A100/H100/L40/RTX 4090)。

> **时序**: `perceive(query)` 发 prompt 后 `await 2.5s` 等 Jetson 周期检测 (5Hz) + HTTP 往返 + annotations 回传。若 LocateAnything-3B 推理 >2s, 调大 `MCPTools.js` `_perceive` 里的等待值。

> **二选一**: `locate_anything:=true` 与 `rgbd_detector:=true` 不要同时启用 (会重复发 `/industrial_tools/annotations`)。

### 自然语言指令理解 (InstructionParser)

基于开源大模型的指令解析模块, 精准理解工业场景简单自然语言指令 (如「取出左侧的扳手」「帮我把滚柱放到料箱的第三个格子中」「抓取蓝色方形工件」), 提取核心需求 (目标工具、动作要求、位置、料箱格)。

**为什么用开源 LLM 而非 BERT**: BERT 槽填充需训练工业指令样本, 工作量大且泛化弱; Qwen2.5-3B-Instruct 靠 few-shot prompt 即可输出结构化 JSON, 无需训练, 还能理解「蓝色方形工件」这类颜色+形状属性描述。模型与 LocateAnything-3B 同语言模型基座, 外置机权重复用性好。

### 任务序列分解 (TaskDecomposer)

将解析后的结构化指令 `{action, target_tool, side, slot, query}` 分解为显式任务步骤列表, 每步映射到对应 MCP 工具, 在 AgentPanel「任务分解」区域以步骤列表形式实时展示进度 (○ 待执行 / ◉ 执行中 / ✓ 完成 / ✗ 失败)。

**分解逻辑** (以「把滚柱放到料箱第三格」为例):

```
步骤1  环境感知与目标定位          → perceive       识别场景工具, 定位滚柱世界坐标
步骤2  可达性评估与底盘站位规划    → move_base      [可选] 若不可达, A*导航到最优站位
步骤3  抓取路径规划与避障          → plan_arm_motion 多起始IK+碰撞检测+经由点轨迹
步骤4  生成抓取姿态指令           → grasp          下降闭合夹爪抓取滚柱
步骤5  机械臂安全收回             → retract        垂直抬升到安全高度
步骤6  底盘导航至料箱区域         → move_base      [可选] 靠近料箱第3格
步骤7  放置路径规划               → plan_arm_motion 规划到格子坐标
步骤8  生成放置姿态指令           → release        张开夹爪放入第3格
步骤9  机械臂安全收回             → retract        放置后抬升收回
步骤10 放置结果验证               → verify         检查滚柱是否入格
步骤11 失败自主感知与重试         → replan         [条件] 未入格则重新抓取放置
```

**进度跟踪**: LLM 模式从 `[MCP]` 日志解析工具调用/结果, NLU 模式从 `[nlu] 步骤N` 日志跟踪, 失败重试从 `[replan]` 日志触发步骤回退。步骤状态实时更新: ◉ 执行中 (脉冲动画) → ✓ 完成 / ✗ 失败, 演示视频中直观可见「感知→决策→执行→验证→重试」全流程。

**模块**: `src/TaskDecomposer.js` (分解逻辑 + 日志解析工具方法)

**输出结构**:

```json
{
  "action": "grasp|place",              // 动作: 取/抓/拿=grasp, 放/放置=place
  "target_tool": "wrench|roller|...|null", // 标准工具类 (5类), 无法映射=null
  "side": "left|right|null",            // 位置: 左/右
  "slot": 1-3|null,                     // 料箱格
  "query": "蓝色方形工件",              // 视觉定位描述 (含颜色/形状), 供 LocateAnything
  "source": "llm|regex"                 // 解析来源: LLM 或正则回退
}
```

**后端可切换 (OpenAI 兼容, 一键切换)**:
- ☁ **云端 API**: `apiBase = https://api.openai.com/v1` (或 DeepSeek/通义千问/智谱等 OpenAI 格式)
- 🖥 **本地外置机**: `apiBase = http://<外置机IP>:11434/v1` (Ollama) 或 `:8001/v1` (vLLM), 本地大模型推理跑在外置 RTX 上, 与 LocateAnything 同理 (重推理离板)
- 正则回退: 无 API Key 时用内置正则解析 (与 `MockAgent.parseInstruction` 一致)

AgentPanel「大模型 + MCP 工具调用」配置区有快捷按钮: ☁云端 API / 🖥本地外置机 (填外置机 IP), 一键切换 apiBase/model。

**与执行链结合 (独立解析 + 兼容现有 agent)**:

```
用户: "抓取蓝色方形工件"
  │ InstructionParser.parse (LLM 云端/本地外置机, 或正则)
  ├→ [NLU] 理解结果: 动作=grasp 目标=按query定位 query="蓝色方形工件"
  ├→ publishLocateQuery(query) → /locate/query → Jetson locate_anything_client → 外置 LocateAnything
  └→ mockAgent.run(text, plan):
       LLM 模式: LangGraphAgent.run(text, plan) — plan 注入初始消息, 引导 perceive 用 plan.query 定位
       正则模式: MockAgent.runNLU(text, plan) — 复用 plan, 跳过 parseInstruction
```

**外置机本地大模型 serving 部署** (`locate_anything_server/llm_serving/`):

```bash
cd locate_anything_server/llm_serving
# 方式 A: vLLM (生产, 高吞吐, OpenAI 兼容 /v1)
./download_qwen.sh      # 提前下载 Qwen2.5-3B-Instruct (~6GB, 离线部署)
./start_vllm.sh         # 启动 :8001/v1, served-model-name=qwen2.5-3b-instruct
# 方式 B: Ollama (快速验证, 易用, OpenAI 兼容 /v1)
./start_ollama.sh       # 启动 :11434/v1, model=qwen2.5:3b
```

启动后前端点「🖥本地外置机」按钮填外置机 IP, 保存即切换。本地模式 apiKey 填任意非空串 (Ollama/vLLM 不校验 key)。两个 serving 与 `server.py` (LocateAnything, :8765) 并列跑, 端口不冲突。

**模块**: `src/InstructionParser.js` (前端解析模块, OpenAI 兼容 `chat/completions` + JSON 输出 + 正则回退)。

### 语音输入 (SenseVoice ASR)

PTT (push-to-talk) 按住说话语音识别: 浏览器用 `MediaRecorder` 采集音频 (webm/opus), POST 到外置机 ASR 服务, 收到识别文本后自动填入指令框并触发 `InstructionParser` 解析 → agent 执行, 实现「语音 → 文本 → 理解 → 规划 → 执行」全语音链路。ASR 推理跑在外置 RTX 上, 与 LocateAnything / LLM serving 同理。

**架构:**

```
浏览器 (🎤 按住说话)               外置机 (RTX)
  │ MediaRecorder → webm/opus        │
  │ POST /asr (multipart)  ─────────►│ SenseVoice-Small (FunASR)
  │                                  │ ffmpeg webm→16kHz wav
  │                                  │ 非自回归推理 (~70ms/10s 音频)
  │◄────────── {text, language,      │ emotion, event, latency_ms}
  │   emotion/event (SenseVoice 特色)│
  ▼
InstructionParser.parse(text) → {action, target_tool, side, slot, query}
  └→ mockAgent.run(text, plan) → 感知 → 规划 → 抓取 → 放置 → 验证
```

**外置机 ASR 服务部署** (`locate_anything_server/asr_server/`):

```bash
cd locate_anything_server/asr_server
pip install -r requirements.txt   # funasr + modelscope + torch + torchaudio
# 系统依赖: ffmpeg (用于 webm→wav 转码), sudo apt install ffmpeg
# 提前下载模型 (离线部署, ~900MB):
./download_sensevoice.sh          # ModelScope → iic/SenseVoiceSmall → ./models/sensevoice
# 启动:
./start_asr.sh                    # BACKEND=sensevoice DEVICE=cuda:0 PORT=8766
# 验证: curl http://<外置机IP>:8766/health
```

后端可插拔 (`BACKEND=`): `sensevoice` (默认, 阿里 FunASR, 50+ 语言 + 情绪 + 音频事件) / `whisper` (faster-whisper CTranslate2, fallback)。`server.py` 暴露 `POST /asr` (multipart file) 和 `POST /asr_base64` → `{text, language, emotion, event, latency_ms}`。

**端口规划 (外置机):**

| 端口 | 服务 | 说明 |
|------|------|------|
| 8765 | LocateAnything | 视觉定位 (2D bbox) |
| 8766 | SenseVoice ASR | 语音识别 (文本) |
| 8001 | vLLM | 本地 LLM (OpenAI 兼容) |
| 11434 | Ollama | 本地 LLM (OpenAI 兼容) |

前端 AgentPanel 指令输入框旁有 🎤 麦克风按钮, **按住说话、松开发送**。ASR 地址自动复用 LLM 配置区的外置机 IP (`:8766`), 无需单独配置。浏览器要求 HTTPS 或 localhost 才能访问麦克风 (本地 `npm run dev` 可直接用)。

**模块**: `src/SpeechRecognizer.js` (PTT 录音 + 上传, 状态回调 idle/recording/processing)

## LLM 智能体自主作业系统

浏览器内置一套完整的「自然语言 → 自主作业」系统,无需真机即可演示,也可下写真机执行。入口在画面右侧「🤖 交互型智能体」面板。

### 双模式 (自动切换)

- **LLM 模式**(有 API Key):基于 **LangGraph** `StateGraph` 编排 —— `agent` 节点(ChatOpenAI + 9 工具)生成下一动作 → `tools` 节点(MCPToolExecutor 调度)执行 → 条件边回 `agent` 或 `END`,循环直到完成/熔断/达最大轮数(默认 100)。`tools` 节点内置重复调用检测(近 3 次相同 tool+params 跳过)、连续失败熔断(5 次)、同目标 `plan_arm_motion` 熔断(3 次)、阈值警告注入。
- **正则模式**(无 API Key):正则 NLU 解析意图(动作/工具/位置/格子)→ 仿真感知 → 规划 → IK 执行 → 失败重试。

在面板填写 `API Base / API Key / Model` 后保存即启用 LLM 模式(配置持久化到 localStorage),可点「验证大模型连接」测试。兼容 OpenAI / DeepSeek / 通义千问 / 智谱 / 本地 Ollama 等所有 OpenAI 格式 API。

> 框架:`LangGraphAgent.js` 用 `@langchain/langgraph` 的 `StateGraph` + `@langchain/openai` 的 `ChatOpenAI` 实现,9 个 MCP 工具以 `DynamicStructuredTool`(Zod schema)注册给模型。`LLMAgent.js`(原生 fetch + 手写 while 循环)保留,在 `main.js` 注释切换即可回退。

### MCP 工具集 (9 个, OpenAI function calling 格式)

| 工具 | 功能 |
|------|------|
| `perceive` | 感知场景中所有可见工具/零件,返回类别、世界坐标、距离、可达性;不可达时附带 `suggested_chassis`。**可选 `query` 参数** (如"蓝色方形工件") 触发外置 LocateAnything 开放词汇检测:经 `/locate/query` 下发到 Jetson `locate_anything_client`, 等待 `/industrial_tools/annotations` 回传后返回真实目标。**数据源自动切换**:有真实检测结果 (5s 内, `/industrial_tools/annotations`) 时用真实数据,否则回退虚拟场景 |
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
| 发布 | `/locate/query` | std_msgs/String | LocateAnything 自然语言 prompt (perceive(query) → Jetson locate_anything_client → 外置机) |

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
| **LLM 智能体** | **LangGraph StateGraph + OpenAI 兼容 API + 9 个 MCP 工具 + function calling 自主作业** |
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
│   ├── LLMAgent.js            # ★ 原生 fetch + function-calling 循环 (fallback, 导出 SYSTEM_PROMPT)
│   ├── LangGraphAgent.js      # ★ LangGraph StateGraph 智能体 (agent↔tools 循环, 主用)
│   ├── MCPTools.js            # ★ MCP 工具集 + 执行器 (9 工具, MoveIt2 规划仿真)
│   ├── MockAgent.js           # ★ 浏览器内全流程智能体 (LLM/正则双模式 + 工业场景 + A* + 4级恢复)
│   ├── SceneLayouts.js        # ★ 5 种工具 3D 模型 + 4 种场景布局 + YOLO 标注元数据
│   ├── DatasetGenerator.js    # ★ 多视角渲染 + YOLO 标注 + 批量下载
│   ├── AgentPanel.js          # ★ 交互型智能体面板 (字幕 + 日志 + 指令 + LLM 配置)
│   ├── UIController.js        # DOM 控件绑定 (含 ROS 版本切换 + 相机面板)
│   ├── InstructionParser.js  # ★ 自然语言指令解析 (LLM 云端/本地 + 正则回退, 输出 action/tool/side/slot/query)
│   ├── TaskDecomposer.js     # ★ 任务序列分解 (解析结果→显式步骤列表, 映射MCP工具, 实时进度跟踪)
│   ├── SpeechRecognizer.js   # ★ 语音识别 (PTT 按住说话, MediaRecorder→外置 SenseVoice ASR)
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
│           │   ├── mock_joint_states.py   # 无硬件 mock (回显 /joint_states + odom + 测试相机)
│           │   ├── dual_camera_bridge.py  # 双 USB 摄像头 → CompressedImage (eye_in_hand / eye_to_hand)
│           │   ├── rgbd_tool_detector.py  # ★ YOLO+RGB-D 工业检测 (本地 YOLO, depth→3D→TF→annotations)
│           │   └── locate_anything_client.py # ★ LocateAnything 外置推理客户端 (HTTP 调外置机, 本地 2D→3D)
│           ├── launch/
│           │   ├── bringup.launch.py      # ★ 总启动: URDF/TF + 串口/mock + rosbridge 一条命令
│           │   ├── sim2real_bridge.launch.py
│           │   └── rosbridge_websocket.launch.py
│           ├── config/params.yaml         # 串口/帧ID 参数
│           └── package.xml / setup.py
├── locate_anything_server/   # ★ 外置机 LocateAnything 推理服务 (独立, 不依赖 ROS)
│   ├── server.py             # FastAPI HTTP: POST /detect {image_base64, prompt} → {boxes}
│   ├── backends.py           # 可插拔后端: LocateAnything-3B / YOLO-World / GroundingDINO
│   ├── requirements.txt      # transformers==4.57.1 peft decord lmdb + torch
│   ├── start_locate.sh       # ★ 启动 LocateAnything (:8765, 自动检测 models/la-3b)
│   ├── DEPLOY.md             # ★ 外置机完整部署文档 (依赖/模型下载/启动/故障排查)
│   ├── llm_serving/          # ★ 本地大模型 serving (OpenAI 兼容, 推理在 RTX)
│       ├── start_vllm.sh     # vLLM Qwen2.5-3B-Instruct (:8001/v1)
│       ├── start_ollama.sh   # Ollama qwen2.5:3b (:11434/v1)
│       └── download_qwen.sh  # 提前下载 Qwen2.5-3B-Instruct 权重
├── locate_anything_server/asr_server/  # ★ 外置机语音识别服务 (SenseVoice, :8766, 独立)
│   ├── server.py             # FastAPI: POST /asr (multipart) → {text, language, emotion, event}
│   ├── backends.py           # 可插拔后端: SenseVoice (FunASR) / Whisper (faster-whisper)
│   ├── requirements.txt      # funasr + modelscope + torch + torchaudio
│   ├── start_asr.sh          # 启动 SenseVoice ASR (:8766)
│   └── download_sensevoice.sh # 提前下载 iic/SenseVoiceSmall
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
| 仿真→Jetson | `/locate/query` | std_msgs/String · std_msgs/msg/String | LocateAnything 自然语言 prompt (perceive(query) → 外置推理) |
| Jetson→仿真 | `/industrial_tools/annotations` | std_msgs/String · std_msgs/msg/String | YOLO/LocateAnything 检测结果 JSON (类别+世界坐标+bbox) |

> `voice_joint_states` 是 WHEELTEC 系统中 `preset.cpp` / `voice_control.cpp` 使用的机械臂控制话题, 串口节点订阅此话题将指令下发至下位机。ROS2 版 `wheeltec_sim2real_bridge/serial_bridge.py` 沿用同一话题名与 6 值顺序 `[joint1..joint5, joint6(夹爪)]`, 帧格式 (0xAA 帧头 / XOR 校验 / ×1000 定点) 与 STM32 下位机完全一致。
