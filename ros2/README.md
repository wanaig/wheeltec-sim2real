# WHEELTEC mini_4wd_six_arm ROS2 Foxy sim2real

这个 `ros2/` 是给你的机械臂小车真机使用的 ROS2 工作空间，目标系统是：

- Ubuntu 20.04.6 LTS focal
- ROS2 Foxy
- WHEELTEC mini_4wd_six_arm / R550A 六自由度机械臂四驱小车
- STM32 下位机串口 `/dev/wheeltec_controller`，波特率 `115200`

前端虚拟世界在项目根目录 `wheeltec-sim2real/`，通过 rosbridge 连接本 ROS2 工作空间，实现浏览器仿真和真机双向通信。

## 实现内容

```text
Windows 浏览器 Three.js 虚拟世界
  └── roslibjs WebSocket ws://小车IP:9090
      └── rosbridge_server (ROS2 Foxy)
          ├── voice_joint_states -> serial_bridge -> 0xAA 机械臂串口帧 -> STM32
          ├── cmd_vel            -> serial_bridge -> 0x7B 底盘串口帧 -> STM32
          ├── /odom              <- serial_bridge <- STM32 24B 上行帧
          ├── /imu               <- serial_bridge <- STM32 24B 上行帧
          ├── /PowerVoltage      <- serial_bridge <- STM32 24B 上行帧
          └── /joint_states      <- serial_bridge 机械臂指令回显
```

## 包结构

```text
ros2/src/
├── mini_4wd_six_arm/
│   ├── urdf/mini_4wd_six_arm.urdf
│   ├── meshes/*.STL
│   └── launch/display.launch.py
└── wheeltec_sim2real_bridge/
    ├── wheeltec_sim2real_bridge/serial_bridge.py
    ├── wheeltec_sim2real_bridge/mock_joint_states.py
    ├── config/params.yaml
    └── launch/bringup.launch.py
```

## 串口协议

协议按 `D:\a\melodic_wheeltec_arm_20240411\wheeltec_arm\src\wheeltec_arm_pick\src\wheeltec_six_arm_pick\wheeltec_arm_six.cpp` 重写到 ROS2。

| 方向 | 长度 | 格式 |
|---|---:|---|
| 上位机 -> 底盘 | 11B | `0x7B product enable vx_hi vx_lo vy_hi vy_lo wz_hi wz_lo xor 0x7D` |
| 上位机 -> 机械臂 | 16B | `0xAA j1_hi j1_lo ... j6_hi j6_lo mode xor 0xBB` |
| STM32 -> 上位机 | 24B | `0x7B stop vx vy wz ax ay az gx gy gz voltage xor 0x7D` |

速度和角度都是 `value * 1000` 的有符号 int16，高字节在前。校验是前 N 字节异或。

## 话题

| 方向 | 话题 | 类型 | 说明 |
|---|---|---|---|
| 前端 -> 真机 | `voice_joint_states` | `sensor_msgs/msg/JointState` | position[0..5] = joint1..joint5 + joint6 夹爪 |
| 前端 -> 真机 | `cmd_vel` | `geometry_msgs/msg/Twist` | 底盘速度指令 |
| 真机 -> 前端 | `/odom` | `nav_msgs/msg/Odometry` | 底盘里程计 |
| 真机 -> 前端 | `/imu` | `sensor_msgs/msg/Imu` | IMU |
| 真机 -> 前端 | `/PowerVoltage` | `std_msgs/msg/Float32` | 电池电压 |
| 真机 -> 前端 | `/joint_states` | `sensor_msgs/msg/JointState` | 机械臂开环指令回显 + 车轮估算 |

注意：旧 STM32 上行 24B 帧没有机械臂编码器角度，所以 `/joint_states` 的机械臂部分是“下发指令回显”，不是编码器真实反馈。

## 构建

在小车 Ubuntu 20.04 上：

```bash
sudo apt update
sudo apt install -y ros-foxy-rosbridge-suite \
  ros-foxy-robot-state-publisher ros-foxy-joint-state-publisher \
  python3-colcon-common-extensions python3-serial python3-opencv

cd ~/ros2
source /opt/ros/foxy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 真机启动

```bash
cd ~/ros2
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch wheeltec_sim2real_bridge bringup.launch.py
```

默认会启动：

- `robot_state_publisher`
- `serial_bridge`
- `rosbridge_websocket`，端口 `9090`
- `dual_camera_bridge`，发布手眼和外部双路相机画面

默认不会启动 `joint_state_publisher`，因为 `serial_bridge` 已经发布 `/joint_states` 回显。

## 双摄像头

真机默认读取两个摄像头设备：

| 相机 | 默认设备 | 默认压缩图像话题 |
|---|---|---|
| 手眼相机 | `/dev/HandCam` | `/eye_in_hand/image_raw/compressed` |
| 外部相机 | `/dev/RgbCam` | `/eye_to_hand/image_raw/compressed` |

旧 WHEELTEC 工程里的 RGB 摄像头默认设备是 `/dev/RgbCam`，本项目继续沿用它作为外部眼；手眼相机新增使用 `/dev/HandCam`。

如果你还没有固定设备名，可以先临时软链接：

```bash
ls -l /dev/video*
sudo ln -sf /dev/video0 /dev/HandCam
sudo ln -sf /dev/video1 /dev/RgbCam
```

也可以启动时直接指定设备：

```bash
ros2 launch wheeltec_sim2real_bridge bringup.launch.py \
  eye_in_hand_device:=/dev/video0 \
  eye_to_hand_device:=/dev/video1
```

如果暂时不启动相机：

```bash
ros2 launch wheeltec_sim2real_bridge bringup.launch.py cameras:=false
```

## RGB-D 工业工具感知与标注

可选启动 `rgbd_tool_detector`，基于 RGB-D 相机获取桌面工具/零件信息，输出类别、像素框、相机坐标和桌面坐标，用于模拟工业现场和构建数据集。

默认输入话题：

| 输入 | 默认话题 |
|---|---|
| RGB 图像 | `/camera/rgb/image_raw` |
| 深度图像 | `/camera/depth/image_raw` |
| 相机内参 | `/camera/rgb/camera_info` |

输出话题：

| 输出 | 话题 | 类型 |
|---|---|---|
| 工具 JSON 标注 | `/industrial_tools/annotations` | `std_msgs/msg/String` |
| 带框调试图 | `/industrial_tools/debug_image/compressed` | `sensor_msgs/msg/CompressedImage` |

启动：

```bash
ros2 launch wheeltec_sim2real_bridge bringup.launch.py rgbd_detector:=true
```

如果 RGB-D 相机话题不同：

```bash
ros2 launch wheeltec_sim2real_bridge bringup.launch.py rgbd_detector:=true \
  rgbd_rgb_topic:=/camera/color/image_raw \
  rgbd_depth_topic:=/camera/aligned_depth_to_color/image_raw \
  rgbd_camera_info_topic:=/camera/color/camera_info
```

验证：

```bash
ros2 topic echo /industrial_tools/annotations
ros2 topic hz /industrial_tools/debug_image/compressed
```

前端连接 rosbridge 后，智能体面板“检测物体”会自动显示 `/industrial_tools/annotations` 中的工具类别、位置坐标、置信度和像素框；也可以把底部任一相机槽话题改为 `/industrial_tools/debug_image/compressed` 查看检测框画面。

## 前端连接

Windows 上：

```powershell
cd C:\Users\aaa\Desktop\test\wheeltec-sim2real
npm install
npm run dev
```

浏览器打开 `http://localhost:5173`：

1. ROS 版本选择 `ROS2`
2. 地址填写 `ws://小车IP:9090`
3. 点击连接
4. 勾选“将仿真指令发送至真机”
5. 需要真机镜像回仿真时，勾选“实时同步真机状态”

## 无硬件测试

```bash
ros2 launch wheeltec_sim2real_bridge bringup.launch.py mock:=true
```

mock 模式会订阅 `voice_joint_states` 和 `cmd_vel`，回发 `/joint_states`、`/odom`，并发布测试相机画面，方便先验证前端和 rosbridge 链路。

## 常用验证命令

```bash
ros2 topic list
ros2 topic info /voice_joint_states
ros2 topic info /cmd_vel
ros2 topic echo /odom
ros2 topic echo /joint_states
ros2 topic echo /PowerVoltage
ros2 topic hz /eye_in_hand/image_raw/compressed
ros2 topic hz /eye_to_hand/image_raw/compressed
ros2 topic echo /industrial_tools/annotations
ros2 topic hz /industrial_tools/debug_image/compressed
```

手动测试底盘：

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

手动测试机械臂：

```bash
ros2 topic pub --once /voice_joint_states sensor_msgs/msg/JointState \
  "{name: [joint1, joint2, joint3, joint4, joint5, joint6], position: [0.0, 0.0, 0.0, 1.57, 0.0, 0.0]}"
```

## 重要注意

- 同一时刻只能有一个节点打开 `/dev/wheeltec_controller`。
- 如果小车上已经跑着 WHEELTEC 原厂 ROS1/ROS2 底层串口节点，必须先停掉，否则本 `serial_bridge` 打不开串口。
- 如果实际串口不是 `/dev/wheeltec_controller`，改 `src/wheeltec_sim2real_bridge/config/params.yaml` 的 `usart_port_name`。
- Ubuntu 20.04 对应 ROS2 Foxy，不要按 Humble 文档安装依赖。
