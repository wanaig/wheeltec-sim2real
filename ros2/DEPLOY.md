# 部署到 Ubuntu 20.04 小车

你的机器人系统是 Ubuntu 20.04.6 LTS focal，所以 ROS2 使用 Foxy。

## 1. 小车安装依赖

```bash
sudo apt update
sudo apt install -y ros-foxy-rosbridge-suite \
  ros-foxy-robot-state-publisher ros-foxy-joint-state-publisher \
  python3-colcon-common-extensions python3-serial python3-opencv
```

如果没有安装 ROS2 Foxy，先按 Foxy 官方源安装 `/opt/ros/foxy`。

## 2. 从 Windows 传输 ros2 工作空间

PowerShell：

```powershell
cd "C:\Users\aaa\Desktop\test\wheeltec-sim2real"
tar -czf ros2_workspace.tar.gz ros2
scp ros2_workspace.tar.gz wheeltec@小车IP:~/
```

小车端：

```bash
cd ~
tar -xzf ros2_workspace.tar.gz
```

## 3. 串口检查

```bash
ls -l /dev/wheeltec_controller
sudo chmod 666 /dev/wheeltec_controller
```

如果设备名是 `/dev/ttyUSB0`，二选一：

```bash
sudo ln -sf /dev/ttyUSB0 /dev/wheeltec_controller
```

或修改：

```bash
nano ~/ros2/src/wheeltec_sim2real_bridge/config/params.yaml
```

把 `usart_port_name` 改成实际设备。

## 4. 双摄像头检查

本项目默认使用两个摄像头设备名：

| 相机 | 设备名 | 前端话题 |
|---|---|---|
| 手眼相机 | `/dev/HandCam` | `/eye_in_hand/image_raw/compressed` |
| 外部相机 | `/dev/RgbCam` | `/eye_to_hand/image_raw/compressed` |

旧 WHEELTEC 工程默认 RGB 摄像头是 `/dev/RgbCam`，这里继续作为外部眼使用。

先查看摄像头：

```bash
ls -l /dev/video*
```

如果还没有固定别名，可以先临时软链接：

```bash
sudo ln -sf /dev/video0 /dev/HandCam
sudo ln -sf /dev/video1 /dev/RgbCam
```

如果两个摄像头顺序相反，就交换 `/dev/video0` 和 `/dev/video1`。

## 5. 构建

```bash
cd ~/ros2
source /opt/ros/foxy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 6. 启动真机 sim2real

```bash
cd ~/ros2
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch wheeltec_sim2real_bridge bringup.launch.py
```

如果需要临时指定摄像头设备：

```bash
ros2 launch wheeltec_sim2real_bridge bringup.launch.py \
  eye_in_hand_device:=/dev/video0 \
  eye_to_hand_device:=/dev/video1
```

如果暂时不启用摄像头：

```bash
ros2 launch wheeltec_sim2real_bridge bringup.launch.py cameras:=false
```

启动成功应看到类似信息：

```text
serial opened: /dev/wheeltec_controller @ 115200
wheeltec ROS2 Foxy sim2real bridge started
eye_to_hand camera: /dev/RgbCam -> /eye_to_hand/image_raw/compressed
eye_in_hand camera: /dev/video0 -> /eye_in_hand/image_raw/compressed
```

如果 `/dev/HandCam` 不存在，新版 `dual_camera_bridge` 会继续尝试 `/dev/video*`，并且会先打开外部眼 `/dev/RgbCam`，再把剩余可出帧的摄像头分配给手眼相机。如果日志里只看到 `eye_in_hand camera open failed: /dev/HandCam` 后没有继续尝试 `/dev/video*`，说明小车端还在跑旧安装产物，需要重新传代码、删除 `build install log` 后重建并重新 `source install/setup.bash`。

## 7. Windows 前端连接

```powershell
cd "C:\Users\aaa\Desktop\test\wheeltec-sim2real"
npm install
npm run dev
```

浏览器打开 `http://localhost:5173`：

1. ROS 版本选 `ROS2`
2. WebSocket 地址填 `ws://小车IP:9090`
3. 点击连接
4. 勾选“将仿真指令发送至真机”
5. 可选勾选“实时同步真机状态”
6. 底部相机面板默认订阅“手眼相机”和“外部相机”，连接 ROS 后会自动开始显示

## 8. 验证

小车端看话题：

```bash
ros2 topic list
ros2 topic echo /odom
ros2 topic echo /joint_states
ros2 topic echo /PowerVoltage
ros2 topic info /voice_joint_states
ros2 topic info /cmd_vel
ros2 topic hz /eye_in_hand/image_raw/compressed
ros2 topic hz /eye_to_hand/image_raw/compressed
```

前端控制机械臂时，`/voice_joint_states` 应有发布者，`wheeltec_arm_six` 应是订阅者。

前端控制底盘时，`/cmd_vel` 应有发布者，`wheeltec_arm_six` 应是订阅者。

## 9. 无硬件联调

```bash
cd ~/ros2
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch wheeltec_sim2real_bridge bringup.launch.py mock:=true
```

这个模式不打开串口，用 ROS2 内部 mock 回显 `/joint_states` 和 `/odom`，用于验证前端、rosbridge、话题类型和 WebSocket。

## 10. 故障排查

| 现象 | 处理 |
|---|---|
| `ros2: command not found` | 没 source：`source /opt/ros/foxy/setup.bash` |
| 找不到包 | 构建后没 source：`source ~/ros2/install/setup.bash` |
| rosbridge 安装失败 | Ubuntu 20.04 应安装 `ros-foxy-rosbridge-suite` |
| 串口打不开 | 检查 `/dev/wheeltec_controller`、权限、是否被旧节点占用 |
| 前端连不上 | 检查小车 IP、9090 端口、防火墙、rosbridge 是否启动 |
| 机械臂不动 | 确认勾选“将仿真指令发送至真机”，看 `/voice_joint_states` |
| 底盘不动 | 看 `/cmd_vel`，确认串口桥接节点没有报错 |
| 手眼相机黑屏 | 确认日志里有 `eye_in_hand camera: ... -> /eye_in_hand/image_raw/compressed`，否则重建新版节点或显式传 `eye_in_hand_device:=/dev/videoN` |
| `/joint_states` 抖动 | 不要同时启动 `joint_state_publisher` 和 `echo_joint_states` |

## 10. 一键命令速查

```bash
cd ~/ros2 && source /opt/ros/foxy/setup.bash && colcon build --symlink-install && source install/setup.bash
ros2 launch wheeltec_sim2real_bridge bringup.launch.py
ros2 launch wheeltec_sim2real_bridge bringup.launch.py mock:=true
```
