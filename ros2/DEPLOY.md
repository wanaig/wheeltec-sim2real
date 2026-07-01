# 部署到 Ubuntu 20.04 小车

你的机器人系统是 Ubuntu 20.04.6 LTS focal，所以 ROS2 使用 Foxy。

## 1. 小车安装依赖

```bash
sudo apt update
sudo apt install -y ros-foxy-rosbridge-suite \
  ros-foxy-robot-state-publisher ros-foxy-joint-state-publisher \
  python3-colcon-common-extensions python3-serial
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

## 4. 构建

```bash
cd ~/ros2
source /opt/ros/foxy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 5. 启动真机 sim2real

```bash
cd ~/ros2
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch wheeltec_sim2real_bridge bringup.launch.py
```

启动成功应看到类似信息：

```text
serial opened: /dev/wheeltec_controller @ 115200
wheeltec ROS2 Foxy sim2real bridge started
```

## 6. Windows 前端连接

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

## 7. 验证

小车端看话题：

```bash
ros2 topic list
ros2 topic echo /odom
ros2 topic echo /joint_states
ros2 topic echo /PowerVoltage
ros2 topic info /voice_joint_states
ros2 topic info /cmd_vel
```

前端控制机械臂时，`/voice_joint_states` 应有发布者，`wheeltec_arm_six` 应是订阅者。

前端控制底盘时，`/cmd_vel` 应有发布者，`wheeltec_arm_six` 应是订阅者。

## 8. 无硬件联调

```bash
cd ~/ros2
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch wheeltec_sim2real_bridge bringup.launch.py mock:=true
```

这个模式不打开串口，用 ROS2 内部 mock 回显 `/joint_states` 和 `/odom`，用于验证前端、rosbridge、话题类型和 WebSocket。

## 9. 故障排查

| 现象 | 处理 |
|---|---|
| `ros2: command not found` | 没 source：`source /opt/ros/foxy/setup.bash` |
| 找不到包 | 构建后没 source：`source ~/ros2/install/setup.bash` |
| rosbridge 安装失败 | Ubuntu 20.04 应安装 `ros-foxy-rosbridge-suite` |
| 串口打不开 | 检查 `/dev/wheeltec_controller`、权限、是否被旧节点占用 |
| 前端连不上 | 检查小车 IP、9090 端口、防火墙、rosbridge 是否启动 |
| 机械臂不动 | 确认勾选“将仿真指令发送至真机”，看 `/voice_joint_states` |
| 底盘不动 | 看 `/cmd_vel`，确认串口桥接节点没有报错 |
| `/joint_states` 抖动 | 不要同时启动 `joint_state_publisher` 和 `echo_joint_states` |

## 10. 一键命令速查

```bash
cd ~/ros2 && source /opt/ros/foxy/setup.bash && colcon build --symlink-install && source install/setup.bash
ros2 launch wheeltec_sim2real_bridge bringup.launch.py
ros2 launch wheeltec_sim2real_bridge bringup.launch.py mock:=true
```
