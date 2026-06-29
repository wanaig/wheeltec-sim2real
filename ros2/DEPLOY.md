# WHEELTEC sim2real — ROS2 工作空间部署到小车使用流程

本文档说明如何把自建 ROS2 工作空间(`ros2/`)从 Windows 传输到 WHEELTEC 机械臂小车,在小车上构建运行,与 Windows 端前端虚拟世界建立 sim2real 双向通信。

## 架构

```
Windows (前端 Three.js 虚拟世界)  ──WebSocket(9090)──►  小车 rosbridge
                                                              │
                                                              ▼
                                                    serial_bridge (串口)
                                                              │
                                                              ▼
                                                         STM32 下位机
```

- **指令(仿真→真机)**:前端 → `voice_joint_states`/`cmd_vel` → serial_bridge → 串口帧 → STM32
- **状态(真机→仿真)**:STM32 → 串口 → serial_bridge → `/odom`/`/imu`/`/joint_states` → 前端镜像

---

## 0. 文件清单

- `ros2_workspace.tar.gz` — 打包好的工作空间(约 1.6MB),位于 `wheeltec-sim2real/` 根目录(已生成)
- 内含两个 ROS2 包:
  - `mini_4wd_six_arm` — URDF + 16 个 STL + robot_state_publisher(TF 坐标树)
  - `wheeltec_sim2real_bridge` — serial_bridge(串口桥接)+ mock_joint_states(无硬件测试)+ bringup 总启动

---

## 1. 前置条件

| 项 | 要求 |
|---|---|
| 小车系统 | Ubuntu 22.04 + ROS2 Humble |
| 网络 | Windows 与小车同一局域网,可 ping 通 |
| SSH | 小车已开 sshd(默认 22 端口) |
| 串口 | STM32 已连接,`/dev/wheeltec_controller` 存在 |
| 前端 | Windows 端 `wheeltec-sim2real/` 前端可 `npm run dev` |

---

## 2. 确认小车 IP 与 SSH 账号

在小车终端执行(接显示器或已有 SSH 会话):
```bash
ip addr        # 记下 wlan0/eth0 的 IP, 下文记作 10.92.74.250
whoami         # 用户名, 下文记作 wheeltec (WHEELTEC 默认 wheeltec)
```

Windows 端测试连通(PowerShell):
```powershell
ping 10.92.74.250
ssh wheeltec@10.92.74.250      # 首次输入 yes 接受密钥, 再输密码登录
```

> 下文所有命令中的 `10.92.74.250` 和 `wheeltec` 请按实际替换。

---

## 3. 传输工作空间到小车

### 方式 A:scp 传打包文件(推荐)

已生成 `ros2_workspace.tar.gz`(在 `wheeltec-sim2real/` 目录)。Windows PowerShell 执行:
```powershell
cd "...\wheeltec-sim2real"
scp ros2_workspace.tar.gz wheeltec@10.92.74.250:~/
```

### 方式 B:如需重新打包
```powershell
cd "...\wheeltec-sim2real"
tar -czf ros2_workspace.tar.gz ros2
```

### 方式 C:rsync 增量同步(反复修改传输用)
```powershell
rsync -avz --exclude build --exclude install --exclude log ros2/ wheeltec@10.92.74.250:~/ros2/
```

---

## 4. 小车端解包

SSH 登录小车后:
```bash
cd ~
tar -xzf ros2_workspace.tar.gz
ls ros2/src    # 应看到 mini_4wd_six_arm 和 wheeltec_sim2real_bridge 两个包
```

---

## 5. 安装运行时依赖

```bash
sudo apt update
# rosbridge (浏览器 WebSocket 接入)
sudo apt install -y ros-humble-rosbridge-suite
# colcon 构建工具 (若已装可跳过)
sudo apt install -y python3-colcon-common-extensions
# pyserial (serial_bridge 串口通信)
pip3 install pyserial
# 或: sudo apt install python3-serial
```

---

## 6. 串口权限

```bash
# 确认设备存在
ls -l /dev/wheeltec_controller

# 临时授权(重启失效)
sudo chmod 666 /dev/wheeltec_controller

# 永久授权(udev 规则, 一次即可)
echo 'KERNEL=="wheeltec_controller", MODE="0666"' | \
  sudo tee /etc/udev/rules.d/99-wheeltec-serial.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

> 若设备名不同(如 `/dev/ttyUSB0`),建软链或改 `ros2/src/wheeltec_sim2real_bridge/config/params.yaml` 的 `usart_port_name`。

---

## 7. 构建工作空间

```bash
cd ~/ros2
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

构建成功后生成 `install/`(含 `serial_bridge`/`mock_joint_states` 可执行 + URDF/meshes/launch)。

> 缺依赖时可用:`sudo rosdep install --from-paths src --ignore-src -r -y`

---

## 8. 启动(真机 sim2real)

```bash
cd ~/ros2
source install/setup.bash
ros2 launch wheeltec_sim2real_bridge bringup.launch.py echo_joint_states:=true
```

一条命令拉起:robot_state_publisher(URDF→TF)+ serial_bridge(串口)+ rosbridge(9090)。

**常用参数:**

| 参数 | 默认 | 说明 |
|---|---|---|
| `echo_joint_states` | false | true=回显臂关节到 /joint_states(真机无 MoveIt2 时开启) |
| `port` | 9090 | rosbridge WebSocket 端口 |
| `use_rviz` | false | 启动 rviz2 |
| `mock` | false | true=无硬件 mock(见第 11 节) |
| `rosbridge` | true | false=不启 rosbridge(已有则关) |

**启动成功标志:** 终端打印 `串口已打开: /dev/wheeltec_controller @ 115200` 和 `wheeltec_arm_six (sim2real bridge) 已启动`。

---

## 9. 前端连接(Windows 浏览器)

1. 启动前端开发服务器:
   ```powershell
   cd "...\wheeltec-sim2real"
   npm install      # 首次
   npm run dev
   ```
2. 浏览器打开 `http://localhost:5173`
3. 顶栏:ROS 版本选 **ROS2**,地址填 `ws://10.92.74.250:9090`,点「连接」
4. 状态变绿后勾选 **「将仿真指令发送至真机」**(默认关,安全考虑)
5. (可选)勾选「实时同步真机状态」→ 虚拟世界镜像真机 `/odom` + `/joint_states`

---

## 10. 操作与验证

### 键盘控制

| 键 | 功能 |
|---|---|
| 方向键 ↑↓←→ | 底盘前进/后退/左转/右转 |
| 1 / Q | joint1 底座偏航(减/增) |
| 2 / W | joint2 肩部俯仰(增/减) |
| 3 / E | joint3 肘部(减/增) |
| 4 / R | joint4 腕部俯仰(减/增) |
| 5 / T | joint5 腕部滚转(减/增) |
| 6 / Y | 夹爪(开/闭) |
| 0 | 归零位 |

### 小车端验证话题

```bash
ros2 topic echo /odom                 # 底盘里程计 (移动底盘应有变化)
ros2 topic echo /joint_states         # 臂关节 (操作臂应有变化)
ros2 topic echo /PowerVoltage         # 电池电压
ros2 topic echo /imu                  # IMU
ros2 topic info /voice_joint_states   # 确认有发布者(前端)+ 订阅者(serial_bridge)
ros2 topic info /cmd_vel
ros2 run tf2_tools view_frames        # 生成 TF 树 PDF (odom→base_link→各link)
```

---

## 11. 无硬件全链路测试(mock)

不需真机串口,验证前端↔ROS2↔仿真闭环:
```bash
cd ~/ros2 && source install/setup.bash
ros2 launch wheeltec_sim2real_bridge bringup.launch.py mock:=true
```
mock 节点会:回显 `/joint_states` + 差速积分 `/odom` + 发布测试彩条相机图像。前端连接后操作,虚拟世界模型应实时响应。

---

## 12. 故障排查

| 现象 | 排查 |
|---|---|
| scp/ssh 连不上 | `ping` 确认网络;小车 `sudo systemctl status ssh`;防火墙 |
| 串口打不开 | `ls -l /dev/wheeltec_controller`;权限(第 6 节);是否被其他节点占用(关 `turn_on_wheeltec_robot`) |
| colcon build 失败 | `source /opt/ros/humble/setup.bash`;缺依赖 `rosdep install --from-paths src --ignore-src -r -y` |
| 前端连不上 rosbridge | 小车 `ros2 topic list` 确认 rosbridge 跑起;端口 `ss -tlnp \| grep 9090`;IP/端口填对 |
| 操作臂但真机不动 | 确认勾选「将仿真指令发送至真机」;`ros2 topic echo /voice_joint_states` 看有无数据 |
| 底盘/臂方向反 | 前端已修正(2/W、6/Y 互换);若仍反检查 cmd_vel/joint 符号 |
| odom 不动 | 串口上行帧;STM32 是否发数据;`ros2 topic hz /odom` 应 ~20Hz |
| 串口节点崩溃 respawn | 看 output 错误;pyserial 是否装;波特率 115200 |

---

## 13. 常用命令速查

```bash
# 构建
cd ~/ros2 && source /opt/ros/humble/setup.bash && colcon build --symlink-install && source install/setup.bash

# 真机启动
ros2 launch wheeltec_sim2real_bridge bringup.launch.py echo_joint_states:=true

# 仅 rosbridge(已有底盘节点时)
ros2 launch wheeltec_sim2real_bridge rosbridge_websocket.launch.py port:=9090

# 查看 TF 树
ros2 run tf2_tools view_frames

# 停止所有节点
pkill -f serial_bridge; pkill -f rosbridge; pkill -f robot_state_publisher
```
