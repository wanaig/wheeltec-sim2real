# WHEELTEC mini_4wd_six_arm · ROS2 工作空间 (sim2real 真机端)

自建的、自洽的 ROS2 系统上位机,参考 `D:\a` 的 ROS1 Melodic 源码 + STM32 固件从零移植。
不依赖现成的 `turn_on_wheeltec_robot` —— 串口桥接、URDF/TF、rosbridge 全部由本工作空间提供,
配 [`../wheeltec-sim2real`](../wheeltec-sim2real) 浏览器前端即可跑通完整 sim2real 双向通信。

## 包组成

```
ros2/src/
├── mini_4wd_six_arm/              # URDF 模型包 (移植自 ROS1 mini_4wd_six_arm)
│   ├── urdf/mini_4wd_six_arm.urdf # SolidWorks 导出, 16 link / 15 joint
│   ├── meshes/*.STL               # 16 个真实网格
│   └── launch/display.launch.py   # robot_state_publisher (URDF→TF) [+ jsp + rviz2]
└── wheeltec_sim2real_bridge/      # 串口桥接包 (移植自 ROS1 wheeltec_arm_six.cpp)
    ├── wheeltec_sim2real_bridge/
    │   ├── serial_bridge.py       # 真机串口桥接: cmd_vel/voice_joint_states→STM32, 串口→odom/imu
    │   └── mock_joint_states.py   # 无硬件 mock (回显 /joint_states + odom + 测试相机)
    ├── launch/
    │   ├── bringup.launch.py      # ★ 总启动: URDF/TF + 串口/mock + rosbridge 一条命令
    │   ├── sim2real_bridge.launch.py
    │   └── rosbridge_websocket.launch.py
    └── config/params.yaml
```

## 串口协议 (与 STM32 下位机逐字节一致, 已单测验证)

| 方向 | 帧 | 格式 |
|------|----|----|
| 下行 底盘 11B | `0x7B` 产品 使能 `vx_hi vx_lo vy_hi vy_lo wz_hi wz_lo` XOR `0x7D` (速度 ×1000 有符号定点) |
| 下行 机械臂 16B | `0xAA` `(j1..j6 各 hi,lo)` 模式 XOR `0xBB` (角度 ×1000) |
| 上行 24B | `0x7B` 停止位 `(vx,vy,wz 各 hi,lo)` `(ax,ay,az,gx,gy,gz 各 hi,lo)` 电压 `hi,lo` XOR `0x7D` |

> 上行帧**不含机械臂关节角**(臂为开环舵机, 无编码器反馈, 已在 STM32 `usartx.c` 确认)。
> 故 `/joint_states` 由 `mock_joint_states`(无硬件)、`serial_bridge` 的 `echo_joint_states`(真机无 MoveIt2)、
> 或 MoveIt2 `joint_state_publisher`(真机有 MoveIt2)提供。

## 构建与运行

```bash
# 依赖
sudo apt install ros-humble-rosbridge-server ros-humble-robot-state-publisher \
                 ros-humble-joint-state-publisher python3-serial
# 可选 (mock 相机压缩流): pip install Pillow

# 构建
cd ros2
colcon build --symlink-install
source install/setup.bash

# 赋予串口权限 (真机)
sudo chmod 666 /dev/wheeltec_controller

# ★ 一条命令总启动 (真机, 无 MoveIt2, 回显关节)
ros2 launch wheeltec_sim2real_bridge bringup.launch.py echo_joint_states:=true

# 无硬件全链路测试 (mock + 测试相机彩条)
ros2 launch wheeltec_sim2real_bridge bringup.launch.py mock:=true

# 仅模型可视化
ros2 launch mini_4wd_six_arm display.launch.py use_rviz:=true
```

然后浏览器打开前端 (同级 `../wheeltec-sim2real`, `npm run dev`),选 ROS2 → 填 `ws://小车IP:9090` → 连接 → 勾选两个同步开关。

## 部署模式对照

| 场景 | 启动命令 | `/joint_states` 来源 |
|------|---------|--------------------|
| 真机, 自建系统 (本工作空间) | `bringup.launch.py echo_joint_states:=true` | serial_bridge 回显 |
| 真机, 已有 MoveIt2 | `bringup.launch.py` (不 echo) | MoveIt2 joint_state_publisher |
| 真机, 已有 turn_on_wheeltec_robot | 不跑 serial_bridge, 仅 `ros2 launch rosbridge_server ...` | turn_on_wheeltec_robot |
| 无硬件 | `bringup.launch.py mock:=true` | mock_joint_states |

> 一条 `/dev/wheeltec_controller` 串口同时承载底盘+机械臂帧, **同一时刻只能一个节点打开**。
> 若真机已跑 `turn_on_wheeltec_robot`,不要再启动 `serial_bridge`(会争串口)。
