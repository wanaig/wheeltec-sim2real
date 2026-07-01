# ROS2 Foxy 环境清理与启动步骤

本文用于处理小车上 ROS1 Noetic 和 ROS2 Foxy 环境混用导致的启动失败问题。

适用系统：

- Ubuntu 20.04.6 LTS
- ROS2 Foxy
- 可选安装了 ROS1 Noetic 的 WHEELTEC 小车
- 工作空间路径：`/home/wheeltec/ros2`

## 1. 典型现象

如果启动时看到下面这些报错，通常说明当前终端混入了 ROS1 Noetic 环境。

```text
ROS_DISTRO was set to 'foxy' before.
ROS_DISTRO was set to 'noetic' before.
```

```text
This might be a ROS 1 message type but it should be a ROS 2 message type.
```

```text
/opt/ros/noetic/lib/python3/dist-packages/geometry_msgs/msg/__init__.py
```

```text
robot_state_publisher: symbol lookup error: undefined symbol
```

如果使用了不完整的 `env -i bash --noprofile --norc`，还可能看到：

```text
Failed to initialize logging: Failed to get logging directory
```

这是因为 `env -i` 清掉了 `HOME`，ROS2 无法创建日志目录。

## 2. 根因说明

ROS1 Noetic 和 ROS2 Foxy 都有 `geometry_msgs`、`nav_msgs`、`sensor_msgs` 等包名。

如果同一个终端里同时存在 ROS1 和 ROS2 的 `PYTHONPATH`、`LD_LIBRARY_PATH` 或 `CMAKE_PREFIX_PATH`，ROS2 节点可能会导入 ROS1 的消息包或动态库，导致运行时崩溃。

本项目代码中使用的是 ROS2 写法，例如：

```python
from nav_msgs.msg import Odometry
```

如果运行时报 ROS1 消息类型错误，优先检查环境，不要先改代码。

## 3. 进入干净 shell

从 Windows PowerShell SSH 登录小车后，先进入一个保留必要变量的干净 shell。

不要直接使用：

```bash
env -i bash --noprofile --norc
```

请使用：

```bash
env -i HOME=/home/wheeltec USER=wheeltec SHELL=/bin/bash TERM=xterm-256color PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin bash --noprofile --norc
```

进入后提示符通常会变成：

```text
bash-5.0$
```

## 4. 清理旧构建产物

旧的 `build`、`install`、`log` 可能是在污染环境下生成的，需要删除后重新编译。

```bash
cd /home/wheeltec/ros2
rm -rf build install log
```

## 5. 只加载 ROS2 Foxy

```bash
source /opt/ros/foxy/setup.bash
```

检查当前环境：

```bash
echo $ROS_DISTRO
echo $PYTHONPATH
echo $LD_LIBRARY_PATH
echo $AMENT_PREFIX_PATH
echo $CMAKE_PREFIX_PATH
```

`ROS_DISTRO` 应该是：

```text
foxy
```

输出中不能出现：

```text
/opt/ros/noetic
catkin_ws
```

如果出现 Noetic 路径，说明当前 shell 仍然不干净，需要退出后重新执行第 3 步。

## 6. 重新编译工作空间

```bash
cd /home/wheeltec/ros2
colcon build --symlink-install
```

编译完成后加载当前工作空间：

```bash
source install/setup.bash
```

再次检查环境：

```bash
echo $ROS_DISTRO
echo $PYTHONPATH
echo $LD_LIBRARY_PATH
echo $AMENT_PREFIX_PATH
echo $CMAKE_PREFIX_PATH
```

仍然必须满足：

- `ROS_DISTRO` 是 `foxy`
- 没有 `/opt/ros/noetic`
- 没有 `catkin_ws`

## 7. 启动真机节点

正常启动：

```bash
ros2 launch wheeltec_sim2real_bridge bringup.launch.py echo_joint_states:=true
```

如果先排查主程序，不想让 rosbridge 干扰，可以先关闭 rosbridge：

```bash
ros2 launch wheeltec_sim2real_bridge bringup.launch.py echo_joint_states:=true rosbridge:=false
```

如果没有连接真实硬件，可以使用 mock 模式：

```bash
ros2 launch wheeltec_sim2real_bridge bringup.launch.py mock:=true rosbridge:=false
```

## 8. 启动成功标志

正常情况下不应再看到：

```text
/opt/ros/noetic
This might be a ROS 1 message type
Failed to initialize logging
symbol lookup error
```

真机串口正常时，应看到类似：

```text
serial opened: /dev/wheeltec_controller @ 115200
wheeltec ROS2 Foxy sim2real bridge started
```

如果提示串口打不开，继续检查：

```bash
ls -l /dev/wheeltec_controller
ls -l /dev/ttyUSB*
```

必要时临时授权：

```bash
sudo chmod 666 /dev/wheeltec_controller
```

或者把实际串口软链接到项目默认设备名：

```bash
sudo ln -sf /dev/ttyUSB0 /dev/wheeltec_controller
```

## 9. 可选：修复 `.bashrc`

如果每次打开终端都会自动混入 Noetic，检查 `~/.bashrc`：

```bash
nano ~/.bashrc
```

找到类似内容：

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
```

如果当前终端主要用于 ROS2 Foxy，可以注释掉：

```bash
# source /opt/ros/noetic/setup.bash
# source ~/catkin_ws/devel/setup.bash
```

建议不要在同一个终端同时 source ROS1 和 ROS2。需要切换时，新开终端分别加载对应环境。

## 10. 推荐日常启动命令

环境修复后，日常只需要：

```bash
cd /home/wheeltec/ros2
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch wheeltec_sim2real_bridge bringup.launch.py echo_joint_states:=true
```

如果再次出现 ROS1/ROS2 混用报错，回到第 3 步重新清理、删除构建产物并编译。
