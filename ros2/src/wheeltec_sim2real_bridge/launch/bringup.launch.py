"""sim2real 总启动 (ROS2) — 一条命令拉起完整数字孪生系统。

组合:
    1. mini_4wd_six_arm display  → robot_state_publisher (URDF→TF 坐标树) [+ joint_state_publisher]
    2. serial_bridge (真机串口) 或 mock_joint_states (无硬件)
    3. rosbridge_websocket       → 供浏览器 roslibjs 前端连接

参数:
    mock              (bool, 默认 False) True=无硬件 mock 节点; False=真机串口桥接
    rosbridge         (bool, 默认 True)  启动 rosbridge_websocket
    port              (int,  默认 9090)  rosbridge 端口
    jsp               (bool, 默认 False) 启动 joint_state_publisher (无 /joint_states 源时用)
    use_rviz          (bool, 默认 False) 启动 rviz2
    echo_joint_states (bool, 默认 False) serial_bridge 把 voice_joint_states 回显到 /joint_states
                                        (真机无 MoveIt2 时开启, 让仿真镜像臂关节)
    params_file       (str)  serial_bridge 参数文件

示例:
    # 真机 (从 ROS1 迁移, 无 turn_on_wheeltec_robot)
    ros2 launch wheeltec_sim2real_bridge bringup.launch.py echo_joint_states:=true
    # 无硬件全链路测试 (含相机彩条)
    ros2 launch wheeltec_sim2real_bridge bringup.launch.py mock:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration, PythonExpression, PathJoinSubstitution)
from launch_ros.actions import Node


def generate_launch_description():
    bridge_share = get_package_share_directory('wheeltec_sim2real_bridge')
    arm_share = get_package_share_directory('mini_4wd_six_arm')
    default_params = os.path.join(bridge_share, 'config', 'params.yaml')

    mock_arg = DeclareLaunchArgument('mock', default_value='false')
    rosbridge_arg = DeclareLaunchArgument('rosbridge', default_value='true')
    port_arg = DeclareLaunchArgument('port', default_value='9090')
    jsp_arg = DeclareLaunchArgument('jsp', default_value='false')
    rviz_arg = DeclareLaunchArgument('use_rviz', default_value='false')
    echo_arg = DeclareLaunchArgument('echo_joint_states', default_value='false')
    params_arg = DeclareLaunchArgument(
        'params_file', default_value=default_params)

    # 1. URDF 模型 + robot_state_publisher (TF)
    arm_display = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(arm_share, 'launch', 'display.launch.py')),
        launch_arguments={
            'jsp': LaunchConfiguration('jsp'),
            'use_rviz': LaunchConfiguration('use_rviz'),
        }.items(),
    )

    # 2a. 真机串口桥接 (mock:=false)
    serial_node = Node(
        package='wheeltec_sim2real_bridge',
        executable='serial_bridge',
        name='wheeltec_arm_six',
        output='screen',
        respawn=True,
        parameters=[
            LaunchConfiguration('params_file'),
            {'echo_joint_states': LaunchConfiguration('echo_joint_states')},
        ],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('mock'), "' == 'false'"])),
    )

    # 2b. mock 节点 (mock:=true)
    mock_node = Node(
        package='wheeltec_sim2real_bridge',
        executable='mock_joint_states',
        name='mock_wheeltec_robot',
        output='screen',
        parameters=[{'mock_camera': True}],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('mock'), "' == 'true'"])),
    )

    # 3. rosbridge_websocket (供浏览器前端)
    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'address': '',
        }],
        condition=IfCondition(LaunchConfiguration('rosbridge')),
    )

    return LaunchDescription([
        mock_arg, rosbridge_arg, port_arg, jsp_arg, rviz_arg, echo_arg, params_arg,
        arm_display, serial_node, mock_node, rosbridge_node,
    ])
