"""sim2real bridge launch for Ubuntu 20.04 + ROS2 Foxy.

启动串口桥接节点 (真机) 或 mock 节点 (无硬件测试), 可选同时启动 rosbridge_server。

参数:
    mock      (bool, 默认 False)  True=运行 mock_joint_states (无硬件测试)
                                  False=运行 serial_bridge (真机串口)
    rosbridge (bool, 默认 True)   是否同时启动 rosbridge_websocket (供浏览器连接)
    port      (int,  默认 9090)    rosbridge WebSocket 端口
    params_file (str)             serial_bridge 参数文件 (默认 config/params.yaml)

示例:
    # 真机: 串口桥接 + rosbridge
    ros2 launch wheeltec_sim2real_bridge sim2real_bridge.launch.py

    # 无硬件闭环测试: mock + rosbridge
    ros2 launch wheeltec_sim2real_bridge sim2real_bridge.launch.py mock:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('wheeltec_sim2real_bridge')
    default_params = os.path.join(pkg_share, 'config', 'params.yaml')

    mock_arg = DeclareLaunchArgument(
        'mock', default_value='false',
        description='True=无硬件 mock 节点, False=真机串口桥接')
    rosbridge_arg = DeclareLaunchArgument(
        'rosbridge', default_value='true',
        description='是否启动 rosbridge_websocket')
    port_arg = DeclareLaunchArgument(
        'port', default_value='9090',
        description='rosbridge WebSocket 端口')
    params_arg = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='serial_bridge 参数文件')

    # 真机串口桥接节点 (mock:=false 时)
    serial_node = Node(
        package='wheeltec_sim2real_bridge',
        executable='serial_bridge',
        name='wheeltec_arm_six',
        output='screen',
        respawn=True,
        parameters=[LaunchConfiguration('params_file')],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('mock'), "' == 'false'"])),
    )

    # mock 节点 (mock:=true 时)
    mock_node = Node(
        package='wheeltec_sim2real_bridge',
        executable='mock_joint_states',
        name='mock_wheeltec_robot',
        output='screen',
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('mock'), "' == 'true'"])),
    )

    # rosbridge_server (供浏览器 roslibjs 连接)
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
        mock_arg, rosbridge_arg, port_arg, params_arg,
        serial_node, mock_node, rosbridge_node,
    ])
