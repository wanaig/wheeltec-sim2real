"""独立启动 rosbridge_server WebSocket (供浏览器 roslibjs 连接)。

等价于官方 `ros2 launch rosbridge_server rosbridge_websocket.launch.xml`,
但直接启动节点, 避免不同发行版 launch 文件名差异。

参数:
    port    (int,  默认 9090)  WebSocket 端口
    address (str,  默认 '')    监听地址 (空=所有接口)

用法:
    ros2 launch wheeltec_sim2real_bridge rosbridge_websocket.launch.py port:=9090
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port_arg = DeclareLaunchArgument(
        'port', default_value='9090', description='WebSocket 端口')
    addr_arg = DeclareLaunchArgument(
        'address', default_value='', description='监听地址 (空=所有接口)')

    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'address': LaunchConfiguration('address'),
        }],
    )
    return LaunchDescription([port_arg, addr_arg, rosbridge_node])
