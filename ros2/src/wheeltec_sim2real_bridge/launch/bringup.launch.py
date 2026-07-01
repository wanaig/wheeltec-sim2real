"""sim2real bringup for Ubuntu 20.04 + ROS2 Foxy.

组合:
    1. mini_4wd_six_arm display -> robot_state_publisher (URDF -> TF)
    2. serial_bridge (real robot serial) or mock_joint_states (no hardware)
    3. rosbridge_websocket -> browser roslibjs frontend

参数:
    mock              (bool, 默认 False) True=无硬件 mock 节点; False=真机串口桥接
    rosbridge         (bool, 默认 True)  启动 rosbridge_websocket
    cameras           (bool, 默认 True)  启动双 USB 摄像头桥接
    port              (int,  默认 9090)  rosbridge 端口
    jsp               (bool, 默认 False) 启动 joint_state_publisher
    use_rviz          (bool, 默认 False) 启动 rviz2
    echo_joint_states (bool, 默认 True) serial_bridge 把 voice_joint_states 回显到 /joint_states
    params_file       (str)  serial_bridge 参数文件

示例:
    # 真机
    ros2 launch wheeltec_sim2real_bridge bringup.launch.py
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
    cameras_arg = DeclareLaunchArgument('cameras', default_value='true')
    eye_in_hand_device_arg = DeclareLaunchArgument(
        'eye_in_hand_device', default_value='/dev/HandCam')
    eye_to_hand_device_arg = DeclareLaunchArgument(
        'eye_to_hand_device', default_value='/dev/RgbCam')
    camera_width_arg = DeclareLaunchArgument('camera_width', default_value='640')
    camera_height_arg = DeclareLaunchArgument('camera_height', default_value='480')
    camera_fps_arg = DeclareLaunchArgument('camera_fps', default_value='15.0')
    port_arg = DeclareLaunchArgument('port', default_value='9090')
    jsp_arg = DeclareLaunchArgument('jsp', default_value='false')
    rviz_arg = DeclareLaunchArgument('use_rviz', default_value='false')
    echo_arg = DeclareLaunchArgument('echo_joint_states', default_value='true')
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

    # 4. 双摄像头: 手眼 + 外部眼, 发布 CompressedImage 供前端订阅
    camera_node = Node(
        package='wheeltec_sim2real_bridge',
        executable='dual_camera_bridge',
        name='dual_camera_bridge',
        output='screen',
        parameters=[{
            'eye_in_hand_device': LaunchConfiguration('eye_in_hand_device'),
            'eye_to_hand_device': LaunchConfiguration('eye_to_hand_device'),
            'image_width': LaunchConfiguration('camera_width'),
            'image_height': LaunchConfiguration('camera_height'),
            'fps': LaunchConfiguration('camera_fps'),
        }],
        condition=IfCondition(LaunchConfiguration('cameras')),
    )

    return LaunchDescription([
        mock_arg, rosbridge_arg, cameras_arg,
        eye_in_hand_device_arg, eye_to_hand_device_arg,
        camera_width_arg, camera_height_arg, camera_fps_arg,
        port_arg, jsp_arg, rviz_arg, echo_arg, params_arg,
        arm_display, serial_node, mock_node, rosbridge_node, camera_node,
    ])
