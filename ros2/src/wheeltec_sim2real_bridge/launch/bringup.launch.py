"""sim2real bringup for Ubuntu 20.04 + ROS2 Foxy.

组合:
    1. mini_4wd_six_arm display -> robot_state_publisher (URDF -> TF)
    2. serial_bridge (real robot serial) or mock_joint_states (no hardware)
    3. rosbridge_websocket -> browser roslibjs frontend
    4. (可选) Astra S 手眼相机: ROS1 astra_camera + ros1_bridge -> ROS2

参数:
    mock              (bool, 默认 False) True=无硬件 mock 节点; False=真机串口桥接
    rosbridge         (bool, 默认 True)  启动 rosbridge_websocket
    cameras           (bool, 默认 True)  启动双 USB 摄像头桥接 (外部相机 /eye_to_hand)
    rgbd_detector     (bool, 默认 False) 启动 RGB-D 工业工具检测/标注节点
    astra_hand        (bool, 默认 False) 启动 Astra S 手眼相机链路 (ROS1 驱动+ros1_bridge,
                                          发布 /camera/color/image_raw/compressed 到 ROS2。
                                          需先 sudo apt install ros-foxy-ros1-bridge。
                                          Astra USB 卡死时需物理拔插, 详见 scripts/start_astra_hand_camera.sh)
    port              (int,  默认 9090)  rosbridge 端口
    jsp               (bool, 默认 False) 启动 joint_state_publisher
    use_rviz          (bool, 默认 False) 启动 rviz2
    echo_joint_states (bool, 默认 True) serial_bridge 把 voice_joint_states 回显到 /joint_states
    params_file       (str)  serial_bridge 参数文件

示例:
    # 真机 (仅外部相机)
    ros2 launch wheeltec_sim2real_bridge bringup.launch.py
    # 真机 + Astra S 手眼相机 (外部+手眼双画面)
    ros2 launch wheeltec_sim2real_bridge bringup.launch.py astra_hand:=true echo_joint_states:=true
    # 无硬件全链路测试 (含相机彩条)
    ros2 launch wheeltec_sim2real_bridge bringup.launch.py mock:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
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
    rgbd_detector_arg = DeclareLaunchArgument('rgbd_detector', default_value='false')
    eye_in_hand_device_arg = DeclareLaunchArgument(
        'eye_in_hand_device', default_value='/dev/HandCam')
    eye_to_hand_device_arg = DeclareLaunchArgument(
        'eye_to_hand_device', default_value='/dev/RgbCam')
    camera_width_arg = DeclareLaunchArgument('camera_width', default_value='640')
    camera_height_arg = DeclareLaunchArgument('camera_height', default_value='480')
    camera_fps_arg = DeclareLaunchArgument('camera_fps', default_value='15.0')
    rgbd_rgb_topic_arg = DeclareLaunchArgument(
        'rgbd_rgb_topic', default_value='/camera/rgb/image_raw')
    rgbd_depth_topic_arg = DeclareLaunchArgument(
        'rgbd_depth_topic', default_value='/camera/depth/image_raw')
    rgbd_camera_info_topic_arg = DeclareLaunchArgument(
        'rgbd_camera_info_topic', default_value='/camera/rgb/camera_info')
    port_arg = DeclareLaunchArgument('port', default_value='9090')
    jsp_arg = DeclareLaunchArgument('jsp', default_value='false')
    rviz_arg = DeclareLaunchArgument('use_rviz', default_value='false')
    echo_arg = DeclareLaunchArgument('echo_joint_states', default_value='true')
    params_arg = DeclareLaunchArgument(
        'params_file', default_value=default_params)
    # Astra S 手眼相机: ROS1 astra_camera + ros1_bridge 链路 (独立脚本 supervise 模式)
    # 详见 scripts/start_astra_hand_camera.sh。需先 sudo apt install ros-foxy-ros1-bridge
    astra_hand_arg = DeclareLaunchArgument('astra_hand', default_value='false')
    astra_hand_script = os.path.join(bridge_share, 'scripts', 'start_astra_hand_camera.sh')

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

    # 5. RGB-D 工业工具/零件检测: 输出 JSON 标注和调试图像
    rgbd_detector_node = Node(
        package='wheeltec_sim2real_bridge',
        executable='rgbd_tool_detector',
        name='rgbd_tool_detector',
        output='screen',
        parameters=[{
            'rgb_topic': LaunchConfiguration('rgbd_rgb_topic'),
            'depth_topic': LaunchConfiguration('rgbd_depth_topic'),
            'camera_info_topic': LaunchConfiguration('rgbd_camera_info_topic'),
        }],
        condition=IfCondition(LaunchConfiguration('rgbd_detector')),
    )

    # 6. Astra S 手眼相机链路 (ROS1 astra_camera + ros1_bridge -> ROS2)
    #    独立脚本以前台 supervise 模式运行: roscore + astra + republish + dynamic_bridge
    #    launch 退出 (Ctrl+C) 时脚本捕获信号清理全部子进程
    astra_hand_proc = ExecuteProcess(
        cmd=['bash', astra_hand_script, 'supervise'],
        name='astra_hand_camera',
        output='screen',
        condition=IfCondition(LaunchConfiguration('astra_hand')),
    )

    return LaunchDescription([
        mock_arg, rosbridge_arg, cameras_arg, rgbd_detector_arg,
        eye_in_hand_device_arg, eye_to_hand_device_arg,
        camera_width_arg, camera_height_arg, camera_fps_arg,
        rgbd_rgb_topic_arg, rgbd_depth_topic_arg, rgbd_camera_info_topic_arg,
        port_arg, jsp_arg, rviz_arg, echo_arg, params_arg, astra_hand_arg,
        arm_display, serial_node, mock_node, rosbridge_node, camera_node,
        rgbd_detector_node, astra_hand_proc,
    ])
