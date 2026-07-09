"""sim2real bringup for Ubuntu 20.04 + ROS2 Foxy.

组合:
    1. mini_4wd_six_arm display -> robot_state_publisher (URDF -> TF)
    2. serial_bridge (real robot serial) or mock_joint_states (no hardware)
    3. rosbridge_websocket -> browser roslibjs frontend
    4. (可选) Astra S 手眼相机: ROS1 astra_camera + ros1_bridge -> ROS2
       (含 RGB+depth 流 + camera_info, 供前端显示和 YOLO 检测器使用)
    5. (可选) YOLO + RGB-D 工业检测: rgbd_tool_detector (YOLO + 深度, 输出世界坐标标注)
    6. (可选) 手眼相机挂载 TF: link5 -> camera_link (静态, 供检测器世界坐标变换)

参数:
    mock              (bool, 默认 False) True=无硬件 mock 节点; False=真机串口桥接
    rosbridge         (bool, 默认 True)  启动 rosbridge_websocket
    cameras           (bool, 默认 True)  启动双 USB 摄像头桥接 (外部相机 /eye_to_hand)
    rgbd_detector     (bool, 默认 False) 启动 YOLO+RGB-D 工业检测/标注节点 (需 astra_hand:=true)
    astra_hand        (bool, 默认 False) 启动 Astra S 手眼相机链路 (ROS1 驱动+ros1_bridge,
                                          发布 RGB+depth+camera_info 到 ROS2。
                                          需先 sudo apt install ros-foxy-ros1-bridge。
                                          Astra USB 卡死时需物理拔插, 详见 scripts/start_astra_hand_camera.sh)
    hand_cam_x/y/z    (float, 默认 0.05/0/0.03) 手眼相机挂载位置 (link5→camera_link 静态 TF)
    hand_cam_roll/pitch/yaw (float, 默认 0/-0.3/0) 手眼相机挂载姿态 (pitch 负值=向下看桌面)
    yolo_model        (str,  默认 'yolov8n.pt') YOLO 模型文件 (COCO 预训练)
                                          需 pip install ultralytics; 未安装时自动回退到 yolov5s.pt
    yolo_conf         (float, 默认 0.45)   YOLO 置信度阈值
    yolo_device       (str,  默认 'cuda:0') YOLO 推理设备 (cuda:0 / cpu)
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
    # 真机 + Astra + YOLO RGB-D 工业检测 (完整工业感知)
    ros2 launch wheeltec_sim2real_bridge bringup.launch.py astra_hand:=true rgbd_detector:=true echo_joint_states:=true
    # 无硬件全链路测试 (含相机彩条)
    ros2 launch wheeltec_sim2real_bridge bringup.launch.py mock:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
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
        'rgbd_rgb_topic', default_value='/camera/color/image_raw')
    rgbd_depth_topic_arg = DeclareLaunchArgument(
        'rgbd_depth_topic', default_value='/camera/depth/image_raw')
    rgbd_camera_info_topic_arg = DeclareLaunchArgument(
        'rgbd_camera_info_topic', default_value='/camera/color/camera_info')
    rgbd_depth_compressed_arg = DeclareLaunchArgument(
        'rgbd_depth_compressed', default_value='true')
    rgbd_rgb_compressed_arg = DeclareLaunchArgument(
        'rgbd_rgb_compressed', default_value='true')
    # YOLO 参数
    yolo_model_arg = DeclareLaunchArgument(
        'yolo_model', default_value='yolov8n.pt')
    yolo_conf_arg = DeclareLaunchArgument(
        'yolo_conf', default_value='0.45')
    yolo_device_arg = DeclareLaunchArgument(
        'yolo_device', default_value='cuda:0')
    yolo_imgsz_arg = DeclareLaunchArgument(
        'yolo_imgsz', default_value='640')
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

    # 手眼相机挂载位姿 (link5 → camera_link 的静态 TF, 用于 RGB-D 检测的世界坐标变换)
    # 默认值: 相机在 link5 前方 5cm、上方 3cm, 向下倾斜 ~17° 看桌面
    # 实际安装不同时通过 launch 参数覆盖, 或做手眼标定后更新
    hand_cam_x_arg = DeclareLaunchArgument('hand_cam_x', default_value='0.05')
    hand_cam_y_arg = DeclareLaunchArgument('hand_cam_y', default_value='0.0')
    hand_cam_z_arg = DeclareLaunchArgument('hand_cam_z', default_value='0.03')
    hand_cam_roll_arg = DeclareLaunchArgument('hand_cam_roll', default_value='0.0')
    hand_cam_pitch_arg = DeclareLaunchArgument('hand_cam_pitch', default_value='-0.3')
    hand_cam_yaw_arg = DeclareLaunchArgument('hand_cam_yaw', default_value='0.0')

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

    # 5. YOLO + RGB-D 工业检测: 输出 JSON 标注和调试图像
    #    延迟 25s 启动: 等待 Astra 链路 (roscore→astra→republish→bridge) 就绪
    #    避免检测器在 bridge publisher 创建前订阅, 导致 ROS2 订阅匹配失败
    rgbd_detector_node = TimerAction(
        period=25.0,
        actions=[Node(
            package='wheeltec_sim2real_bridge',
            executable='rgbd_tool_detector',
            name='rgbd_tool_detector',
            output='screen',
            parameters=[{
                'rgb_topic': LaunchConfiguration('rgbd_rgb_topic'),
                'depth_topic': LaunchConfiguration('rgbd_depth_topic'),
                'camera_info_topic': LaunchConfiguration('rgbd_camera_info_topic'),
                'depth_compressed': LaunchConfiguration('rgbd_depth_compressed'),
                'rgb_compressed': LaunchConfiguration('rgbd_rgb_compressed'),
                'yolo_model': LaunchConfiguration('yolo_model'),
                'yolo_conf': LaunchConfiguration('yolo_conf'),
                'yolo_device': LaunchConfiguration('yolo_device'),
                'yolo_imgsz': LaunchConfiguration('yolo_imgsz'),
            }],
            condition=IfCondition(LaunchConfiguration('rgbd_detector')),
        )],
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

    # 7. 手眼相机挂载 TF (link5 → camera_link)
    #    连接 URDF TF 树 (base_link→...→link5) 与 Astra 自身 TF (camera_link→camera_color_optical_frame)
    #    使 rgbd_tool_detector 能把相机坐标变换到 base_link 世界坐标
    #    Foxy 的 static_transform_publisher 用位置参数: x y z yaw pitch roll frame_id child_frame_id
    hand_camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='hand_camera_mount_tf',
        arguments=[
            LaunchConfiguration('hand_cam_x'),
            LaunchConfiguration('hand_cam_y'),
            LaunchConfiguration('hand_cam_z'),
            LaunchConfiguration('hand_cam_yaw'),
            LaunchConfiguration('hand_cam_pitch'),
            LaunchConfiguration('hand_cam_roll'),
            'link5',
            'camera_link',
        ],
        condition=IfCondition(LaunchConfiguration('astra_hand')),
    )

    # 7b. 光学坐标系 TF (camera_link → camera_color_optical_frame)
    #     Astra publish_tf:=false 时不发布此 TF, 需手动补
    #     旋转: yaw=-90° roll=-90° (ROS body frame → optical frame: Z前X右Y下)
    hand_camera_optical_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='hand_camera_optical_tf',
        arguments=['0', '0', '0', '-1.5708', '0', '-1.5708',
                   'camera_link', 'camera_color_optical_frame'],
        condition=IfCondition(LaunchConfiguration('astra_hand')),
    )

    return LaunchDescription([
        mock_arg, rosbridge_arg, cameras_arg, rgbd_detector_arg,
        eye_in_hand_device_arg, eye_to_hand_device_arg,
        camera_width_arg, camera_height_arg, camera_fps_arg,
        rgbd_rgb_topic_arg, rgbd_depth_topic_arg, rgbd_camera_info_topic_arg,
        rgbd_depth_compressed_arg, rgbd_rgb_compressed_arg,
        yolo_model_arg, yolo_conf_arg, yolo_device_arg, yolo_imgsz_arg,
        port_arg, jsp_arg, rviz_arg, echo_arg, params_arg, astra_hand_arg,
        hand_cam_x_arg, hand_cam_y_arg, hand_cam_z_arg,
        hand_cam_roll_arg, hand_cam_pitch_arg, hand_cam_yaw_arg,
        arm_display, serial_node, mock_node, rosbridge_node, camera_node,
        rgbd_detector_node, astra_hand_proc, hand_camera_tf,
        hand_camera_optical_tf,
    ])
