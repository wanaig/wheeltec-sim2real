"""mini_4wd_six_arm 模型启动 (ROS2)

发布 robot_description (URDF) + robot_state_publisher (TF 坐标树) + joint_state_publisher
(默认关节角, 真机/仿真通过 /joint_states 覆盖)。可选 rviz2 可视化。

参数:
    use_rviz (bool, 默认 False) 是否启动 rviz2
    jsp      (bool, 默认 True)  是否启动 joint_state_publisher (无 /joint_states 发布者时用)

用法:
    ros2 launch mini_4wd_six_arm display.launch.py
    ros2 launch mini_4wd_six_arm display.launch.py use_rviz:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('mini_4wd_six_arm')
    urdf = os.path.join(pkg_share, 'urdf', 'mini_4wd_six_arm.urdf')

    with open(urdf, 'r', encoding='utf-8') as f:
        robot_desc = f.read()

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='false', description='启动 rviz2')
    jsp_arg = DeclareLaunchArgument(
        'jsp', default_value='true',
        description='启动 joint_state_publisher (无外部 /joint_states 时用)')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}],
    )

    # joint_state_publisher: 读取 URDF 所有可动关节, 发布默认 0 (供 robot_state_publisher 推 TF)。
    # 真机/仿真另发 /joint_states 时会被覆盖 (QoS 默认 reliable, 后启动者共存; 若冲突可关 jsp)。
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        condition=IfCondition(LaunchConfiguration('jsp')),
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription([
        use_rviz_arg, jsp_arg,
        robot_state_publisher, joint_state_publisher, rviz2,
    ])
