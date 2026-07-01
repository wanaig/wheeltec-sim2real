from setuptools import setup

package_name = 'wheeltec_sim2real_bridge'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/sim2real_bridge.launch.py',
            'launch/rosbridge_websocket.launch.py',
            'launch/bringup.launch.py',
        ]),
        ('share/' + package_name + '/config', ['config/params.yaml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='wheeltec',
    maintainer_email='wheeltec@todo.todo',
    description='WHEELTEC mini_4wd_six_arm sim2real serial bridge (ROS2)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_bridge = wheeltec_sim2real_bridge.serial_bridge:main',
            'mock_joint_states = wheeltec_sim2real_bridge.mock_joint_states:main',
            'dual_camera_bridge = wheeltec_sim2real_bridge.dual_camera_bridge:main',
        ],
    },
)
