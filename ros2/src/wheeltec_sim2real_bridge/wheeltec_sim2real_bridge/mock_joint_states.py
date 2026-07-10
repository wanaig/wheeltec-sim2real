"""
mock_joint_states.py — 无真机时的 sim2real 闭环测试节点 (ROS2)

当没有下位机 (STM32) 时, 用本节点代替 serial_bridge, 在 ROS2 内部模拟真机:
    订阅 voice_joint_states → 直接作为 /joint_states 回发 (仿真镜像真机指令)
    订阅 cmd_vel            → 差速积分 → 发布 /odom (仿真镜像底盘运动)

这样浏览器数字孪生 + rosbridge + 本节点即可跑通完整 sim2real 双向通信,
无需任何硬件。配合 serial_bridge 的真机切换只需替换运行节点。

用法:
    ros2 run wheeltec_sim2real_bridge mock_joint_states
或:
    ros2 launch wheeltec_sim2real_bridge sim2real_bridge.launch.py mock:=true
"""
import math
import sys as _sys
for _p in list(_sys.path):
    if 'noetic' in _p or 'ros1' in _p:
        _sys.path.remove(_p)

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState, Image
from nav_msgs.msg import Odometry

try:
    from sensor_msgs.msg import CompressedImage
    _HAS_COMPRESSED = True
except ImportError:
    _HAS_COMPRESSED = False

try:
    from PIL import Image as PILImage
    import io
    _HAS_PIL = True
except ImportError:
    _HAS_PIL = False


def _yaw_to_quaternion(yaw):
    h = yaw * 0.5
    return (0.0, 0.0, math.sin(h), math.cos(h))


class MockRobotNode(Node):
    def __init__(self):
        super().__init__('mock_wheeltec_robot')

        self.declare_parameter('robot_frame_id', 'base_link')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('mock_camera', True)   # 发布测试画面便于验证相机面板
        self.robot_frame_id = self.get_parameter('robot_frame_id').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.mock_camera = self.get_parameter('mock_camera').value

        # 仿真状态
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # /joint_states 发布 (真机→仿真)
        self.js_pub = self.create_publisher(JointState, '/joint_states', 10)
        # /odom 发布 (真机→仿真)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 50)

        # 相机测试画面发布 (便于在无硬件时验证前端相机面板)
        if self.mock_camera:
            self.img_pub = self.create_publisher(Image, '/usb_cam/image_raw', 10)
            if _HAS_COMPRESSED and _HAS_PIL:
                self.cmp_pub = self.create_publisher(
                    CompressedImage, '/image_raw/compressed', 10)
                self.cmp_pub2 = self.create_publisher(
                    CompressedImage, '/usb_cam/image_raw/compressed', 10)
                self.cmp_pub_eye_in_hand = self.create_publisher(
                    CompressedImage, '/eye_in_hand/image_raw/compressed', 10)
                self.cmp_pub_eye_to_hand = self.create_publisher(
                    CompressedImage, '/eye_to_hand/image_raw/compressed', 10)
            self._cam_t = 0.0
            self.create_timer(0.1, self._pub_test_image)  # 10Hz
            if _HAS_COMPRESSED and _HAS_PIL:
                self.get_logger().info(
                    'mock 相机: 发布 /eye_in_hand/image_raw/compressed 和 /eye_to_hand/image_raw/compressed')
            else:
                self.get_logger().info(
                    'mock 相机: 发布 /usb_cam/image_raw (raw rgb8). '
                    '装 Pillow 可改为 CompressedImage: pip install Pillow')

        # 订阅仿真→真机指令
        self.create_subscription(
            JointState, 'voice_joint_states', self.voice_cb, 100)
        self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_cb, 100)

        self._last_js = None
        self._last_time = self.get_clock().now()
        # 20Hz 推进底盘积分
        self.timer = self.create_timer(0.05, self.tick)
        self.get_logger().info(
            'mock_wheeltec_robot 已启动 (无硬件 sim2real 测试模式)')

    def voice_cb(self, msg):
        # 真机 "执行" 后将关节状态回传给仿真 (镜像)
        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = msg.name
        out.position = msg.position
        self.js_pub.publish(out)
        self._last_js = msg

    def cmd_vel_cb(self, twist):
        self._cmd = (twist.linear.x, twist.linear.y, twist.angular.z)

    def tick(self):
        now = self.get_clock().now()
        dt = (now - self._last_time).nanoseconds * 1e-9
        self._last_time = now
        if dt <= 0.0 or dt > 1.0:
            dt = 0.05
        cmd = getattr(self, '_cmd', (0.0, 0.0, 0.0))
        vx, vy, wz = cmd
        self.x += (vx * math.cos(self.yaw) - vy * math.sin(self.yaw)) * dt
        self.y += (vx * math.sin(self.yaw) + vy * math.cos(self.yaw)) * dt
        self.yaw += wz * dt

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.robot_frame_id
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        qx, qy, qz, qw = _yaw_to_quaternion(self.yaw)
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = wz
        self.odom_pub.publish(odom)

    def _pub_test_image(self):
        """发布一个移动的彩条测试图 (320x240), 验证前端相机面板。
        有 Pillow 时发 CompressedImage(jpeg), 否则发 raw rgb8 Image。"""
        W, H = 320, 240
        self._cam_t += 0.1
        phase = self._cam_t
        # 生成 rgb8 字节: 水平彩条 + 随时间移动的亮带
        buf = bytearray(W * H * 3)
        for y in range(H):
            for x in range(W):
                i = (y * W + x) * 3
                buf[i] = (x * 2 + int(phase * 20)) & 0xFF          # R
                buf[i + 1] = (y + int(phase * 10)) & 0xFF          # G
                buf[i + 2] = (int(phase * 30) & 0xFF)              # B
        if _HAS_COMPRESSED and _HAS_PIL:
            img = PILImage.frombytes('RGB', (W, H), bytes(buf))
            with io.BytesIO() as out:
                img.save(out, format='JPEG', quality=60)
                jpg = out.getvalue()
            for pub in (getattr(self, 'cmp_pub', None),
                        getattr(self, 'cmp_pub2', None),
                        getattr(self, 'cmp_pub_eye_in_hand', None),
                        getattr(self, 'cmp_pub_eye_to_hand', None)):
                if pub is None:
                    continue
                msg = CompressedImage()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.format = 'jpeg'
                msg.data = jpg
                pub.publish(msg)
        else:
            msg = Image()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'usb_cam'
            msg.height = H
            msg.width = W
            msg.encoding = 'rgb8'
            msg.step = W * 3
            msg.data = bytes(buf)
            self.img_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockRobotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
