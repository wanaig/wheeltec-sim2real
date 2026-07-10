"""
ROS2 Foxy sim2real serial bridge for WHEELTEC mini_4wd_six_arm.

Target robot:
  Ubuntu 20.04 (focal) + ROS2 Foxy + WHEELTEC R550A / mini_4wd_six_arm.

This node is a ROS2/rclpy rewrite of the ROS1 node in D:\a:
  wheeltec_arm_pick/src/wheeltec_six_arm_pick/wheeltec_arm_six.cpp

The wire protocol is intentionally kept byte-for-byte compatible with the old
STM32 firmware:
  Chassis command: 11 bytes, 0x7B ... XOR ... 0x7D
  Arm command:     16 bytes, 0xAA ... XOR ... 0xBB
  Sensor frame:    24 bytes, 0x7B ... XOR ... 0x7D

sim2real data flow:
  Browser/roslibjs -> voice_joint_states -> this node -> STM32 arm servos
  Browser/roslibjs -> cmd_vel            -> this node -> STM32 chassis
  STM32            -> this node          -> /odom /imu /PowerVoltage /pose

Important: the STM32 sensor frame does not contain arm encoder positions. When
echo_joint_states is enabled this node publishes commanded arm values as
/joint_states so the digital twin can mirror the command state. That is open
loop arm feedback, not measured encoder feedback.
"""

import math
from typing import Iterable, List, Optional, Sequence, Tuple

# 机器人上 noetic + foxy 共存, 需移除 noetic 的 Python 包路径,
# 否则 import sensor_msgs/nav_msgs 会解析为 ROS1 版本 (缺少 _TYPE_SUPPORT)
import sys as _sys
for _p in list(_sys.path):
    if 'noetic' in _p or 'ros1' in _p:
        _sys.path.remove(_p)

import rclpy
from geometry_msgs.msg import Pose, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster

try:
    import serial as pyserial
except ImportError:  # pragma: no cover - handled at runtime on robot
    pyserial = None


FRAME_HEADER = 0x7B
FRAME_TAIL = 0x7D
FRAME_HEADER_ARM = 0xAA
FRAME_TAIL_ARM = 0xBB

RECEIVE_DATA_SIZE = 24
SEND_DATA_SIZE_CHASSIS = 11
SEND_DATA_SIZE_ARM = 16

DEFAULT_MODE = 1
FOLLOWER_MODE = 2

GYROSCOPE_RATIO = 0.00026644
ACCEL_RATIO = 16384.0
LOOP_HZ = 50.0
AHRS_HZ = 20.0

ARM_JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
ALL_JOINT_NAMES = [
    'left_front_wheel_joint', 'left_rear_wheel_joint',
    'right_front_wheel_joint', 'right_rear_wheel_joint',
    'joint1', 'joint2', 'joint3', 'joint4', 'joint5',
    'joint6', 'joint7', 'joint8', 'joint9', 'joint10', 'joint11',
]

ODOM_POSE_COVARIANCE = [
    1e-3, 0, 0, 0, 0, 0,
    0, 1e-3, 0, 0, 0, 0,
    0, 0, 1e6, 0, 0, 0,
    0, 0, 0, 1e6, 0, 0,
    0, 0, 0, 0, 1e6, 0,
    0, 0, 0, 0, 0, 1e3,
]
ODOM_POSE_COVARIANCE_STOPPED = [
    1e-9, 0, 0, 0, 0, 0,
    0, 1e-3, 1e-9, 0, 0, 0,
    0, 0, 1e6, 0, 0, 0,
    0, 0, 0, 1e6, 0, 0,
    0, 0, 0, 0, 1e6, 0,
    0, 0, 0, 0, 0, 1e-9,
]
ODOM_TWIST_COVARIANCE = [
    1e-3, 0, 0, 0, 0, 0,
    0, 1e-3, 0, 0, 0, 0,
    0, 0, 1e6, 0, 0, 0,
    0, 0, 0, 1e6, 0, 0,
    0, 0, 0, 0, 1e6, 0,
    0, 0, 0, 0, 0, 1e3,
]
ODOM_TWIST_COVARIANCE_STOPPED = [
    1e-9, 0, 0, 0, 0, 0,
    0, 1e-3, 1e-9, 0, 0, 0,
    0, 0, 1e6, 0, 0, 0,
    0, 0, 0, 1e6, 0, 0,
    0, 0, 0, 0, 1e6, 0,
    0, 0, 0, 0, 0, 1e-9,
]


def _xor_checksum(data: Sequence[int], count: int) -> int:
    value = 0
    for i in range(count):
        value ^= int(data[i]) & 0xFF
    return value & 0xFF


def _pack_fixed(value: float) -> Tuple[int, int]:
    raw = int(round(float(value) * 1000.0))
    raw = max(-32768, min(32767, raw))
    return (raw >> 8) & 0xFF, raw & 0xFF


def _int16(high: int, low: int) -> int:
    raw = ((int(high) & 0xFF) << 8) | (int(low) & 0xFF)
    if raw >= 0x8000:
        raw -= 0x10000
    return raw


def _yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


class MahonyAhrs:
    """Small Mahony AHRS port matching the old quaternion_solution.cpp usage."""

    def __init__(self) -> None:
        self.two_kp = 1.0
        self.two_ki = 0.0
        self.q0 = 1.0
        self.q1 = 0.0
        self.q2 = 0.0
        self.q3 = 0.0
        self.integral_fb = [0.0, 0.0, 0.0]

    def update(self, gx: float, gy: float, gz: float,
               ax: float, ay: float, az: float, dt: float) -> None:
        q0, q1, q2, q3 = self.q0, self.q1, self.q2, self.q3

        if not (ax == 0.0 and ay == 0.0 and az == 0.0):
            norm = math.sqrt(ax * ax + ay * ay + az * az)
            if norm > 0.0:
                ax, ay, az = ax / norm, ay / norm, az / norm
                halfvx = q1 * q3 - q0 * q2
                halfvy = q0 * q1 + q2 * q3
                halfvz = q0 * q0 - 0.5 + q3 * q3
                halfex = ay * halfvz - az * halfvy
                halfey = az * halfvx - ax * halfvz
                halfez = ax * halfvy - ay * halfvx
                if self.two_ki > 0.0:
                    self.integral_fb[0] += self.two_ki * halfex * dt
                    self.integral_fb[1] += self.two_ki * halfey * dt
                    self.integral_fb[2] += self.two_ki * halfez * dt
                    gx += self.integral_fb[0]
                    gy += self.integral_fb[1]
                    gz += self.integral_fb[2]
                else:
                    self.integral_fb = [0.0, 0.0, 0.0]
                gx += self.two_kp * halfex
                gy += self.two_kp * halfey
                gz += self.two_kp * halfez

        gx *= 0.5 * dt
        gy *= 0.5 * dt
        gz *= 0.5 * dt
        qa, qb, qc = q0, q1, q2
        q0 += -qb * gx - qc * gy - q3 * gz
        q1 += qa * gx + qc * gz - q3 * gy
        q2 += qa * gy - qb * gz + q3 * gx
        q3 += qa * gz + qb * gy - qc * gx
        norm = math.sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
        if norm > 0.0:
            self.q0, self.q1, self.q2, self.q3 = (
                q0 / norm, q1 / norm, q2 / norm, q3 / norm)

    def quaternion(self) -> Tuple[float, float, float, float]:
        return self.q1, self.q2, self.q3, self.q0


class SerialBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__('wheeltec_arm_six')

        self.declare_parameter('usart_port_name', '/dev/wheeltec_controller')
        self.declare_parameter('serial_baud_rate', 115200)
        self.declare_parameter('robot_frame_id', 'base_link')
        self.declare_parameter('odom_frame_id', 'odom_combined')
        self.declare_parameter('product_number', 1)
        self.declare_parameter('joint_num', 4)
        self.declare_parameter('arm_command_topic', 'voice_joint_states')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('echo_joint_states', True)
        self.declare_parameter('init_arm_on_startup', True)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('read_timeout_sec', 0.02)

        self.usart_port_name = str(self.get_parameter('usart_port_name').value)
        self.serial_baud_rate = int(self.get_parameter('serial_baud_rate').value)
        self.robot_frame_id = str(self.get_parameter('robot_frame_id').value)
        self.odom_frame_id = str(self.get_parameter('odom_frame_id').value)
        self.product_number = int(self.get_parameter('product_number').value) or 1
        self.arm_command_topic = str(self.get_parameter('arm_command_topic').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.echo_joint_states = bool(self.get_parameter('echo_joint_states').value)
        self.init_arm_on_startup = bool(self.get_parameter('init_arm_on_startup').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.read_timeout_sec = float(self.get_parameter('read_timeout_sec').value)

        self.robot_pos = [0.0, 0.0, 0.0]
        self.robot_vel = [0.0, 0.0, 0.0]
        self.power_voltage = 0.0
        self.accel = (0.0, 0.0, 0.0)
        self.gyro = (0.0, 0.0, 0.0)
        self.last_arm = [0.0, 0.0, 0.0, 1.57, 0.0, 0.0]
        self.wheel_pos = [0.0, 0.0, 0.0, 0.0]
        self.rx_buffer = bytearray()
        self.ser = None
        self.ahrs = MahonyAhrs()
        self.last_time = self.get_clock().now()
        self.voltage_count = 0

        self.odom_pub = self.create_publisher(Odometry, '/odom', 50)
        self.imu_pub = self.create_publisher(Imu, '/imu', 20)
        self.voltage_pub = self.create_publisher(Float32, '/PowerVoltage', 10)
        self.pose_pub = self.create_publisher(Pose, '/pose', 20)
        self.joint_pub = (self.create_publisher(JointState, '/joint_states', 10)
                          if self.echo_joint_states else None)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.create_subscription(
            JointState, self.arm_command_topic, self.arm_command_cb, 100)
        self.create_subscription(
            Twist, self.cmd_vel_topic, self.cmd_vel_cb, 100)

        if pyserial is None:
            self.get_logger().error(
                'python3-serial is not installed. Run: sudo apt install python3-serial')
        else:
            self._open_serial()
            if self.ser is not None and self.ser.is_open and self.init_arm_on_startup:
                self.send_arm_frame(self.last_arm, DEFAULT_MODE)
                self.get_logger().info('sent startup arm pose [0,0,0,1.57,0,0]')

        self.timer = self.create_timer(1.0 / LOOP_HZ, self.control_loop)
        self.get_logger().info(
            'wheeltec ROS2 Foxy sim2real bridge started: arm_topic=%s, cmd_vel=%s, serial=%s@%d' % (
                self.arm_command_topic, self.cmd_vel_topic,
                self.usart_port_name, self.serial_baud_rate))

    def _open_serial(self) -> None:
        try:
            self.ser = pyserial.Serial(
                port=self.usart_port_name,
                baudrate=self.serial_baud_rate,
                timeout=self.read_timeout_sec,
            )
            self.get_logger().info(
                'serial opened: %s @ %d' % (self.usart_port_name, self.serial_baud_rate))
        except Exception as exc:  # pragma: no cover - hardware dependent
            self.ser = None
            self.get_logger().error(
                'cannot open serial %s: %s' % (self.usart_port_name, exc))

    def _write(self, data: Iterable[int]) -> None:
        if self.ser is None or not self.ser.is_open:
            return
        try:
            self.ser.write(bytes(data))
        except Exception as exc:  # pragma: no cover - hardware dependent
            self.get_logger().error('serial write failed: %s' % exc)

    def cmd_vel_cb(self, msg: Twist) -> None:
        frame = bytearray(SEND_DATA_SIZE_CHASSIS)
        frame[0] = FRAME_HEADER
        frame[1] = self.product_number & 0xFF
        frame[2] = 0
        frame[3], frame[4] = _pack_fixed(msg.linear.x)
        frame[5], frame[6] = _pack_fixed(msg.linear.y)
        frame[7], frame[8] = _pack_fixed(msg.angular.z)
        frame[9] = _xor_checksum(frame, 9)
        frame[10] = FRAME_TAIL
        self._write(frame)

    def arm_command_cb(self, msg: JointState) -> None:
        values = self._extract_arm_positions(msg)
        if values is None:
            return
        self.last_arm = values
        self.send_arm_frame(values, DEFAULT_MODE)
        if self.joint_pub is not None:
            self.publish_joint_echo()

    def _extract_arm_positions(self, msg: JointState) -> Optional[List[float]]:
        if len(msg.position) >= 6:
            return [float(v) for v in msg.position[:6]]

        if msg.name and msg.position and len(msg.name) == len(msg.position):
            by_name = dict(zip(msg.name, msg.position))
            if all(name in by_name for name in ARM_JOINT_NAMES):
                return [float(by_name[name]) for name in ARM_JOINT_NAMES]

        self.get_logger().warn(
            '%s requires six arm positions [joint1..joint6], got %d' % (
                self.arm_command_topic, len(msg.position)))
        return None

    def send_arm_frame(self, values: Sequence[float], mode: int = DEFAULT_MODE) -> None:
        frame = bytearray(SEND_DATA_SIZE_ARM)
        frame[0] = FRAME_HEADER_ARM
        for i in range(6):
            frame[2 * i + 1], frame[2 * i + 2] = _pack_fixed(values[i])
        frame[13] = int(mode) & 0xFF
        frame[14] = _xor_checksum(frame, 14)
        frame[15] = FRAME_TAIL_ARM
        self._write(frame)

    def _read_sensor_frame(self) -> Optional[bytes]:
        if self.ser is None or not self.ser.is_open:
            return None
        try:
            waiting = self.ser.in_waiting if hasattr(self.ser, 'in_waiting') else 0
            chunk = self.ser.read(max(RECEIVE_DATA_SIZE, int(waiting)))
        except Exception as exc:  # pragma: no cover - hardware dependent
            self.get_logger().error('serial read failed: %s' % exc)
            return None

        if chunk:
            self.rx_buffer.extend(chunk)
        if len(self.rx_buffer) > 512:
            del self.rx_buffer[:-RECEIVE_DATA_SIZE]

        while len(self.rx_buffer) >= RECEIVE_DATA_SIZE:
            try:
                start = self.rx_buffer.index(FRAME_HEADER)
            except ValueError:
                self.rx_buffer.clear()
                return None
            if start > 0:
                del self.rx_buffer[:start]
            if len(self.rx_buffer) < RECEIVE_DATA_SIZE:
                return None
            if self.rx_buffer[23] != FRAME_TAIL:
                del self.rx_buffer[0]
                continue
            frame = bytes(self.rx_buffer[:RECEIVE_DATA_SIZE])
            del self.rx_buffer[:RECEIVE_DATA_SIZE]
            if frame[22] == _xor_checksum(frame, 22):
                return frame
            self.get_logger().warn('discarded sensor frame with bad checksum')
        return None

    def parse_sensor_frame(self, frame: bytes) -> None:
        self.robot_vel[0] = _int16(frame[2], frame[3]) / 1000.0
        self.robot_vel[1] = _int16(frame[4], frame[5]) / 1000.0
        self.robot_vel[2] = _int16(frame[6], frame[7]) / 1000.0
        self.accel = (
            _int16(frame[8], frame[9]) / ACCEL_RATIO,
            _int16(frame[10], frame[11]) / ACCEL_RATIO,
            _int16(frame[12], frame[13]) / ACCEL_RATIO,
        )
        self.gyro = (
            _int16(frame[14], frame[15]) * GYROSCOPE_RATIO,
            _int16(frame[16], frame[17]) * GYROSCOPE_RATIO,
            _int16(frame[18], frame[19]) * GYROSCOPE_RATIO,
        )
        self.power_voltage = _int16(frame[20], frame[21]) / 1000.0

    def control_loop(self) -> None:
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        if dt <= 0.0 or dt > 1.0:
            dt = 1.0 / AHRS_HZ

        frame = self._read_sensor_frame()
        if frame is None:
            return

        self.parse_sensor_frame(frame)
        x, y, yaw = self.robot_pos
        vx, vy, wz = self.robot_vel
        self.robot_pos[0] = x + (vx * math.cos(yaw) - vy * math.sin(yaw)) * dt
        self.robot_pos[1] = y + (vx * math.sin(yaw) + vy * math.cos(yaw)) * dt
        self.robot_pos[2] = yaw + wz * dt

        # Approximate wheel rotation for RViz/digital twin TF continuity.
        wheel_radius = 0.032
        wheel_delta = (vx / wheel_radius) * dt
        self.wheel_pos[0] += wheel_delta
        self.wheel_pos[1] += wheel_delta
        self.wheel_pos[2] += wheel_delta
        self.wheel_pos[3] += wheel_delta

        gx, gy, gz = self.gyro
        ax, ay, az = self.accel
        self.ahrs.update(gx, gy, gz, ax, ay, az, dt)

        self.publish_odom(now)
        self.publish_pose()
        self.publish_imu(now)
        self.publish_voltage()
        if self.joint_pub is not None:
            self.publish_joint_echo()

    def publish_odom(self, stamp) -> None:
        qx, qy, qz, qw = _yaw_to_quaternion(self.robot_pos[2])
        msg = Odometry()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = self.odom_frame_id
        msg.child_frame_id = self.robot_frame_id
        msg.pose.pose.position.x = self.robot_pos[0]
        msg.pose.pose.position.y = self.robot_pos[1]
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.twist.twist.linear.x = self.robot_vel[0]
        msg.twist.twist.linear.y = self.robot_vel[1]
        msg.twist.twist.angular.z = self.robot_vel[2]
        stopped = abs(self.robot_vel[0]) < 1e-6 and abs(self.robot_vel[2]) < 1e-6
        msg.pose.covariance = list(map(float, (ODOM_POSE_COVARIANCE_STOPPED if stopped else ODOM_POSE_COVARIANCE)))
        msg.twist.covariance = list(map(float, (ODOM_TWIST_COVARIANCE_STOPPED if stopped else ODOM_TWIST_COVARIANCE)))
        self.odom_pub.publish(msg)

        if self.tf_broadcaster is not None:
            tfm = TransformStamped()
            tfm.header.stamp = msg.header.stamp
            tfm.header.frame_id = self.odom_frame_id
            tfm.child_frame_id = self.robot_frame_id
            tfm.transform.translation.x = self.robot_pos[0]
            tfm.transform.translation.y = self.robot_pos[1]
            tfm.transform.translation.z = 0.0
            tfm.transform.rotation.x = qx
            tfm.transform.rotation.y = qy
            tfm.transform.rotation.z = qz
            tfm.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tfm)

    def publish_pose(self) -> None:
        qx, qy, qz, qw = _yaw_to_quaternion(self.robot_pos[2])
        msg = Pose()
        msg.position.x = self.robot_pos[0]
        msg.position.y = self.robot_pos[1]
        msg.position.z = self.robot_pos[2]
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw
        self.pose_pub.publish(msg)

    def publish_imu(self, stamp) -> None:
        qx, qy, qz, qw = self.ahrs.quaternion()
        gx, gy, gz = self.gyro
        ax, ay, az = self.accel
        msg = Imu()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = 'gyro_link'
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw
        msg.orientation_covariance[0] = 1e6
        msg.orientation_covariance[4] = 1e6
        msg.orientation_covariance[8] = 1e-6
        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz
        msg.angular_velocity_covariance[0] = 1e6
        msg.angular_velocity_covariance[4] = 1e6
        msg.angular_velocity_covariance[8] = 1e-6
        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az
        self.imu_pub.publish(msg)

    def publish_voltage(self) -> None:
        self.voltage_count += 1
        if self.voltage_count > 10:
            self.voltage_count = 0
            self.voltage_pub.publish(Float32(data=float(self.power_voltage)))

    def publish_joint_echo(self) -> None:
        if self.joint_pub is None:
            return
        joint6 = self.last_arm[5]
        positions = [
            self.wheel_pos[0], self.wheel_pos[1], self.wheel_pos[2], self.wheel_pos[3],
            self.last_arm[0], self.last_arm[1], self.last_arm[2], self.last_arm[3], self.last_arm[4],
            joint6, -joint6, joint6, joint6, joint6, joint6,
        ]
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(ALL_JOINT_NAMES)
        msg.position = positions
        self.joint_pub.publish(msg)

    def send_stop_frame(self) -> None:
        frame = bytearray(SEND_DATA_SIZE_CHASSIS)
        frame[0] = FRAME_HEADER
        frame[1] = 0
        frame[2] = 0
        frame[9] = _xor_checksum(frame, 9)
        frame[10] = FRAME_TAIL
        self._write(frame)

    def destroy_node(self) -> None:
        if self.ser is not None and self.ser.is_open:
            self.send_stop_frame()
            self.ser.close()
            self.get_logger().info('serial closed, stop frame sent')
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SerialBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
