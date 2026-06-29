"""
serial_bridge.py — WHEELTEC mini_4wd_six_arm sim2real 串口桥接节点 (ROS2)

由 ROS1 wheeltec_arm_pick/src/wheeltec_six_arm_pick/wheeltec_arm_six.cpp
及 quaternion_solution.cpp 移植而来。串口协议 (帧头 0x7B/0xAA、XOR 校验、
速度/角度 ×1000 有符号定点、24B 接收帧) 与真机下位机 100% 一致。

数据流:
    仿真 (浏览器 Three.js) ──rosbridge──►  voice_joint_states (JointState) ──► 本节点
                                          cmd_vel            (Twist)      ──► 本节点
                                            │
                                            ▼  打包成下位机帧, 串口下发
                                          STM32
                                            ▲  串口读取 24B 帧
                                            │
    仿真 ◄──rosbridge── /odom (Odometry) ◄┘
                       /imu  (Imu)
                       /PowerVoltage (Float32)

参数 (与 ROS1 base_serial.launch 对应):
    usart_port_name  默认 /dev/wheeltec_controller
    serial_baud_rate 默认 115200
    robot_frame_id   默认 base_link
    odom_frame_id    默认 odom_combined
    joint_num        机械臂关节起始编号 (sim2real voice_joint_states 固定从 0 读 6 值, 此处保留兼容)
"""
import math
import struct

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, Pose, TransformStamped
from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

try:
    import serial as pyserial
except ImportError:
    pyserial = None


# ────────────────────────── 协议常量 (与 wheeltec_arm_six.h 一致) ──────────────────────────
FRAME_HEADER = 0x7B          # 底盘帧头
FRAME_TAIL = 0x7D            # 底盘帧尾
FRAME_HEADER_ARM = 0xAA      # 机械臂帧头
FRAME_TAIL_ARM = 0xBB        # 机械臂帧尾
RECEIVE_DATA_SIZE = 24       # 下位机 → 上位机 帧长
SEND_DATA_SIZE_ARM = 16      # 上位机 → 下位机 机械臂帧长
SEND_DATA_SIZE_CHASSIS = 11  # 上位机 → 下位机 底盘帧长
DEFAULT_MODE = 1
FOLLOWER_MODE = 2
GYROSCOPE_RATIO = 0.00026644   # 1/65.5/57.30, MPU6050 FS_SEL=1
ACCEL_RATIO = 16384.0          # 量程 ±2g
SAMPLING_FREQ = 20.0           # Mahony 采样频率 (Hz)

# 协方差矩阵 (与 ROS1 一致)
ODOM_POSE_COVARIANCE = [
    1e-3, 0, 0, 0, 0, 0, 0, 1e-3, 0, 0, 0, 0, 0, 0, 1e6, 0, 0, 0,
    0, 0, 0, 1e6, 0, 0, 0, 0, 0, 0, 1e6, 0, 0, 0, 0, 0, 0, 1e3,
]
ODOM_POSE_COVARIANCE2 = [
    1e-9, 0, 0, 0, 0, 0, 0, 1e-3, 1e-9, 0, 0, 0, 0, 0, 1e6, 0, 0, 0,
    0, 0, 0, 1e6, 0, 0, 0, 0, 0, 0, 1e6, 0, 0, 0, 0, 0, 0, 1e-9,
]
ODOM_TWIST_COVARIANCE = [
    1e-3, 0, 0, 0, 0, 0, 0, 1e-3, 0, 0, 0, 0, 0, 0, 1e6, 0, 0, 0,
    0, 0, 0, 1e6, 0, 0, 0, 0, 0, 0, 1e6, 0, 0, 0, 0, 0, 0, 1e3,
]
ODOM_TWIST_COVARIANCE2 = [
    1e-9, 0, 0, 0, 0, 0, 0, 1e-3, 1e-9, 0, 0, 0, 0, 0, 1e6, 0, 0, 0,
    0, 0, 0, 1e6, 0, 0, 0, 0, 0, 0, 1e6, 0, 0, 0, 0, 0, 0, 1e-9,
]


def _yaw_to_quaternion(yaw):
    """yaw (rad) → (x, y, z, w) (等价 tf::createQuaternionMsgFromYaw)"""
    h = yaw * 0.5
    return (0.0, 0.0, math.sin(h), math.cos(h))


def _int16(high, low):
    """两个字节按 (high<<8)|low 组合成有符号 int16 (下位机高位在前)"""
    val = (high << 8) | low
    if val >= 0x8000:
        val -= 0x10000
    return val


def _pack_fixed(v):
    """浮点 → ×1000 有符号定点, 返回 (high, low) 两字节 (与下位机一致, 高位在前)"""
    raw = int(round(v * 1000.0))
    if raw > 32767:
        raw = 32767
    elif raw < -32768:
        raw = -32768
    return (raw >> 8) & 0xFF, raw & 0xFF


def _xor_checksum(data, count):
    """XOR 校验: data 前 count 字节按位异或 (与 Check_Sum 一致)"""
    s = 0
    for i in range(count):
        s ^= data[i]
    return s & 0xFF


class MahonyAhrs:
    """Mahony 互补滤波姿态解算 (移植 quaternion_solution.cpp)。
    twoKi=0 时退化为纯比例校正; 采样频率固定 SAMPLING_FREQ。"""

    def __init__(self):
        self.two_kp = 1.0
        self.two_ki = 0.0
        self.q0, self.q1, self.q2, self.q3 = 1.0, 0.0, 0.0, 0.0
        self.integral_fb = [0.0, 0.0, 0.0]

    def update(self, gx, gy, gz, ax, ay, az):
        q0, q1, q2, q3 = self.q0, self.q1, self.q2, self.q3
        # 仅当加速度有效时计算反馈
        if not (ax == 0.0 and ay == 0.0 and az == 0.0):
            norm = math.sqrt(ax * ax + ay * ay + az * az)
            inv = 1.0 / norm
            ax *= inv
            ay *= inv
            az *= inv
            # 四元数 → 方向余弦第三行
            halfvx = q1 * q3 - q0 * q2
            halfvy = q0 * q1 + q2 * q3
            halfvz = q0 * q0 - 0.5 + q3 * q3
            # 估计重力方向 × 测量重力方向
            halfex = ay * halfvz - az * halfvy
            halfey = az * halfvx - ax * halfvz
            halfez = ax * halfvy - ay * halfvx
            if self.two_ki > 0.0:
                dt = 1.0 / SAMPLING_FREQ
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
        # 积分四元数变化率
        dt2 = 0.5 / SAMPLING_FREQ
        gx *= dt2
        gy *= dt2
        gz *= dt2
        qa, qb, qc = q0, q1, q2
        q0 += -qb * gx - qc * gy - q3 * gz
        q1 += qa * gx + qc * gz - q3 * gy
        q2 += qa * gy - qb * gz + q3 * gx
        q3 += qa * gz + qb * gy - qc * gx
        # 归一化
        norm = math.sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
        inv = 1.0 / norm
        self.q0, self.q1, self.q2, self.q3 = q0 * inv, q1 * inv, q2 * inv, q3 * inv

    def quaternion(self):
        # 返回 (x, y, z, w) 与 sensor_msgs/Imu.orientation 字段顺序对应
        return (self.q1, self.q2, self.q3, self.q0)


class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('wheeltec_arm_six')

        # ── 参数 (对应 ROS1 private_nh.param) ──
        self.declare_parameter('usart_port_name', '/dev/wheeltec_controller')
        self.declare_parameter('serial_baud_rate', 115200)
        self.declare_parameter('robot_frame_id', 'base_link')
        self.declare_parameter('odom_frame_id', 'odom_combined')
        self.declare_parameter('joint_num', 4)
        self.declare_parameter('product_number', 0)
        # STM32 上行帧不含机械臂关节角 (开环舵机), 故 /joint_states 无法来自串口。
        # 开启此项后, 把收到的 voice_joint_states 机械臂关节回显到 /joint_states,
        # 供仿真镜像 (真机无臂编码器反馈时的占位)。若已有 joint_state_publisher 则勿开。
        self.declare_parameter('echo_joint_states', False)
        self.usart_port_name = self.get_parameter('usart_port_name').value
        self.serial_baud_rate = int(self.get_parameter('serial_baud_rate').value)
        self.robot_frame_id = self.get_parameter('robot_frame_id').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.joint_num = int(self.get_parameter('joint_num').value)
        self.product_number = int(self.get_parameter('product_number').value)
        self.echo_joint_states = self.get_parameter('echo_joint_states').value

        # ── 状态 ──
        self.robot_pos = [0.0, 0.0, 0.0]   # X, Y, Z(=yaw)
        self.robot_vel = [0.0, 0.0, 0.0]
        self.power_voltage = 0.0
        self.ahrs = MahonyAhrs()
        self.ser = None
        self._voltage_count = 0
        self._accel = (0.0, 0.0, 0.0)
        self._gyro = (0.0, 0.0, 0.0)

        # ── 发布者 ──
        self.odom_pub = self.create_publisher(Odometry, 'odom', 50)
        self.imu_pub = self.create_publisher(Imu, 'imu', 20)
        self.voltage_pub = self.create_publisher(Float32, 'PowerVoltage', 10)
        self.pose_pub = self.create_publisher(Pose, 'pose', 20)
        # odom → base_link TF 广播 (供 robot_state_publisher 锚定, rviz/导航)
        self.tf_broadcaster = TransformBroadcaster(self)
        # 关节回显 (可选): 仿真镜像真机指令值, 因硬件无臂编码器反馈
        if self.echo_joint_states:
            self.js_pub = self.create_publisher(JointState, '/joint_states', 10)
        else:
            self.js_pub = None

        # ── 订阅者 (仿真 → 真机) ──
        # voice_joint_states: position[0..5] = [joint1..joint5, joint6(夹爪)]
        self.create_subscription(
            JointState, 'voice_joint_states', self.voice_joint_states_cb, 100)
        # cmd_vel: 底盘线速度/角速度
        self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_cb, 100)

        # ── 串口初始化 ──
        if pyserial is None:
            self.get_logger().error(
                '未安装 pyserial, 请执行: pip install pyserial '
                '(或 sudo apt install python3-serial). 串口桥接不可用。')
        else:
            self._open_serial()
            if self.ser is not None and self.ser.is_open:
                self.init_joint_states()  # 开机预设位姿 (j4=1.57)

        # ── 控制循环 (20Hz, 与 SAMPLING_FREQ 一致) ──
        self._last_time = self.get_clock().now()
        self.timer = self.create_timer(1.0 / SAMPLING_FREQ, self.control_loop)
        self.get_logger().info('wheeltec_arm_six (sim2real bridge) 已启动')

    # ─────────────── 串口 ───────────────
    def _open_serial(self):
        try:
            self.ser = pyserial.Serial(
                port=self.usart_port_name,
                baudrate=self.serial_baud_rate,
                timeout=2.0,
            )
            self.get_logger().info(
                f'串口已打开: {self.usart_port_name} @ {self.serial_baud_rate}')
        except Exception as e:
            self.ser = None
            self.get_logger().error(
                f'无法打开串口 {self.usart_port_name}: {e}. '
                '请检查线缆与权限 (sudo chmod 666 /dev/wheeltec_controller)')

    def _write(self, data):
        if self.ser is None or not self.ser.is_open:
            return
        try:
            self.ser.write(bytes(data))
        except Exception as e:
            self.get_logger().error(f'串口写入失败: {e}')

    # ─────────────── 发送: 底盘 (11B) ───────────────
    def cmd_vel_cb(self, twist):
        """对应 turn_on_robot::Cmd_Vel_Callback"""
        tx = bytearray(SEND_DATA_SIZE_CHASSIS)
        tx[0] = FRAME_HEADER
        tx[1] = self.product_number if self.product_number else 1
        tx[2] = 0  # 使能标志
        # 速度 ×1000 定点, 高位在前 (tx[3]=hi, tx[4]=lo)
        tx[3], tx[4] = _pack_fixed(twist.linear.x)
        tx[5], tx[6] = _pack_fixed(twist.linear.y)
        tx[7], tx[8] = _pack_fixed(twist.angular.z)
        tx[9] = _xor_checksum(tx, 9)
        tx[10] = FRAME_TAIL
        self._write(tx)

    # ─────────────── 发送: 机械臂 (16B) ───────────────
    def voice_joint_states_cb(self, msg):
        """对应 turn_on_robot::voice_joint_states_Callback
        position[0..5] → 关节1~5 + 夹爪6, 每个角度 ×1000 定点"""
        pos = msg.position
        if len(pos) < 6:
            self.get_logger().warn(
                f'voice_joint_states position 长度 {len(pos)} < 6, 忽略')
            return
        tx = bytearray(SEND_DATA_SIZE_ARM)
        tx[0] = FRAME_HEADER_ARM
        # joint i (0..5): tx[2i+1]=hi, tx[2i+2]=lo
        for i in range(6):
            tx[2 * i + 1], tx[2 * i + 2] = _pack_fixed(pos[i])
        tx[13] = DEFAULT_MODE
        tx[14] = _xor_checksum(tx, 14)
        tx[15] = FRAME_TAIL_ARM
        self._write(tx)

        # 关节回显: 硬件无臂编码器, 把指令值作为 /joint_states 占位供仿真镜像
        if self.js_pub is not None:
            out = JointState()
            out.header.stamp = self.get_clock().now().to_msg()
            # 6 个臂/夹爪关节名与 URDF 一致 (joint1..joint5, joint6)
            out.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
            out.position = list(pos[:6])
            self.js_pub.publish(out)

    def init_joint_states(self):
        """对应 turn_on_robot::init_joint_states: 开机运动到预设 (j4=1.57, 其余 0)"""
        tx = bytearray(SEND_DATA_SIZE_ARM)
        tx[0] = FRAME_HEADER_ARM
        # j1,j2,j3 = 0; j4 = 1.57; j5,j6 = 0
        vals = [0.0, 0.0, 0.0, 1.57, 0.0, 0.0]
        for i in range(6):
            tx[2 * i + 1], tx[2 * i + 2] = _pack_fixed(vals[i])
        tx[13] = DEFAULT_MODE
        tx[14] = _xor_checksum(tx, 14)
        tx[15] = FRAME_TAIL_ARM
        self._write(tx)
        self.get_logger().info('已发送开机预设位姿 (j4=1.57rad)')

    # ─────────────── 接收: 24B 传感器帧 ───────────────
    def get_sensor_data(self):
        """对应 turn_on_robot::Get_Sensor_Data: 读 24B, 帧头帧尾 + 校验, 解析速度/IMU/电压"""
        if self.ser is None or not self.ser.is_open:
            return False
        try:
            rx_pr = self.ser.read(RECEIVE_DATA_SIZE)
        except Exception as e:
            self.get_logger().error(f'串口读取失败: {e}')
            return False
        if len(rx_pr) < RECEIVE_DATA_SIZE:
            return False

        # 查找帧头帧尾位置 (处理字节漂移), 对应 Get_Sensor_Data 的对齐逻辑
        header_pos = -1
        tail_pos = -1
        for j in range(RECEIVE_DATA_SIZE):
            if rx_pr[j] == FRAME_HEADER:
                header_pos = j
            elif rx_pr[j] == FRAME_TAIL:
                tail_pos = j
        if header_pos < 0 or tail_pos < 0:
            return False
        if tail_pos == header_pos + 23:
            rx = rx_pr
        elif header_pos == tail_pos + 1:
            # 帧跨缓冲区边界, 旋转使 header 落到 rx[0]
            rx = bytes(rx_pr[(k + header_pos) % RECEIVE_DATA_SIZE]
                       for k in range(RECEIVE_DATA_SIZE))
        else:
            return False

        if rx[0] != FRAME_HEADER or rx[23] != FRAME_TAIL:
            return False
        # 校验位 (允许 header_pos==tail_pos+1 时跳过, 与 ROS1 一致)
        if rx[22] != _xor_checksum(rx, 22) and header_pos != tail_pos + 1:
            return False

        self.robot_vel[0] = _int16(rx[2], rx[3]) / 1000.0   # X 速度
        self.robot_vel[1] = _int16(rx[4], rx[5]) / 1000.0   # Y 速度 (全向底盘)
        self.robot_vel[2] = _int16(rx[6], rx[7]) / 1000.0   # Z 角速度

        ax = _int16(rx[8], rx[9])
        ay = _int16(rx[10], rx[11])
        az = _int16(rx[12], rx[13])
        gx = _int16(rx[14], rx[15])
        gy = _int16(rx[16], rx[17])
        gz = _int16(rx[18], rx[19])
        self._accel = (ax / ACCEL_RATIO, ay / ACCEL_RATIO, az / ACCEL_RATIO)
        self._gyro = (gx * GYROSCOPE_RATIO, gy * GYROSCOPE_RATIO, gz * GYROSCOPE_RATIO)

        volt = _int16(rx[20], rx[21])
        self.power_voltage = volt / 1000.0
        return True

    # ─────────────── 发布 ───────────────
    def publish_odom(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.odom_frame_id
        msg.child_frame_id = self.robot_frame_id
        msg.pose.pose.position.x = self.robot_pos[0]
        msg.pose.pose.position.y = self.robot_pos[1]
        msg.pose.pose.position.z = self.robot_pos[2]
        qx, qy, qz, qw = _yaw_to_quaternion(self.robot_pos[2])
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.twist.twist.linear.x = self.robot_vel[0]
        msg.twist.twist.linear.y = self.robot_vel[1]
        msg.twist.twist.angular.z = self.robot_vel[2]
        # 协方差: 静止 vs 运动 (与 ROS1 一致)
        if self.robot_vel[0] == 0.0 and self.robot_vel[2] == 0.0:
            msg.pose.covariance = ODOM_POSE_COVARIANCE2
            msg.twist.covariance = ODOM_TWIST_COVARIANCE2
        else:
            msg.pose.covariance = ODOM_POSE_COVARIANCE
            msg.twist.covariance = ODOM_TWIST_COVARIANCE
        self.odom_pub.publish(msg)
        # 广播 odom → base_link TF (供 robot_state_publisher 锚定, rviz/导航)
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

    def publish_pose(self):
        msg = Pose()
        msg.position.x = self.robot_pos[0]
        msg.position.y = self.robot_pos[1]
        msg.position.z = self.robot_pos[2]
        qx, qy, qz, qw = _yaw_to_quaternion(self.robot_pos[2])
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw
        self.pose_pub.publish(msg)

    def publish_imu(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gyro_link'
        qx, qy, qz, qw = self.ahrs.quaternion()
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw
        msg.orientation_covariance[0] = 1e6
        msg.orientation_covariance[4] = 1e6
        msg.orientation_covariance[8] = 1e-6
        gx, gy, gz = self._gyro
        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz
        msg.angular_velocity_covariance[0] = 1e6
        msg.angular_velocity_covariance[4] = 1e6
        msg.angular_velocity_covariance[8] = 1e-6
        ax, ay, az = self._accel
        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az
        self.imu_pub.publish(msg)

    def publish_voltage(self):
        self._voltage_count += 1
        if self._voltage_count > 10:
            self._voltage_count = 0
            self.voltage_pub.publish(Float32(data=self.power_voltage))

    # ─────────────── 主循环 (20Hz) ───────────────
    def control_loop(self):
        now = self.get_clock().now()
        dt = (now - self._last_time).nanoseconds * 1e-9
        self._last_time = now
        if dt <= 0.0 or dt > 1.0:
            dt = 1.0 / SAMPLING_FREQ

        if not self.get_sensor_data():
            return

        # 里程积分解算
        x, y, yaw = self.robot_pos
        vx, vy, wz = self.robot_vel
        self.robot_pos[0] = x + (vx * math.cos(yaw) - vy * math.sin(yaw)) * dt
        self.robot_pos[1] = y + (vx * math.sin(yaw) + vy * math.cos(yaw)) * dt
        self.robot_pos[2] = yaw + wz * dt

        # AHRS 姿态解算
        gx, gy, gz = self._gyro
        ax, ay, az = self._accel
        self.ahrs.update(gx, gy, gz, ax, ay, az)

        self.publish_odom()
        self.publish_pose()
        self.publish_imu()
        self.publish_voltage()

    def destroy_node(self):
        # 析构时发送零速度停车帧 (对应 ~turn_on_robot)
        if self.ser is not None and self.ser.is_open:
            tx = bytearray(SEND_DATA_SIZE_CHASSIS)
            tx[0] = FRAME_HEADER
            tx[1] = 0
            tx[2] = 0
            tx[9] = _xor_checksum(tx, 9)
            tx[10] = FRAME_TAIL
            self._write(tx)
            self.ser.close()
            self.get_logger().info('串口已关闭, 已发送停车帧')
        super().destroy_node()


def main(args=None):
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
