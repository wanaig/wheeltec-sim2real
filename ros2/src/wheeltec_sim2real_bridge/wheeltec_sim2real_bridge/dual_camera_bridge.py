"""
dual_camera_bridge.py - publish two USB camera streams for sim2real.

Camera roles:
  eye_in_hand  - camera mounted on the gripper/arm
  eye_to_hand  - fixed external camera looking at the workspace

The node publishes JPEG CompressedImage topics so the browser can subscribe via
rosbridge without requiring image_transport compressed plugins.
"""

import threading
import time
import os
import glob
import re
import sys as _sys
for _p in list(_sys.path):
    if 'noetic' in _p or 'ros1' in _p:
        _sys.path.remove(_p)
from typing import Optional, Set

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

try:
    import cv2
except ImportError:  # pragma: no cover - handled at runtime on robot
    cv2 = None


class CameraWorker:
    def __init__(self, node: Node, role: str, device: str, topic: str,
                 frame_id: str, width: int, height: int, fps: float,
                 jpeg_quality: int) -> None:
        self.node = node
        self.role = role
        self.device = device
        self.device_candidates = [device]
        self.topic = topic
        self.frame_id = frame_id
        self.width = width
        self.height = height
        self.fps = max(1.0, float(fps))
        self.jpeg_quality = max(10, min(95, int(jpeg_quality)))
        self.pub = node.create_publisher(CompressedImage, topic, 10)
        self.cap = None
        self.running = False
        self.thread: Optional[threading.Thread] = None
        self._warned_no_frame = False
        self._warned_encode = False
        self._published_once = False

    @staticmethod
    def _device_key(device: str) -> str:
        return str(device).strip()

    @staticmethod
    def _parse_index(device: str):
        text = str(device).strip()
        if text.isdigit():
            return int(text)
        if text.startswith('/dev/video') and text[10:].isdigit():
            return int(text[10:])
        return None

    @staticmethod
    def _video_sort_key(device: str):
        match = re.search(r'/dev/video(\d+)$', device)
        if match:
            return int(match.group(1))
        return 9999

    def _resolved_device(self, device: str) -> str:
        text = str(device).strip()
        if text.startswith('/dev/') and os.path.exists(text):
            try:
                return os.path.realpath(text)
            except OSError:
                return text
        return text

    def _warmup_capture(self) -> bool:
        if self.cap is None:
            return False
        for _ in range(5):
            ok, frame = self.cap.read()
            if ok and frame is not None:
                self._publish_frame(frame)
                return True
            time.sleep(0.05)
        return False

    def _publish_frame(self, frame) -> None:
        if hasattr(frame, 'shape') and len(frame.shape) == 3 and frame.shape[2] == 2:
            frame = cv2.cvtColor(frame, cv2.COLOR_YUV2BGR_YUYV)
        ok, jpg = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        if not ok:
            if not self._warned_encode:
                self.node.get_logger().warn('%s camera jpeg encode failed' % self.role)
                self._warned_encode = True
            return
        msg = CompressedImage()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.format = 'jpeg'
        msg.data = jpg.tobytes()
        self.pub.publish(msg)
        if not self._published_once:
            self.node.get_logger().info('%s camera first frame published' % self.role)
            self._published_once = True

    def _try_open(self, device: str) -> bool:
        # Keep the old WHEELTEC behavior: open the device path directly.
        # Opening aliases as integer indexes can select a different V4L backend
        # and broke /dev/RgbCam on the robot.
        self.cap = cv2.VideoCapture(device)
        if not self.cap.isOpened():
            if self.cap is not None:
                self.cap.release()
            self.cap = None
            return False

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)

        # 某些 UVC 摄像头在刚打开时不会立刻出帧, 先做几次预热读取。
        if self._warmup_capture():
            self.device = device
            return True

        self.cap.release()
        self.cap = None
        return False

    def start(self, taken_devices: Set[str]) -> None:
        if cv2 is None:
            self.node.get_logger().error(
                'python3-opencv is not installed. Run: sudo apt install python3-opencv')
            return
        candidates = []
        for item in self.device_candidates:
            key = self._device_key(item)
            if key and key not in candidates:
                candidates.append(key)

        for video in sorted(glob.glob('/dev/video*'), key=self._video_sort_key):
            if video not in candidates:
                candidates.append(video)

        # 角色优先级不同, 但都允许回退到 /dev/video*。
        # 启动时会先打开外部眼, 所以手眼回退时会跳过外部眼已占用的设备。
        if self.role == 'eye_in_hand':
            for extra in ('/dev/video2', '/dev/video4', '/dev/video0', '/dev/video1'):
                if extra not in candidates:
                    candidates.append(extra)
        else:
            for extra in ('/dev/video1', '/dev/video0'):
                if extra not in candidates:
                    candidates.append(extra)

        for candidate in candidates:
            if not candidate:
                continue
            candidate_key = self._resolved_device(candidate)
            if candidate_key in taken_devices:
                continue
            if self._try_open(candidate):
                self.running = True
                taken_devices.add(self._resolved_device(self.device))
                self.thread = threading.Thread(target=self._loop, daemon=True)
                self.thread.start()
                self.node.get_logger().info(
                    '%s camera: %s -> %s (%dx%d @ %.1ffps)' % (
                        self.role, self.device, self.topic, self.width, self.height, self.fps))
                return

            self.node.get_logger().warn(
                '%s camera open failed: %s' % (self.role, candidate))

        self.node.get_logger().error(
            '%s camera disabled: no available device among %s' % (self.role, candidates))

    def stop(self) -> None:
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=1.0)
            self.thread = None
        if self.cap is not None:
            self.cap.release()
            self.cap = None

    def _loop(self) -> None:
        period = 1.0 / self.fps
        while self.running and rclpy.ok():
            t0 = time.time()
            ok, frame = self.cap.read()
            if not ok or frame is None:
                if not self._warned_no_frame:
                    self.node.get_logger().warn('%s camera read failed' % self.role)
                    self._warned_no_frame = True
                time.sleep(0.2)
                continue

            self._warned_no_frame = False
            self._publish_frame(frame)

            elapsed = time.time() - t0
            if elapsed < period:
                time.sleep(period - elapsed)


class DualCameraBridge(Node):
    def __init__(self) -> None:
        super().__init__('dual_camera_bridge')

        self.declare_parameter('eye_in_hand_device', '/dev/HandCam')
        self.declare_parameter('eye_to_hand_device', '/dev/RgbCam')
        self.declare_parameter('eye_in_hand_topic', '/eye_in_hand/image_raw/compressed')
        self.declare_parameter('eye_to_hand_topic', '/eye_to_hand/image_raw/compressed')
        self.declare_parameter('eye_in_hand_frame_id', 'eye_in_hand_camera')
        self.declare_parameter('eye_to_hand_frame_id', 'eye_to_hand_camera')
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('fps', 15.0)
        self.declare_parameter('jpeg_quality', 70)

        width = int(self.get_parameter('image_width').value)
        height = int(self.get_parameter('image_height').value)
        fps = float(self.get_parameter('fps').value)
        jpeg_quality = int(self.get_parameter('jpeg_quality').value)

        self.workers = [
            CameraWorker(
                self, 'eye_in_hand',
                str(self.get_parameter('eye_in_hand_device').value),
                str(self.get_parameter('eye_in_hand_topic').value),
                str(self.get_parameter('eye_in_hand_frame_id').value),
                width, height, fps, jpeg_quality),
            CameraWorker(
                self, 'eye_to_hand',
                str(self.get_parameter('eye_to_hand_device').value),
                str(self.get_parameter('eye_to_hand_topic').value),
                str(self.get_parameter('eye_to_hand_frame_id').value),
                width, height, fps, jpeg_quality),
        ]
        self._taken_devices = set()
        for worker in sorted(self.workers, key=lambda w: 0 if w.role == 'eye_to_hand' else 1):
            worker.start(self._taken_devices)

    def destroy_node(self) -> bool:
        for worker in self.workers:
            worker.stop()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DualCameraBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
