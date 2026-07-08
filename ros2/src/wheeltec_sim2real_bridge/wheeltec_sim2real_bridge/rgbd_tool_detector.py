"""
rgbd_tool_detector.py - RGB-D tabletop tool perception for industrial scenes.

This node follows the old WHEELTEC visual tracker idea:
  RGB image + depth image -> contour detection -> depth averaging -> object pose.

It publishes:
  /industrial_tools/annotations         std_msgs/String JSON
  /industrial_tools/debug_image/compressed  sensor_msgs/CompressedImage

The detector is intentionally dependency-light for ROS2 Foxy on Ubuntu 20.04:
OpenCV + numpy only, no deep-learning runtime required. Classification is a
geometry/color heuristic suitable for dataset bootstrapping and simulation state
annotation; trained detection can replace _classify_contour later.
"""

import json
import math
import time
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import String
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

try:
    import cv2
except ImportError:  # pragma: no cover - handled at runtime on robot
    cv2 = None


TOOL_CLASS_ID = {
    'screwdriver': 0,
    'wrench': 1,
    'nut': 2,
    'roller': 3,
    'screw': 4,
    'unknown': -1,
}

TOOL_CN = {
    'screwdriver': '螺丝刀',
    'wrench': '扳手',
    'nut': '螺母',
    'roller': '滚柱',
    'screw': '螺丝',
    'unknown': '未知工具',
}


def _stamp_to_float(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class RgbdToolDetector(Node):
    def __init__(self) -> None:
        super().__init__('rgbd_tool_detector')

        self.declare_parameter('rgb_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('annotations_topic', '/industrial_tools/annotations')
        self.declare_parameter('debug_image_topic', '/industrial_tools/debug_image/compressed')
        self.declare_parameter('frame_id', 'rgbd_camera')
        self.declare_parameter('world_frame', 'base_link')
        self.declare_parameter('depth_compressed', True)
        self.declare_parameter('rgb_compressed', True)
        self.declare_parameter('detect_hz', 5.0)
        self.declare_parameter('min_area_px', 180.0)
        self.declare_parameter('max_area_px', 60000.0)
        self.declare_parameter('min_depth_m', 0.12)
        self.declare_parameter('max_depth_m', 2.00)
        self.declare_parameter('depth_sync_tolerance_s', 5.0)
        self.declare_parameter('jpeg_quality', 70)

        self.rgb_topic = str(self.get_parameter('rgb_topic').value)
        self.depth_topic = str(self.get_parameter('depth_topic').value)
        self.info_topic = str(self.get_parameter('camera_info_topic').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.world_frame = str(self.get_parameter('world_frame').value)
        self.min_area = float(self.get_parameter('min_area_px').value)
        self.max_area = float(self.get_parameter('max_area_px').value)
        self.min_depth = float(self.get_parameter('min_depth_m').value)
        self.max_depth = float(self.get_parameter('max_depth_m').value)
        self.sync_tol = float(self.get_parameter('depth_sync_tolerance_s').value)
        self.jpeg_quality = max(10, min(95, int(self.get_parameter('jpeg_quality').value)))

        self._rgb_msg: Optional[Image] = None
        self._depth_msg: Optional[Image] = None
        self._info_msg: Optional[CameraInfo] = None
        self._last_warn = 0.0

        self.rgb_compressed = bool(self.get_parameter('rgb_compressed').value)
        if self.rgb_compressed:
            self.create_subscription(CompressedImage, self.rgb_topic + '/compressed', self._on_rgb, 10)
        else:
            self.create_subscription(Image, self.rgb_topic, self._on_rgb, 10)

        self.depth_compressed = bool(self.get_parameter('depth_compressed').value)
        if self.depth_compressed:
            depth_sub_topic = self.depth_topic + '/compressed'
            self.create_subscription(CompressedImage, depth_sub_topic, self._on_depth, 10)
        else:
            depth_sub_topic = self.depth_topic
            self.create_subscription(Image, self.depth_topic, self._on_depth, 10)

        self.create_subscription(CameraInfo, self.info_topic, self._on_info, 10)

        self.annotations_pub = self.create_publisher(
            String, str(self.get_parameter('annotations_topic').value), 10)
        self.debug_pub = self.create_publisher(
            CompressedImage, str(self.get_parameter('debug_image_topic').value), 5)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_available = False

        period = 1.0 / max(0.5, float(self.get_parameter('detect_hz').value))
        self.create_timer(period, self._tick)

        self.get_logger().info(
            'RGB-D tool detector started: rgb=%s%s depth=%s%s info=%s world_frame=%s' % (
                self.rgb_topic, '/compressed' if self.rgb_compressed else '',
                self.depth_topic, '/compressed' if self.depth_compressed else '',
                self.info_topic, self.world_frame))

    def _transform_to_world(self, x, y, z, camera_frame):
        """Transform a 3D point from camera optical frame to self.world_frame via TF.

        Returns [x, y, z] in world frame, or None if TF is unavailable.
        """
        try:
            tf = self._tf_buffer.lookup_transform(
                self.world_frame, camera_frame, Time())
        except Exception:
            if not self._tf_available:
                self._warn_throttled(
                    'TF %s -> %s not available, using camera-frame coords' % (
                        camera_frame, self.world_frame))
            return None
        self._tf_available = True

        t = tf.transform.translation
        q = tf.transform.rotation
        qx, qy, qz, qw = q.x, q.y, q.z, q.w
        r11 = 1 - 2 * (qy * qy + qz * qz)
        r12 = 2 * (qx * qy - qz * qw)
        r13 = 2 * (qx * qz + qy * qw)
        r21 = 2 * (qx * qy + qz * qw)
        r22 = 1 - 2 * (qx * qx + qz * qz)
        r23 = 2 * (qy * qz - qx * qw)
        r31 = 2 * (qx * qz - qy * qw)
        r32 = 2 * (qy * qz + qx * qw)
        r33 = 1 - 2 * (qx * qx + qy * qy)

        wx = r11 * x + r12 * y + r13 * z + t.x
        wy = r21 * x + r22 * y + r23 * z + t.y
        wz = r31 * x + r32 * y + r33 * z + t.z
        return [round(float(wx), 4), round(float(wy), 4), round(float(wz), 4)]

    def _on_rgb(self, msg: Image) -> None:
        self._rgb_msg = msg

    def _on_depth(self, msg: Image) -> None:
        self._depth_msg = msg

    def _on_info(self, msg: CameraInfo) -> None:
        self._info_msg = msg

    def _warn_throttled(self, text: str) -> None:
        now = time.time()
        if now - self._last_warn > 2.0:
            self.get_logger().warn(text)
            self._last_warn = now

    def _image_to_cv(self, msg):
        # CompressedImage (JPEG from image_transport republish)
        if isinstance(msg, CompressedImage):
            buf = np.frombuffer(msg.data, dtype=np.uint8)
            img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
            if img is None:
                raise ValueError('failed to decode compressed RGB image')
            return img

        # Raw Image
        h, w = int(msg.height), int(msg.width)
        enc = (msg.encoding or '').lower()
        data = np.frombuffer(msg.data, dtype=np.uint8)

        if enc in ('rgb8', 'bgr8'):
            img = data.reshape((h, int(msg.step // 3), 3))[:, :w, :]
            if enc == 'rgb8':
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            return img.copy()
        if enc in ('mono8', '8uc1'):
            img = data.reshape((h, int(msg.step)))[:, :w]
            return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        if enc in ('rgba8', 'bgra8'):
            img = data.reshape((h, int(msg.step // 4), 4))[:, :w, :]
            code = cv2.COLOR_RGBA2BGR if enc == 'rgba8' else cv2.COLOR_BGRA2BGR
            return cv2.cvtColor(img, code)

        raise ValueError('unsupported RGB encoding: %s' % msg.encoding)

    def _depth_to_meters(self, msg):
        # CompressedImage (PNG-encoded depth from image_transport republish)
        if isinstance(msg, CompressedImage):
            buf = np.frombuffer(msg.data, dtype=np.uint8)
            img = cv2.imdecode(buf, cv2.IMREAD_UNCHANGED)
            if img is None:
                raise ValueError('failed to decode compressed depth image')
            if img.dtype == np.uint16:
                return img.astype(np.float32) * 0.001  # mm → meters
            return img.astype(np.float32)

        # Raw Image
        h, w = int(msg.height), int(msg.width)
        enc = (msg.encoding or '').lower()
        if enc in ('16uc1', 'mono16'):
            data = np.frombuffer(msg.data, dtype=np.uint16)
            depth = data.reshape((h, int(msg.step // 2)))[:, :w].astype(np.float32) * 0.001
            return depth
        if enc in ('32fc1', '32fc'):
            data = np.frombuffer(msg.data, dtype=np.float32)
            return data.reshape((h, int(msg.step // 4)))[:, :w]
        raise ValueError('unsupported depth encoding: %s' % msg.encoding)

    def _camera_intrinsics(self, width: int, height: int):
        if self._info_msg is not None and len(self._info_msg.k) >= 9 and self._info_msg.k[0] > 0:
            k = self._info_msg.k
            return float(k[0]), float(k[4]), float(k[2]), float(k[5])
        # Fallback: approximate 60deg horizontal FOV.
        fx = width / (2.0 * math.tan(math.radians(60.0) * 0.5))
        fy = fx
        return fx, fy, width * 0.5, height * 0.5

    def _build_mask(self, bgr, depth):
        valid_depth = np.isfinite(depth) & (depth >= self.min_depth) & (depth <= self.max_depth)
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        saturation = hsv[:, :, 1]
        value = hsv[:, :, 2]

        # Industrial tools are often metallic/low-saturation. Combine depth ROI,
        # edges and color saliency so plain screws/nuts are still detected.
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 60, 140)
        color_saliency = ((saturation > 35) & (value > 35)).astype(np.uint8) * 255
        depth_mask = valid_depth.astype(np.uint8) * 255
        mask = cv2.bitwise_and(cv2.bitwise_or(edges, color_saliency), depth_mask)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        mask = cv2.dilate(mask, kernel, iterations=1)
        return mask

    def _classify_contour(self, contour, bbox, depth_m, bgr):
        x, y, w, h = bbox
        area = max(1.0, cv2.contourArea(contour))
        rect = cv2.minAreaRect(contour)
        (_, _), (rw, rh), angle = rect
        long_side = max(rw, rh, 1.0)
        short_side = max(min(rw, rh), 1.0)
        aspect = long_side / short_side
        extent = area / max(1.0, float(w * h))
        perimeter = max(1.0, cv2.arcLength(contour, True))
        circularity = 4.0 * math.pi * area / (perimeter * perimeter)

        crop = bgr[y:y + h, x:x + w]
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV) if crop.size else None
        hue = float(np.median(hsv[:, :, 0])) if hsv is not None else 0.0
        sat = float(np.median(hsv[:, :, 1])) if hsv is not None else 0.0

        cls = 'unknown'
        conf = 0.45

        if circularity > 0.55 and aspect < 1.8:
            cls = 'nut' if area > 450 else 'screw'
            conf = 0.62
        elif aspect > 4.5:
            cls = 'screwdriver' if sat > 45 and (95 <= hue <= 135) else 'roller'
            conf = 0.58
        elif aspect > 2.2:
            cls = 'wrench' if extent < 0.55 else 'roller'
            conf = 0.55
        elif area < 500:
            cls = 'screw'
            conf = 0.50

        if not np.isfinite(depth_m):
            conf *= 0.5
        return cls, conf, float(angle), float(aspect), float(circularity)

    def _tick(self) -> None:
        if cv2 is None:
            self._warn_throttled('python3-opencv is not installed')
            return
        if self._rgb_msg is None or self._depth_msg is None:
            self._warn_throttled('waiting for RGB-D images')
            return

        rgb_t = _stamp_to_float(self._rgb_msg.header.stamp)
        depth_t = _stamp_to_float(self._depth_msg.header.stamp)
        if abs(rgb_t - depth_t) > self.sync_tol:
            self._warn_throttled('RGB/depth timestamps differ by %.3fs' % abs(rgb_t - depth_t))

        try:
            bgr = self._image_to_cv(self._rgb_msg)
            depth = self._depth_to_meters(self._depth_msg)
        except Exception as exc:
            self._warn_throttled(str(exc))
            return

        h, w = bgr.shape[:2]
        if depth.shape[:2] != (h, w):
            depth = cv2.resize(depth, (w, h), interpolation=cv2.INTER_NEAREST)

        fx, fy, cx, cy = self._camera_intrinsics(w, h)
        mask = self._build_mask(bgr, depth)
        contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

        objects = []
        debug = bgr.copy()
        for contour in sorted(contours, key=cv2.contourArea, reverse=True):
            area = cv2.contourArea(contour)
            if area < self.min_area or area > self.max_area:
                continue
            x, y, bw, bh = cv2.boundingRect(contour)
            if bw < 4 or bh < 4:
                continue

            contour_mask = np.zeros(depth.shape[:2], dtype=np.uint8)
            cv2.drawContours(contour_mask, [contour], -1, 255, -1)
            roi_depth = depth[(contour_mask > 0) & np.isfinite(depth)]
            roi_depth = roi_depth[(roi_depth >= self.min_depth) & (roi_depth <= self.max_depth)]
            if roi_depth.size == 0:
                continue
            depth_m = float(np.median(roi_depth))

            moments = cv2.moments(contour)
            if abs(moments['m00']) > 1e-6:
                u = float(moments['m10'] / moments['m00'])
                v = float(moments['m01'] / moments['m00'])
            else:
                u = float(x + bw * 0.5)
                v = float(y + bh * 0.5)

            x_cam = (u - cx) * depth_m / fx
            y_cam = (v - cy) * depth_m / fy
            z_cam = depth_m
            cls, conf, angle, aspect, circularity = self._classify_contour(
                contour, (x, y, bw, bh), depth_m, bgr)

            cam_frame = self._rgb_msg.header.frame_id or self.frame_id
            world_pos = self._transform_to_world(x_cam, y_cam, z_cam, cam_frame)
            if world_pos is None:
                table_pos = [round(x_cam, 4), round(y_cam, 4), round(z_cam, 4)]
            else:
                table_pos = world_pos

            obj = {
                'id': len(objects),
                'class': cls,
                'class_cn': TOOL_CN.get(cls, cls),
                'class_id': TOOL_CLASS_ID.get(cls, -1),
                'confidence': round(conf, 3),
                'bbox_xywh': [int(x), int(y), int(bw), int(bh)],
                'center_px': [round(u, 1), round(v, 1)],
                'position_camera_m': [round(x_cam, 4), round(y_cam, 4), round(z_cam, 4)],
                'position_world_m': world_pos,
                'position_table_m': table_pos,
                'rotation_deg': round(angle, 2),
                'area_px': round(float(area), 1),
                'aspect': round(aspect, 3),
                'circularity': round(circularity, 3),
            }
            objects.append(obj)

            label = '%s %.2fm' % (obj['class_cn'], z_cam)
            cv2.rectangle(debug, (x, y), (x + bw, y + bh), (0, 220, 80), 2)
            cv2.circle(debug, (int(u), int(v)), 3, (0, 0, 255), -1)
            cv2.putText(debug, label, (x, max(14, y - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 220, 80), 1, cv2.LINE_AA)

        payload = {
            'stamp': _stamp_to_float(self._rgb_msg.header.stamp),
            'frame_id': self._rgb_msg.header.frame_id or self.frame_id,
            'world_frame': self.world_frame if self._tf_available else None,
            'source': {'rgb': self.rgb_topic, 'depth': self.depth_topic},
            'objects': objects,
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.annotations_pub.publish(msg)
        self._publish_debug(debug)

    def _publish_debug(self, bgr) -> None:
        ok, jpg = cv2.imencode('.jpg', bgr, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        if not ok:
            return
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.format = 'jpeg'
        msg.data = jpg.tobytes()
        self.debug_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RgbdToolDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
