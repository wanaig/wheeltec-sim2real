"""
rgbd_tool_detector.py - YOLO + RGB-D industrial perception for WHEELTEC robot.

Architecture: ROS2 + OpenCV + YOLO + RGB-D

Pipeline:
  1. Subscribe to color (compressed) + depth (compressed) + camera_info from Astra S
  2. YOLO inference on color frame -> bounding boxes + class + confidence
  3. For each detection: depth ROI -> median depth -> 3D point in camera frame
  4. TF transform: camera optical frame -> base_link (world coordinates)
  5. Publish JSON annotations + debug image (with YOLO bboxes + world coords)

YOLO backend (auto-detect at startup):
  - ultralytics (YOLOv8) if `pip install ultralytics` is available
  - YOLOv5 via torch.hub, local path ~/yolov5-pytorch/ (preinstalled on robot)
  - Falls back to torch.hub from GitHub if local path not found

Topics published:
  /industrial_tools/annotations             std_msgs/String (JSON)
  /industrial_tools/debug_image/compressed  sensor_msgs/CompressedImage

JSON format (per object):
  {
    "id": 0,
    "class": "bottle",           // YOLO class name
    "class_cn": "瓶子",           // Chinese name
    "class_id": 39,              // COCO class ID
    "confidence": 0.87,
    "bbox_xywh": [100, 200, 80, 120],
    "center_px": [140.0, 260.0],
    "position_camera_m": [0.12, -0.05, 0.45],
    "position_world_m": [0.35, 0.02, 0.08],
    "depth_m": 0.45
  }
"""

import json
import math
import os
import sys
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
except ImportError:
    cv2 = None


# COCO 80-class names (YOLOv5/v8 default)
COCO_NAMES = [
    'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck',
    'boat', 'traffic light', 'fire hydrant', 'stop sign', 'parking meter', 'bench',
    'bird', 'cat', 'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra',
    'giraffe', 'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee',
    'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove',
    'skateboard', 'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup',
    'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple', 'sandwich', 'orange',
    'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch',
    'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse',
    'remote', 'keyboard', 'cell phone', 'microwave', 'oven', 'toaster', 'sink',
    'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear',
    'hair drier', 'toothbrush',
]

COCO_CN = {
    'person': '人', 'bicycle': '自行车', 'car': '汽车', 'motorcycle': '摩托车',
    'airplane': '飞机', 'bus': '公交车', 'train': '火车', 'truck': '卡车',
    'boat': '船', 'traffic light': '交通灯', 'fire hydrant': '消火栓',
    'stop sign': '停车标志', 'parking meter': '停车计时器', 'bench': '长椅',
    'bird': '鸟', 'cat': '猫', 'dog': '狗', 'horse': '马', 'sheep': '羊',
    'cow': '牛', 'elephant': '大象', 'bear': '熊', 'zebra': '斑马',
    'giraffe': '长颈鹿', 'backpack': '背包', 'umbrella': '雨伞',
    'handbag': '手提包', 'tie': '领带', 'suitcase': '行李箱', 'frisbee': '飞盘',
    'skis': '滑雪板', 'snowboard': '单板滑雪', 'sports ball': '运动球',
    'kite': '风筝', 'baseball bat': '棒球棒', 'baseball glove': '棒球手套',
    'skateboard': '滑板', 'surfboard': '冲浪板', 'tennis racket': '网球拍',
    'bottle': '瓶子', 'wine glass': '酒杯', 'cup': '杯子', 'fork': '叉子',
    'knife': '刀', 'spoon': '勺子', 'bowl': '碗', 'banana': '香蕉',
    'apple': '苹果', 'sandwich': '三明治', 'orange': '橙子', 'broccoli': '西兰花',
    'carrot': '胡萝卜', 'hot dog': '热狗', 'pizza': '披萨', 'donut': '甜甜圈',
    'cake': '蛋糕', 'chair': '椅子', 'couch': '沙发', 'potted plant': '盆栽',
    'bed': '床', 'dining table': '餐桌', 'toilet': '马桶', 'tv': '电视',
    'laptop': '笔记本电脑', 'mouse': '鼠标', 'remote': '遥控器',
    'keyboard': '键盘', 'cell phone': '手机', 'microwave': '微波炉',
    'oven': '烤箱', 'toaster': '烤面包机', 'sink': '水槽',
    'refrigerator': '冰箱', 'book': '书', 'clock': '时钟', 'vase': '花瓶',
    'scissors': '剪刀', 'teddy bear': '泰迪熊', 'hair drier': '吹风机',
    'toothbrush': '牙刷',
}

# Industrial-relevant COCO classes (for frontend filtering / highlighting)
INDUSTRIAL_CLASSES = {
    'bottle', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'scissors',
    'book', 'clock', 'vase', 'remote', 'cell phone', 'laptop',
    'mouse', 'keyboard', 'tv',
}

# COCO → 工业工具类映射 (供前端 createToolMesh 匹配)
# 当 YOLO 使用 COCO 预训练模型时, 将相近的 COCO 类映射到工业工具类,
# 使前端能用 createToolMesh 渲染对应的工业工具 3D 模型
COCO_TO_INDUSTRIAL = {
    'scissors': 'screwdriver',   # 长条形工具
    'knife': 'wrench',           # 扁平长条
    'fork': 'screwdriver',       # 长条形
    'spoon': 'screwdriver',
    'bottle': 'roller',          # 圆柱形
    'sports ball': 'nut',        # 圆形
    'orange': 'nut',
    'apple': 'nut',
    'banana': 'roller',
    'bowl': 'roller',
    'cup': 'nut',
    'clock': 'nut',
    'vase': 'roller',
    'book': 'wrench',
    'remote': 'wrench',
    'cell phone': 'wrench',
}


def _stamp_to_float(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class RgbdToolDetector(Node):
    def __init__(self) -> None:
        super().__init__('rgbd_tool_detector')

        # -- Topic parameters --
        self.declare_parameter('rgb_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('annotations_topic', '/industrial_tools/annotations')
        self.declare_parameter('debug_image_topic', '/industrial_tools/debug_image/compressed')
        self.declare_parameter('frame_id', 'rgbd_camera')
        self.declare_parameter('world_frame', 'base_link')

        # -- Transport parameters --
        self.declare_parameter('depth_compressed', True)
        self.declare_parameter('rgb_compressed', True)

        # -- Detection parameters --
        self.declare_parameter('detect_hz', 5.0)
        self.declare_parameter('min_depth_m', 0.12)
        self.declare_parameter('max_depth_m', 2.00)
        self.declare_parameter('depth_sync_tolerance_s', 5.0)
        self.declare_parameter('jpeg_quality', 70)

        # -- YOLO parameters --
        self.declare_parameter('yolo_model', 'yolov8n.pt')
        self.declare_parameter('yolo_conf', 0.45)
        self.declare_parameter('yolo_device', 'cuda:0')
        self.declare_parameter('yolo_imgsz', 640)
        self.declare_parameter('yolo_local_repo', '/home/wheeltec/yolov5-pytorch')

        # Read parameters
        self.rgb_topic = str(self.get_parameter('rgb_topic').value)
        self.depth_topic = str(self.get_parameter('depth_topic').value)
        self.info_topic = str(self.get_parameter('camera_info_topic').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.world_frame = str(self.get_parameter('world_frame').value)
        self.min_depth = float(self.get_parameter('min_depth_m').value)
        self.max_depth = float(self.get_parameter('max_depth_m').value)
        self.sync_tol = float(self.get_parameter('depth_sync_tolerance_s').value)
        self.jpeg_quality = max(10, min(95, int(self.get_parameter('jpeg_quality').value)))
        self.yolo_conf = float(self.get_parameter('yolo_conf').value)
        self.yolo_device = str(self.get_parameter('yolo_device').value)
        self.yolo_imgsz = int(self.get_parameter('yolo_imgsz').value)

        # State
        self._rgb_msg: Optional[Image] = None
        self._depth_msg: Optional[Image] = None
        self._info_msg: Optional[CameraInfo] = None
        self._last_warn = 0.0
        self._yolo_model = None
        self._yolo_backend = None

        # Subscriptions (RELIABLE QoS to match dynamic_bridge publisher)
        self.rgb_compressed = bool(self.get_parameter('rgb_compressed').value)
        if self.rgb_compressed:
            self.create_subscription(CompressedImage, self.rgb_topic + '/compressed', self._on_rgb, 10)
        else:
            self.create_subscription(Image, self.rgb_topic, self._on_rgb, 10)

        self.depth_compressed = bool(self.get_parameter('depth_compressed').value)
        if self.depth_compressed:
            self.create_subscription(CompressedImage, self.depth_topic + '/compressed', self._on_depth, 10)
        else:
            self.create_subscription(Image, self.depth_topic, self._on_depth, 10)

        self.create_subscription(CameraInfo, self.info_topic, self._on_info, 10)

        # Publishers
        self.annotations_pub = self.create_publisher(
            String, str(self.get_parameter('annotations_topic').value), 10)
        self.debug_pub = self.create_publisher(
            CompressedImage, str(self.get_parameter('debug_image_topic').value), 5)
        # 8-bit depth visualization for browser point cloud generation
        self.depth_viz_pub = self.create_publisher(
            CompressedImage, '/industrial_tools/depth_viz/compressed', 5)

        # TF
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_available = False

        # Load YOLO model
        self._load_yolo()

        # Detection timer
        period = 1.0 / max(0.5, float(self.get_parameter('detect_hz').value))
        self.create_timer(period, self._tick)

        self.get_logger().info(
            'YOLO RGB-D detector started: rgb=%s%s depth=%s%s info=%s '
            'world_frame=%s yolo=%s backend=%s device=%s' % (
                self.rgb_topic, '/compressed' if self.rgb_compressed else '',
                self.depth_topic, '/compressed' if self.depth_compressed else '',
                self.info_topic, self.world_frame,
                str(self.get_parameter('yolo_model').value),
                self._yolo_backend, self.yolo_device))

    # ------------------------------------------------------------------
    # YOLO model loading
    # ------------------------------------------------------------------
    def _load_yolo(self) -> None:
        model_path = str(self.get_parameter('yolo_model').value)
        local_repo = str(self.get_parameter('yolo_local_repo').value)

        # Resolve bare filename against package-local models/ directory
        # (avoids downloading from GitHub on the robot when pre-shipped)
        if (os.path.sep not in model_path and
                not os.path.isabs(model_path) and
                not os.path.isfile(model_path)):
            pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            # ament install layout: share/<pkg>/models
            for candidate in (
                os.path.join(pkg_dir, 'models', model_path),            # source tree
                os.path.join(pkg_dir, '..', 'share',
                             'wheeltec_sim2real_bridge',
                             'models', model_path),                     # installed
            ):
                candidate = os.path.normpath(candidate)
                if os.path.isfile(candidate):
                    model_path = candidate
                    self.get_logger().info(
                        'Resolved YOLO model to package models/: %s' % model_path)
                    break

        is_yolov8 = 'yolov8' in model_path.lower()

        # Method 1: ultralytics (YOLOv8) — required for v8 models
        try:
            from ultralytics import YOLO
            self._yolo_model = YOLO(model_path)
            self._yolo_model.to(self.yolo_device)
            self._yolo_backend = 'ultralytics'
            self.get_logger().info(
                'YOLO loaded (ultralytics/YOLOv8): %s on %s' % (model_path, self.yolo_device))
            return
        except ImportError:
            if is_yolov8:
                self.get_logger().warn(
                    'YOLOv8 model requested but ultralytics not installed. '
                    'Run: pip3 install ultralytics. Falling back to yolov5s.pt')
                model_path = 'yolov5s.pt'
            else:
                pass
        except Exception as exc:
            self.get_logger().warn('ultralytics load failed: %s' % exc)
            if is_yolov8:
                model_path = 'yolov5s.pt'

        # Method 2: YOLOv5 via torch.hub (local repo first, then GitHub)
        try:
            import torch
            if os.path.isdir(local_repo):
                self.get_logger().info('Loading YOLOv5 from local repo: %s' % local_repo)
                self._yolo_model = torch.hub.load(
                    local_repo, 'custom', path=model_path, source='local')
            else:
                self.get_logger().info('Loading YOLOv5 from GitHub: %s' % model_path)
                self._yolo_model = torch.hub.load(
                    'ultralytics/yolov5', 'custom', path=model_path)
            self._yolo_model.conf = self.yolo_conf
            self._yolo_backend = 'yolov5'
            self.get_logger().info(
                'YOLO loaded (yolov5/torch.hub): %s on %s' % (model_path, self.yolo_device))
            return
        except Exception as exc:
            self.get_logger().error('Failed to load YOLO model: %s' % exc)
            self._yolo_model = None
            self._yolo_backend = None

    # ------------------------------------------------------------------
    # YOLO inference
    # ------------------------------------------------------------------
    def _yolo_detect(self, bgr):
        """Run YOLO inference. Returns list of (x1, y1, x2, y2, conf, cls_name, cls_id)."""
        if self._yolo_model is None:
            return []

        if self._yolo_backend == 'ultralytics':
            results = self._yolo_model(
                bgr, verbose=False, conf=self.yolo_conf,
                device=self.yolo_device, imgsz=self.yolo_imgsz)
            detections = []
            for r in results:
                if r.boxes is None:
                    continue
                for box in r.boxes:
                    xyxy = box.xyxy[0].cpu().numpy()
                    x1, y1, x2, y2 = int(xyxy[0]), int(xyxy[1]), int(xyxy[2]), int(xyxy[3])
                    conf = float(box.conf[0].cpu().numpy())
                    cls_id = int(box.cls[0].cpu().numpy())
                    cls_name = r.names.get(cls_id, str(cls_id))
                    detections.append((x1, y1, x2, y2, conf, cls_name, cls_id))
            return detections

        if self._yolo_backend == 'yolov5':
            results = self._yolo_model(bgr, size=self.yolo_imgsz)
            df = results.pandas().xyxy[0]
            detections = []
            for _, row in df.iterrows():
                conf = float(row['confidence'])
                if conf < self.yolo_conf:
                    continue
                x1 = int(row['xmin'])
                y1 = int(row['ymin'])
                x2 = int(row['xmax'])
                y2 = int(row['ymax'])
                cls_id = int(row['class'])
                cls_name = str(row['name'])
                detections.append((x1, y1, x2, y2, conf, cls_name, cls_id))
            return detections

        return []

    # ------------------------------------------------------------------
    # TF transform
    # ------------------------------------------------------------------
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

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
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

    # ------------------------------------------------------------------
    # Image decoding
    # ------------------------------------------------------------------
    def _image_to_cv(self, msg):
        if isinstance(msg, CompressedImage):
            buf = np.frombuffer(msg.data, dtype=np.uint8)
            img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
            if img is None:
                raise ValueError('failed to decode compressed RGB image')
            return img

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
        if isinstance(msg, CompressedImage):
            buf = np.frombuffer(msg.data, dtype=np.uint8)
            img = cv2.imdecode(buf, cv2.IMREAD_UNCHANGED)
            if img is None:
                raise ValueError('failed to decode compressed depth image')
            if img.dtype == np.uint16:
                return img.astype(np.float32) * 0.001
            return img.astype(np.float32)

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
            fx, fy, cx, cy = float(k[0]), float(k[4]), float(k[2]), float(k[5])
            info_w = self._info_msg.width
            info_h = self._info_msg.height
            if info_w > 0 and info_h > 0 and (info_w != width or info_h != height):
                sx = width / info_w
                sy = height / info_h
                fx *= sx; fy *= sy; cx *= sx; cy *= sy
            return fx, fy, cx, cy
        fx = width / (2.0 * math.tan(math.radians(60.0) * 0.5))
        fy = fx
        return fx, fy, width * 0.5, height * 0.5

    # ------------------------------------------------------------------
    # Main detection tick
    # ------------------------------------------------------------------
    def _tick(self) -> None:
        if cv2 is None:
            self._warn_throttled('python3-opencv is not installed')
            return
        if self._yolo_model is None:
            self._warn_throttled('YOLO model not loaded')
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

        # YOLO detection
        detections = self._yolo_detect(bgr)

        objects = []
        debug = bgr.copy()
        for (x1, y1, x2, y2, conf, cls_name, cls_id) in detections:
            bw = x2 - x1
            bh = y2 - y1
            if bw < 4 or bh < 4:
                continue

            # Bounding box center
            u = (x1 + x2) * 0.5
            v = (y1 + y2) * 0.5

            # Depth ROI: central 50% of bbox (avoid edge noise)
            roi_x1 = max(0, x1 + bw // 4)
            roi_y1 = max(0, y1 + bh // 4)
            roi_x2 = min(w, x2 - bw // 4)
            roi_y2 = min(h, y2 - bh // 4)
            roi_depth = depth[roi_y1:roi_y2, roi_x1:roi_x2]
            valid = roi_depth[(roi_depth >= self.min_depth) & (roi_depth <= self.max_depth)]

            if valid.size > 0:
                depth_m = float(np.median(valid))
            else:
                depth_m = float('nan')

            # 3D point in camera optical frame (X right, Y down, Z forward)
            if np.isfinite(depth_m):
                x_cam = (u - cx) * depth_m / fx
                y_cam = (v - cy) * depth_m / fy
                z_cam = depth_m
            else:
                x_cam = y_cam = z_cam = float('nan')

            # Transform to world frame (base_link)
            cam_frame = self._rgb_msg.header.frame_id or self.frame_id
            if np.isfinite(depth_m):
                world_pos = self._transform_to_world(x_cam, y_cam, z_cam, cam_frame)
            else:
                world_pos = None

            if world_pos is None:
                cam_pos = [
                    round(x_cam, 4) if np.isfinite(x_cam) else None,
                    round(y_cam, 4) if np.isfinite(y_cam) else None,
                    round(z_cam, 4) if np.isfinite(z_cam) else None,
                ]
                table_pos = cam_pos
            else:
                table_pos = world_pos

            cls_cn = COCO_CN.get(cls_name, cls_name)
            is_industrial = cls_name in INDUSTRIAL_CLASSES
            industrial_class = COCO_TO_INDUSTRIAL.get(cls_name, cls_name)

            obj = {
                'id': len(objects),
                'class': industrial_class if industrial_class != cls_name else cls_name,
                'yolo_class': cls_name,
                'class_cn': cls_cn,
                'class_id': cls_id,
                'confidence': round(conf, 3),
                'bbox_xywh': [int(x1), int(y1), int(bw), int(bh)],
                'center_px': [round(u, 1), round(v, 1)],
                'position_camera_m': [
                    round(x_cam, 4) if np.isfinite(x_cam) else None,
                    round(y_cam, 4) if np.isfinite(y_cam) else None,
                    round(z_cam, 4) if np.isfinite(z_cam) else None,
                ],
                'position_world_m': world_pos,
                'position_table_m': table_pos,
                'depth_m': round(depth_m, 4) if np.isfinite(depth_m) else None,
                'industrial': is_industrial,
            }
            objects.append(obj)

            # Draw on debug image
            color = (0, 220, 80) if is_industrial else (80, 80, 220)
            depth_str = '%.2fm' % depth_m if np.isfinite(depth_m) else 'N/A'
            label = '%s %.0f%% %s' % (cls_cn, conf * 100, depth_str)
            cv2.rectangle(debug, (x1, y1), (x2, y2), color, 2)
            cv2.circle(debug, (int(u), int(v)), 3, (0, 0, 255), -1)

            # Label background
            (tw, th), _ = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
            cv2.rectangle(debug, (x1, max(0, y1 - th - 6)),
                          (x1 + tw + 4, max(th + 2, y1 - 2)), color, -1)
            cv2.putText(debug, label, (x1 + 2, max(th, y1 - 4)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1, cv2.LINE_AA)

            # World coordinates text
            if world_pos is not None:
                coord_label = '(%.2f, %.2f, %.2f)' % (
                    world_pos[0], world_pos[1], world_pos[2])
                cv2.putText(debug, coord_label, (x1, y2 + 14),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35,
                            (0, 220, 80), 1, cv2.LINE_AA)

        payload = {
            'stamp': _stamp_to_float(self._rgb_msg.header.stamp),
            'frame_id': self._rgb_msg.header.frame_id or self.frame_id,
            'world_frame': self.world_frame if self._tf_available else None,
            'detector': 'yolo',
            'yolo_backend': self._yolo_backend,
            'source': {
                'rgb': self.rgb_topic,
                'depth': self.depth_topic,
                'yolo_model': str(self.get_parameter('yolo_model').value),
            },
            'objects': objects,
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.annotations_pub.publish(msg)
        self._publish_debug(debug)
        self._publish_depth_viz(depth)

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

    def _publish_depth_viz(self, depth) -> None:
        """Publish 8-bit depth JPEG for browser point cloud generation.
        depth_m = pixel_value / 100  (1cm resolution, 0-2.55m range)
        0 = invalid (NaN or out of range)
        """
        valid = np.isfinite(depth) & (depth >= self.min_depth) & (depth <= self.max_depth)
        depth_8 = np.zeros(depth.shape, dtype=np.uint8)
        depth_8[valid] = np.clip(depth[valid] * 100, 1, 255).astype(np.uint8)
        ok, jpg = cv2.imencode('.jpg', depth_8, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if not ok:
            return
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.format = 'jpeg'
        msg.data = jpg.tobytes()
        self.depth_viz_pub.publish(msg)


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
