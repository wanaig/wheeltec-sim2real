"""
locate_anything_client.py - LocateAnything (open-vocabulary) perception for WHEELTEC.

Architecture: ROS2 + (external GPU HTTP) + RGB-D (local 3D back-projection).

This node is the Jetson side of the distributed pipeline:
  - subscribes the Astra S RGB / depth / camera_info (same as rgbd_tool_detector)
  - subscribes /locate/query (std_msgs/String) carrying the natural-language
    target prompt (e.g. "蓝色方形工件") published by the browser LLM agent
  - JPEG-encodes the latest RGB frame + the prompt, HTTP POSTs it to the
    external LocateAnything inference server (server.py on the RTX host)
  - receives 2D bboxes back, then performs 2D->3D back-projection LOCALLY
    (depth ROI median + camera intrinsics + TF base_link<-camera) exactly
    like rgbd_tool_detector, because the depth stream + TF tree live here
  - publishes /industrial_tools/annotations in the SAME format as the YOLO
    detector, so the frontend (AgentPanel._onToolAnnotations -> MockAgent
    .setRealAnnotations) and the perceive tool work unchanged

Why local 3D: streaming depth + live TF to the external host would be far
costlier than sending one JPEG and receiving a few bboxes. The heavy model
runs off-board; the cheap geometry runs on-board.

Topic contract (identical to rgbd_tool_detector + one input):
  in : /camera/color/image_raw/compressed  (CompressedImage)
       /camera/depth/image_raw/compressed  (CompressedImage)
       /camera/color/camera_info           (CameraInfo)
       /joint_states                       (JointState, for fallback FK)
       /locate/query                       (std_msgs/String, NL prompt)
  out: /industrial_tools/annotations             (std_msgs/String JSON)
       /industrial_tools/debug_image/compressed  (CompressedImage)
       /industrial_tools/depth_viz/compressed    (CompressedImage)
"""

import base64
import json
import sys
import urllib.request
import urllib.error

for _p in list(sys.path):
    if 'noetic' in _p or 'ros1' in _p:
        sys.path.remove(_p)

from std_msgs.msg import String

try:
    import cv2
except ImportError:
    cv2 = None

from .rgbd_tool_detector import RgbdToolDetector


class LocateAnythingClient(RgbdToolDetector):
    """Jetson-side perception node backed by an external LocateAnything server.

    Inherits all image decoding, depth->3D, TF/FK transform and publishing
    logic from RgbdToolDetector. Only the detection step is replaced: instead
    of running YOLO locally it POSTs the frame + prompt to the HTTP server.
    """

    def __init__(self) -> None:
        # super().__init__ reuses all subscriptions/publishers/TF/timer and
        # calls _load_yolo (overridden below to a no-op).
        super().__init__()

        self.declare_parameter('server_url', 'http://192.168.0.100:8765')
        self.declare_parameter('http_timeout_s', 3.0)
        self.declare_parameter('default_prompt', '')
        self.declare_parameter('query_topic', '/locate/query')

        self.server_url = str(self.get_parameter('server_url').value).rstrip('/')
        self.http_timeout = float(self.get_parameter('http_timeout_s').value)
        self._current_query = str(self.get_parameter('default_prompt').value)
        query_topic = str(self.get_parameter('query_topic').value)

        self.create_subscription(String, query_topic, self._on_query, 10)

        self.get_logger().info(
            'LocateAnything HTTP client ready: server=%s timeout=%.1fs '
            'query_topic=%s default_prompt=%r' % (
                self.server_url, self.http_timeout, query_topic, self._current_query))

    # ------------------------------------------------------------------
    # Override: no local YOLO model. Backend marker reused by parent _tick.
    # ------------------------------------------------------------------
    def _load_yolo(self) -> None:
        self._yolo_model = None
        self._yolo_backend = 'locate_anything_http'
        self.get_logger().info(
            'LocateAnything HTTP backend selected (no local YOLO model loaded)')

    # ------------------------------------------------------------------
    # Override: detect by calling the external server instead of YOLO.
    # Returns the SAME tuple list shape as RgbdToolDetector._yolo_detect:
    #   [(x1, y1, x2, y2, conf, cls_name, cls_id), ...]
    # so the parent _tick (depth ROI -> 3D -> TF -> publish) is unchanged.
    # ------------------------------------------------------------------
    def _yolo_detect(self, bgr):
        prompt = (self._current_query or '').strip()
        if not prompt:
            return []
        if cv2 is None:
            self._warn_throttled('python3-opencv is not installed')
            return []

        ok, jpg = cv2.imencode('.jpg', bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if not ok:
            self._warn_throttled('JPEG encode failed')
            return []

        body = json.dumps({
            'image_base64': base64.b64encode(jpg.tobytes()).decode('ascii'),
            'prompt': prompt,
            'max_boxes': 50,
        }).encode('utf-8')
        req = urllib.request.Request(
            self.server_url + '/detect', data=body,
            headers={'Content-Type': 'application/json'}, method='POST')

        try:
            with urllib.request.urlopen(req, timeout=self.http_timeout) as resp:
                data = json.loads(resp.read().decode('utf-8'))
        except urllib.error.URLError as exc:
            self._warn_throttled('LocateAnything server unreachable: %s' % exc)
            return []
        except Exception as exc:
            self._warn_throttled('LocateAnything server error: %s' % exc)
            return []

        boxes = data.get('boxes', []) if isinstance(data, dict) else []
        detections = []
        for b in boxes:
            bbox = b.get('bbox', [0, 0, 0, 0])
            if len(bbox) != 4:
                continue
            conf = float(b.get('confidence', 0.0))
            cls_name = str(b.get('class', prompt))
            detections.append((
                int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3]),
                conf, cls_name, 0))
        self.get_logger().debug(
            'LA detect: prompt=%r boxes=%d' % (prompt[:40], len(detections)))
        return detections

    # ------------------------------------------------------------------
    # Natural-language prompt input (from browser LLM agent via rosbridge)
    # ------------------------------------------------------------------
    def _on_query(self, msg: String) -> None:
        self._current_query = (msg.data or '').strip()
        self.get_logger().info('locate query set: %r' % self._current_query[:80])


def main(args=None):
    import rclpy
    rclpy.init(args=args)
    node = LocateAnythingClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
