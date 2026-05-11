import json
import math
import time

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


class StereoDetectorNode(Node):
    def __init__(self):
        super().__init__("stereo_detector_node")

        self.declare_parameter("image_width", 640)
        self.declare_parameter("image_height", 480)
        self.declare_parameter("focal_length_px", 320.0)
        self.declare_parameter("depth_timeout_s", 0.5)

        self._w = int(self.get_parameter("image_width").value)
        self._h = int(self.get_parameter("image_height").value)
        self._focal = float(self.get_parameter("focal_length_px").value)
        self._depth_timeout = float(self.get_parameter("depth_timeout_s").value)

        self._bridge = CvBridge()
        self._depth_img: np.ndarray | None = None
        self._depth_stamp: float = 0.0

        self.create_subscription(Image, "/camera/color/image_raw", self._rgb_cb, 10)
        self.create_subscription(Image, "/camera/depth/image_raw", self._depth_cb, 10)
        self._pub = self.create_publisher(String, "/perception/stereo_detections", 10)

        self.get_logger().info("stereo_detector_node ready (YOLO stub active)")

    def _depth_cb(self, msg: Image) -> None:
        try:
            raw = np.frombuffer(bytes(msg.data), dtype=np.uint16)
            self._depth_img = raw.reshape(msg.height, msg.width).astype(np.float32) / 1000.0
            self._depth_stamp = time.monotonic()
        except Exception as e:
            self.get_logger().error(f"depth convert: {e}")

    def _rgb_cb(self, msg: Image) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"rgb convert: {e}")
            return

        dets = self._yolo_detect(frame)
        result = []
        for d in dets:
            x1, y1, x2, y2 = d["bbox"]
            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)
            dist = self._sample_depth(cx, cy)
            lat = self._lateral_m(cx, dist) if dist is not None else None
            result.append(
                {
                    "class_name": d["class_name"],
                    "confidence": float(d["confidence"]),
                    "bbox_px": [float(x1), float(y1), float(x2), float(y2)],
                    "distance_m": dist,
                    "lateral_m": lat,
                }
            )

        self._pub.publish(String(data=json.dumps(result)))

    def _yolo_detect(self, frame) -> list[dict]:
        # YOLO entegrasyon noktası: gerçek model eklendiğinde yalnızca burası değişmeli.
        # Format: [{'class_name': str, 'confidence': float, 'bbox': (x1,y1,x2,y2)}, ...]
        return []

    def _sample_depth(self, cx: int, cy: int) -> float | None:
        if self._depth_img is None:
            return None
        if time.monotonic() - self._depth_stamp > self._depth_timeout:
            return None

        h, w = self._depth_img.shape
        cy_c = max(0, min(cy, h - 1))
        cx_c = max(0, min(cx, w - 1))
        d = float(self._depth_img[cy_c, cx_c])
        return d if d > 0.05 else None

    def _lateral_m(self, cx: int, dist: float) -> float:
        du = cx - (self._w / 2.0)
        return -(du * dist) / self._focal


def main(args=None):
    rclpy.init(args=args)
    node = StereoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

