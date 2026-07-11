#!/usr/bin/env python3
"""
DEOS Camera Stream - MJPEG sunucu
Docker ICINDE calistirilir.

Kullanim:
  docker exec -it epic_torvalds bash
  source /ros2_ws/install/setup.bash
  python3 /ros2_ws/src/../../../ops/scripts/camera_stream_server.py

  ya da konteyner icindeyken:
  python3 /ros2_ws/src/camera_stream_server.py   (kopyalanmissa)

Erisim (dis agdan):
  http://<raspi-ip>:8080/stream

Ortam degiskenleri:
  DEOS_CAM_TOPIC  - ROS topic (varsayilan: /deos/sensors/camera/color)
  DEOS_CAM_PORT   - port (varsayilan: 8080)
  DEOS_CAM_FPS    - maks fps (varsayilan: 30)
  DEOS_CAM_QUALITY- JPEG kalitesi 1-100 (varsayilan: 75)
"""

import os
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

TOPIC   = os.environ.get("DEOS_CAM_TOPIC",   "/deos/sensors/camera/color/image_raw")
PORT    = int(os.environ.get("DEOS_CAM_PORT",    "8080"))
FPS     = int(os.environ.get("DEOS_CAM_FPS",     "30"))
QUALITY = int(os.environ.get("DEOS_CAM_QUALITY", "75"))

_latest_jpg: bytes = b""
_lock = threading.Lock()
_frame_count = 0


class CameraSubscriberNode(Node):
    def __init__(self):
        super().__init__("deos_camera_stream_server")
        self._bridge = CvBridge()
        self._min_interval = 1.0 / FPS
        self._last_t = 0.0
        self.create_subscription(Image, TOPIC, self._cb, 1)
        self.get_logger().info(f"Subscribed to {TOPIC}, serving MJPEG on :{PORT}")

    def _cb(self, msg: Image) -> None:
        global _latest_jpg, _frame_count
        now = time.monotonic()
        if now - self._last_t < self._min_interval:
            return
        self._last_t = now
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, QUALITY])
            if ok:
                with _lock:
                    _latest_jpg = bytes(buf)
                    _frame_count += 1
        except Exception as exc:
            self.get_logger().warning(f"Frame encode error: {exc}")


class MjpegHandler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args) -> None:
        pass  # access log bastir

    def do_GET(self) -> None:
        if self.path.startswith("/stream"):
            self._stream()
        elif self.path == "/snapshot":
            self._snapshot()
        elif self.path == "/health":
            body = b"OK"
            self.send_response(200)
            self.send_header("Content-Type", "text/plain")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        else:
            self.send_response(404)
            self.end_headers()

    def _stream(self) -> None:
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        interval = 1.0 / FPS
        try:
            while True:
                with _lock:
                    jpg = _latest_jpg
                if jpg:
                    self.wfile.write(
                        b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n"
                    )
                    self.wfile.flush()
                time.sleep(interval)
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass

    def _snapshot(self) -> None:
        with _lock:
            jpg = _latest_jpg
        if not jpg:
            self.send_response(503)
            self.end_headers()
            return
        self.send_response(200)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Content-Length", str(len(jpg)))
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(jpg)


def main() -> None:
    rclpy.init()
    node = CameraSubscriberNode()

    server = HTTPServer(("0.0.0.0", PORT), MjpegHandler)
    t = threading.Thread(target=server.serve_forever, daemon=True)
    t.start()
    print(f"DEOS Camera Stream baslatildi")
    print(f"  MJPEG stream  : http://0.0.0.0:{PORT}/stream")
    print(f"  Snapshot      : http://0.0.0.0:{PORT}/snapshot")
    print(f"  Topic         : {TOPIC}")
    print(f"  Max FPS       : {FPS}  |  JPEG quality: {QUALITY}")
    print("Durdurmak icin Ctrl+C")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
