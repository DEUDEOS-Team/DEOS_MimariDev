import json
import math
import time
from typing import Optional

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from deos_algorithms.ros_topic_layout import build_deos_topics

from deos_logging.logger import DeosLogger

# ---------------------------------------------------------------------------
# DEOS sınıf adı haritası (COCO indeksi → DEOS dahili adı)
# Özel model kullanılıyorsa bu eşleme devreye girmez; class_names doğrudan kullanılır.
# ---------------------------------------------------------------------------
COCO_TO_DEOS: dict[int, str] = {
    0:  "pedestrian",   # person
    1:  "bicycle",
    2:  "vehicle",      # car
    3:  "vehicle",      # motorcycle
    5:  "vehicle",      # bus
    7:  "vehicle",      # truck
    9:  "traffic_light_red",    # traffic light (renk sonradan ayrıştırılır)
    11: "stop sign",    # stop sign → dur tabelası (traffic_sign_logic alias'ı)
    39: "bottle",
    56: "chair",
    # Yaya geçidi tabelası, koni, bariyer özel modelden gelir
}

# DEOS tabela sınıfı alias → traffic_sign_logic'in tanıdığı canonical isim
SIGN_CANONICAL: dict[str, str] = {
    "stop sign":        "dur tabelası",
    "dur":              "dur tabelası",
    "dur tabelasi":     "dur tabelası",
    "girilmez":         "girilmez",
    "yaya gecidi":      "yaya gecidi",
    "yaya geçidi":      "yaya gecidi",
    "durak":            "durak",
    "park":             "park",
    "park yapilmaz":    "park yapilmaz",
    "isikli isaret":    "isikli isaret cihazi",
    "tunel":            "tunel",
    "tünel":            "tunel",
}

# Trafik ışığı renk etiketleri
LIGHT_CLASSES = {
    "traffic_light_red", "traffic_light_yellow", "traffic_light_green",
    "red", "yellow", "green",
    "kirmizi", "sari", "yesil",
    "kırmızı", "sarı", "yeşil",
}


def _map_coco(class_id: int, class_name: str) -> str:
    """COCO class id/name → DEOS canonical class name."""
    return COCO_TO_DEOS.get(class_id, class_name.lower())


class StereoDetectorNode(Node):
    def __init__(self):
        super().__init__("stereo_detector_node")
        self.logger = DeosLogger(self.get_logger(), "stereo_detector_node")

        self.declare_parameter("image_width", 640)
        self.declare_parameter("image_height", 480)
        self.declare_parameter("focal_length_px", 320.0)
        self.declare_parameter("depth_timeout_s", 0.5)
        # Model parametreleri
        self.declare_parameter("model_path", "")
        self.declare_parameter("model_backend", "ultralytics")  # "ultralytics" | "hailo"
        self.declare_parameter("conf_threshold", 0.35)
        # COCO modelinde sınıf adlarını DEOS'a dönüştür (özel model: False)
        self.declare_parameter("use_coco_mapping", True)
        self.declare_parameter("deos_root", "/deos")
        _T = build_deos_topics(str(self.get_parameter("deos_root").value))
        self.declare_parameter("rgb_topic", _T["sensors_camera_color"])
        self.declare_parameter("depth_topic", _T["sensors_camera_depth"])
        self.declare_parameter("stereo_detections_topic", _T["perception_stereo_detections"])

        self._w = int(self.get_parameter("image_width").value)
        self._h = int(self.get_parameter("image_height").value)
        self._focal = float(self.get_parameter("focal_length_px").value)
        self._depth_timeout = float(self.get_parameter("depth_timeout_s").value)
        self._conf_thr = float(self.get_parameter("conf_threshold").value)
        self._use_coco_map = bool(self.get_parameter("use_coco_mapping").value)

        self._bridge = CvBridge()
        self._depth_img: Optional[np.ndarray] = None
        self._depth_stamp: float = 0.0

        # Model yükleme
        self._model = None
        self._backend: str = ""
        self._hailo_runner = None
        model_path = str(self.get_parameter("model_path").value)
        backend = str(self.get_parameter("model_backend").value)
        if model_path:
            self._load_model(model_path, backend)
        else:
            self.logger.warning(
                "model_path parametresi boş — YOLO devre dışı. "
                "Başlatmak için: ros2 run ... --ros-args -p model_path:=/path/to/model.pt"
            )

        self.create_subscription(Image, str(self.get_parameter("rgb_topic").value), self._rgb_cb, 10)
        self.create_subscription(Image, str(self.get_parameter("depth_topic").value), self._depth_cb, 10)
        self._pub = self.create_publisher(String, str(self.get_parameter("stereo_detections_topic").value), 10)

        self.logger.info(
            f"stereo_detector_node ready — backend={self._backend or 'none'}, "
            f"model={'loaded' if self._model or self._hailo_runner else 'NOT loaded'}"
        )

    # ------------------------------------------------------------------
    # Model yükleme
    # ------------------------------------------------------------------

    def _load_model(self, path: str, backend: str) -> None:
        if backend == "hailo":
            self._load_hailo(path)
            if self._hailo_runner is None:
                self.logger.warning("Hailo yüklenemedi, ultralytics deneniyor...")
                self._load_ultralytics(path)
        else:
            self._load_ultralytics(path)

    def _load_hailo(self, hef_path: str) -> None:
        try:
            from hailo_platform import (
                HEF, VDevice, HailoStreamInterface,
                InferVStreams, ConfigureParams, FormatType,
            )
            hef = HEF(hef_path)
            target = VDevice()
            cfg = ConfigureParams.create_from_hef(hef, interface=HailoStreamInterface.PCIe)
            net_groups = target.configure(hef, cfg)
            self._hailo_runner = {
                "target": target,
                "net_groups": net_groups,
                "hef": hef,
            }
            self._backend = "hailo"
            self.logger.info(f"Hailo-8 HEF yüklendi: {hef_path}")
        except ImportError:
            self.logger.warning("hailo_platform paketi bulunamadı (sadece Raspberry Pi + Hailo-8'de mevcut)")
        except Exception as e:
            self.logger.error(f"Hailo yükleme hatası: {e}")

    def _load_ultralytics(self, path: str) -> None:
        try:
            from ultralytics import YOLO
            self._model = YOLO(path)
            self._backend = "ultralytics"
            self.logger.info(f"Ultralytics YOLO yüklendi: {path}")
        except ImportError:
            self.logger.error(
                "ultralytics paketi bulunamadı. Kurmak için: pip install ultralytics"
            )
        except Exception as e:
            self.logger.error(f"Ultralytics yükleme hatası: {e}")

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------

    def _depth_cb(self, msg: Image) -> None:
        try:
            raw = np.frombuffer(bytes(msg.data), dtype=np.uint16)
            self._depth_img = raw.reshape(msg.height, msg.width).astype(np.float32) / 1000.0
            self._depth_stamp = time.monotonic()
        except Exception as e:
            self.logger.error(f"depth convert: {e}")

    def _rgb_cb(self, msg: Image) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.logger.error(f"rgb convert: {e}")
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

    # ------------------------------------------------------------------
    # İnference
    # ------------------------------------------------------------------

    def _yolo_detect(self, frame: np.ndarray) -> list[dict]:
        """
        Ana inference noktası. Backend'e göre Hailo-8 veya ultralytics kullanır.
        Dönüş: [{'class_name': str, 'confidence': float, 'bbox': (x1,y1,x2,y2)}, ...]
        """
        if self._hailo_runner is not None:
            return self._infer_hailo(frame)
        if self._model is not None:
            return self._infer_ultralytics(frame)
        return []

    def _infer_ultralytics(self, frame: np.ndarray) -> list[dict]:
        try:
            results = self._model(frame, conf=self._conf_thr, verbose=False)
            out: list[dict] = []
            for r in results:
                boxes = r.boxes
                if boxes is None:
                    continue
                for box in boxes:
                    cls_id = int(box.cls[0])
                    cls_name_raw = self._model.names.get(cls_id, str(cls_id))
                    cls_name = self._resolve_class(cls_id, cls_name_raw)
                    conf = float(box.conf[0])
                    x1, y1, x2, y2 = (float(v) for v in box.xyxy[0])
                    out.append({"class_name": cls_name, "confidence": conf, "bbox": (x1, y1, x2, y2)})
            return out
        except Exception as e:
            self.logger.error(f"ultralytics infer: {e}")
            return []

    def _infer_hailo(self, frame: np.ndarray) -> list[dict]:
        """
        Hailo-8 NPU üzerinde YOLOv8 HEF inference.
        hailo_platform InferVStreams API kullanır.
        """
        try:
            from hailo_platform import InferVStreams, ConfigureParams
            runner = self._hailo_runner
            net_group = runner["net_groups"][0]
            hef = runner["hef"]

            # HEF giriş bilgisi
            input_vstream_info = hef.get_input_vstream_infos()[0]
            output_vstream_info = hef.get_output_vstream_infos()[0]
            _, h, w, c = input_vstream_info.shape

            # Frame hazırla (RGB, model boyutuna resize)
            import cv2
            img = cv2.resize(frame, (w, h))
            img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            input_data = {input_vstream_info.name: np.expand_dims(img_rgb, axis=0).astype(np.float32) / 255.0}

            with InferVStreams(net_group, {}, {}) as pipeline:
                raw_output = pipeline.infer(input_data)

            # YOLOv8 çıkış post-processing (basit NMS + decode)
            out_name = output_vstream_info.name
            preds = raw_output[out_name][0]  # shape: [n_boxes, 5+n_cls] or YOLO native format
            return self._decode_yolo_output(preds, orig_w=frame.shape[1], orig_h=frame.shape[0], model_w=w, model_h=h)
        except Exception as e:
            self.logger.error(f"hailo infer: {e}")
            return []

    def _decode_yolo_output(
        self,
        preds: np.ndarray,
        orig_w: int, orig_h: int,
        model_w: int, model_h: int,
    ) -> list[dict]:
        """
        YOLOv8 Hailo çıkışını DEOS detection listesine dönüştür.
        preds: [n_boxes, 4+n_cls] formatında (cx,cy,w,h, cls_scores...)
        """
        out: list[dict] = []
        if preds.ndim != 2 or preds.shape[1] < 5:
            return out
        sx = orig_w / model_w
        sy = orig_h / model_h
        for row in preds:
            cx, cy, bw, bh = row[0], row[1], row[2], row[3]
            cls_scores = row[4:]
            cls_id = int(np.argmax(cls_scores))
            conf = float(cls_scores[cls_id])
            if conf < self._conf_thr:
                continue
            x1 = (cx - bw / 2) * sx
            y1 = (cy - bh / 2) * sy
            x2 = (cx + bw / 2) * sx
            y2 = (cy + bh / 2) * sy
            cls_name = self._resolve_class(cls_id, str(cls_id))
            out.append({"class_name": cls_name, "confidence": conf, "bbox": (x1, y1, x2, y2)})
        return out

    def _resolve_class(self, cls_id: int, raw_name: str) -> str:
        """COCO id/isim → DEOS canonical sınıf adı."""
        if self._use_coco_map:
            mapped = _map_coco(cls_id, raw_name)
        else:
            mapped = raw_name.lower().strip()
        return SIGN_CANONICAL.get(mapped, mapped)

    # ------------------------------------------------------------------
    # Derinlik & lateral offset
    # ------------------------------------------------------------------

    def _sample_depth(self, cx: int, cy: int) -> Optional[float]:
        if self._depth_img is None:
            return None
        if time.monotonic() - self._depth_stamp > self._depth_timeout:
            return None
        h, w = self._depth_img.shape
        cy_c = max(0, min(cy, h - 1))
        cx_c = max(0, min(cx, w - 1))
        # 3x3 medyan ile gürültüyü azalt
        y0, y1 = max(0, cy_c - 1), min(h, cy_c + 2)
        x0, x1 = max(0, cx_c - 1), min(w, cx_c + 2)
        patch = self._depth_img[y0:y1, x0:x1]
        vals = patch[patch > 0.05]
        if len(vals) == 0:
            return None
        return float(np.median(vals))

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
