#!/usr/bin/env python3
"""
test_detection_hef.py — Intel RealSense D415 + Hailo-8 HEF nesne tanıma testi.

ROS gerektirmez; Raspberry Pi üzerinde doğrudan çalışır.
Gereksinimler: pyrealsense2, hailo_platform, opencv-python, numpy

Kullanım:
  python3 test_detection_hef.py
  python3 test_detection_hef.py --hef /ros2_ws/models/detection.hef --conf 0.35
  python3 test_detection_hef.py --hef model.hef --classes cone barrier pedestrian vehicle

Ekranda gösterilir:
  • INF FPS  — Hailo çıkarım FPS'i
  • DSP FPS  — ekran (cv2) FPS'i
  • HAILO °C — Hailo-8 çip sıcaklığı
  • CAM °C   — RealSense depth sensör sıcaklığı
  • Her tespit: sınıf / güven skoru / mesafe (depth kameradan)

Tuşlar:
  q / ESC  — çıkış
  d        — HEF çıkış katmanı isimlerini + şekillerini konsola döker
  s        — geçerli kareyi frame_NNNN.jpg olarak kaydeder
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import threading
import time
from collections import deque
from contextlib import ExitStack
from pathlib import Path
from typing import Optional

import cv2
import numpy as np

# ── Varsayılan değerler ────────────────────────────────────────────────────
DEFAULT_HEF     = "/ros2_ws/models/detection.hef"
DEFAULT_CONF    = 0.35
DEFAULT_IOU     = 0.45
DEFAULT_CLASSES = ["cone", "barrier", "pedestrian", "vehicle",
                   "traffic_light", "stop_sign", "unknown"]
PRE_NMS_TOPK    = 300
TEMP_INTERVAL_S = 2.0   # sıcaklık okuma aralığı (saniye)
FPS_WINDOW      = 30    # FPS rolling-average penceresi (frame)
DEPTH_TIMEOUT_S = 0.5   # depth frame bu süreden eskiyse kullanma

# sınıf id → BGR rengi
_PALETTE = [
    (0, 220, 0),     # 0 cone        — yeşil
    (0, 0, 220),     # 1 barrier     — kırmızı
    (0, 165, 255),   # 2 pedestrian  — turuncu
    (220, 0, 220),   # 3 vehicle     — magenta
    (220, 220, 0),   # 4 traffic_light — cyan
    (0, 220, 220),   # 5 stop_sign   — sarı
    (120, 120, 120), # 6 unknown     — gri
]


def _color(cls_id: int) -> tuple[int, int, int]:
    return _PALETTE[cls_id % len(_PALETTE)]


# ═══════════════════════════════════════════════════════════════════════════
# Sıcaklık monitörü (arka plan thread)
# ═══════════════════════════════════════════════════════════════════════════

class TemperatureMonitor:
    """Periyodik olarak Hailo çip sıcaklığını sorgular (bloklamadan)."""

    def __init__(self, interval_s: float = TEMP_INTERVAL_S):
        self.hailo_temp: Optional[float] = None
        self._interval = interval_s
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self) -> None:
        while not self._stop.wait(self._interval):
            self.hailo_temp = self._read_hailo_temp()

    @staticmethod
    def _read_hailo_temp() -> Optional[float]:
        # Önce hailortcli ile dene
        try:
            out = subprocess.check_output(
                ["hailortcli", "sensor-control", "--board-config"],
                stderr=subprocess.DEVNULL, timeout=2,
            ).decode(errors="ignore")
            for line in out.splitlines():
                if "temperature" in line.lower() or "temp" in line.lower():
                    parts = line.split()
                    for p in parts:
                        try:
                            return float(p)
                        except ValueError:
                            continue
        except Exception:
            pass

        # Hailo ısıl bölgesi için sysfs'e bak
        try:
            for zone in Path("/sys/class/thermal").glob("thermal_zone*"):
                try:
                    ttype = (zone / "type").read_text().strip().lower()
                    if "hailo" in ttype:
                        raw = int((zone / "temp").read_text().strip())
                        return raw / 1000.0
                except Exception:
                    continue
        except Exception:
            pass
        return None

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=3)


# ═══════════════════════════════════════════════════════════════════════════
# Hailo çıkarım motoru
# ═══════════════════════════════════════════════════════════════════════════

class HailoDetector:
    """Hailo-8 HEF ile UINT8 kalıcı vstream inference."""

    def __init__(self, hef_path: str):
        self._hef_path = hef_path
        self._stack = ExitStack()
        self._pipeline = None
        self._input_name: Optional[str] = None
        self._output_names: list[str] = []
        self._output_shapes: dict[str, tuple] = {}
        self._input_h = 640
        self._input_w = 640
        self._ok = False
        self._init()

    def _init(self) -> None:
        path = Path(self._hef_path)
        if not path.exists():
            print(f"[HATA] HEF bulunamadı: {path}", file=sys.stderr)
            return
        try:
            from hailo_platform import (  # type: ignore
                HEF, VDevice, HailoStreamInterface,
                InferVStreams, ConfigureParams,
                InputVStreamParams, OutputVStreamParams, FormatType,
            )
        except ImportError as e:
            print(f"[HATA] hailo_platform import edilemedi: {e}", file=sys.stderr)
            return

        try:
            hef = HEF(str(path))
            target = self._stack.enter_context(VDevice())
            cfg = ConfigureParams.create_from_hef(hef=hef, interface=HailoStreamInterface.PCIe)
            net_group = target.configure(hef, cfg)[0]
            net_params = net_group.create_params()

            in_params  = InputVStreamParams.make(net_group, format_type=FormatType.UINT8)
            out_params = OutputVStreamParams.make(net_group, format_type=FormatType.FLOAT32)

            in_info  = hef.get_input_vstream_infos()[0]
            out_info = hef.get_output_vstream_infos()

            self._input_name = in_info.name
            # Hailo shape: (batch, H, W, C) ya da (batch, C, H, W) — her ikisini destekle
            shape = tuple(in_info.shape)
            if len(shape) == 4:
                # NHWC varsay
                self._input_h = shape[1]
                self._input_w = shape[2]
            else:
                self._input_h = self._input_w = 640

            for o in out_info:
                self._output_names.append(o.name)
                self._output_shapes[o.name] = tuple(o.shape)

            self._stack.enter_context(net_group.activate(net_params))
            self._pipeline = self._stack.enter_context(
                InferVStreams(net_group, in_params, out_params))

            self._ok = True
            print(f"[Hailo] Hazır  — giriş: {in_info.name} {shape}")
            print(f"[Hailo] Çıkışlar ({len(out_info)}):")
            for o in out_info:
                print(f"   {o.name}  {tuple(o.shape)}")
        except Exception as e:
            print(f"[HATA] Hailo init: {e}", file=sys.stderr)

    @property
    def ok(self) -> bool:
        return self._ok

    def infer(self, frame_bgr: np.ndarray) -> tuple[dict, float]:
        """Kare -> (ham çıktı sözlüğü, ms cinsinden süre).
        Giriş UINT8 RGB NHWC olarak hazırlanır."""
        resized = cv2.resize(frame_bgr, (self._input_w, self._input_h))
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        tensor = np.expand_dims(np.ascontiguousarray(rgb), axis=0)

        t0 = time.perf_counter()
        raw = self._pipeline.infer({str(self._input_name): tensor})
        elapsed_ms = (time.perf_counter() - t0) * 1000.0
        return raw, elapsed_ms

    def close(self) -> None:
        self._stack.close()

    def dump_outputs(self) -> None:
        """'d' tuşuna basılınca çıkış katmanlarını konsola yaz."""
        print("\n─── HEF Çıkış Katmanları ───────────────────────────────")
        for name, shape in self._output_shapes.items():
            print(f"  {name:60s}  {str(shape):30s}")
        print("────────────────────────────────────────────────────────\n")


# ═══════════════════════════════════════════════════════════════════════════
# Tespit decode
# ═══════════════════════════════════════════════════════════════════════════

def _sigmoid(x: np.ndarray) -> np.ndarray:
    return 1.0 / (1.0 + np.exp(-np.clip(x, -88, 88)))


def _nms(boxes_xyxy: np.ndarray, scores: np.ndarray, iou_thr: float) -> np.ndarray:
    if len(boxes_xyxy) == 0:
        return np.array([], dtype=int)
    xywh = np.empty_like(boxes_xyxy)
    xywh[:, 0] = boxes_xyxy[:, 0]
    xywh[:, 1] = boxes_xyxy[:, 1]
    xywh[:, 2] = boxes_xyxy[:, 2] - boxes_xyxy[:, 0]
    xywh[:, 3] = boxes_xyxy[:, 3] - boxes_xyxy[:, 1]
    idx = cv2.dnn.NMSBoxes(xywh.tolist(), scores.tolist(), 0.0, iou_thr)
    return idx.flatten() if len(idx) > 0 else np.array([], dtype=int)


def _decode_hailo_nms(raw_outputs: dict, output_names: list[str],
                      conf_thr: float,
                      model_h: int, model_w: int, orig_h: int, orig_w: int) -> list[dict]:
    """Hailo entegre NMS çıktısı.

    Format: raw_outputs[name][batch][cls_id] = ndarray(N, 5)
    Her satır: [x1, y1, x2, y2, score]  — koordinatlar model input boyutunda (0..model_size).
    N sınıfa göre değiştiği için np.asarray() hata verir (inhomogeneous).
    """
    dets = []
    for name in output_names:
        out = raw_outputs[name]
        try:
            batch0 = out[0]       # batch 0
        except (IndexError, TypeError):
            continue

        for cls_id, cls_dets in enumerate(batch0):
            if cls_dets is None:
                continue
            try:
                cls_arr = np.asarray(cls_dets, dtype=np.float32)
            except (ValueError, TypeError):
                continue
            if cls_arr.ndim < 2 or cls_arr.shape[0] == 0 or cls_arr.shape[1] < 5:
                continue

            for row in cls_arr:
                score = float(row[4])
                if score < conf_thr:
                    continue
                x1, y1, x2, y2 = float(row[0]), float(row[1]), float(row[2]), float(row[3])

                # Koordinat aralığını tespit et: >1 ise model piksel uzayında, <=1 ise normalize
                if x2 > 1.0 or y2 > 1.0:
                    ox1 = max(0, int(x1 * orig_w / model_w))
                    oy1 = max(0, int(y1 * orig_h / model_h))
                    ox2 = min(orig_w, int(x2 * orig_w / model_w))
                    oy2 = min(orig_h, int(y2 * orig_h / model_h))
                else:
                    ox1 = max(0, int(x1 * orig_w))
                    oy1 = max(0, int(y1 * orig_h))
                    ox2 = min(orig_w, int(x2 * orig_w))
                    oy2 = min(orig_h, int(y2 * orig_h))

                if ox2 <= ox1 or oy2 <= oy1:
                    continue
                dets.append({
                    "cls_id": cls_id,
                    "score":  score,
                    "bbox":   (ox1, oy1, ox2, oy2),
                })
    return dets


def _decode_dfl(raw_outputs: dict, output_names: list[str], output_shapes: dict,
                conf_thr: float, iou_thr: float, num_classes: int,
                model_h: int, model_w: int, orig_h: int, orig_w: int) -> list[dict]:
    """3 stride (8/16/32) DFL decode — şekle göre bbox(64-kanal) ve cls ayırt eder."""
    dfl_weights = np.arange(16, dtype=np.float32)
    stride_groups: dict[tuple, dict] = {}  # (H,W) -> {bbox, cls}

    for name in output_names:
        try:
            arr = np.asarray(raw_outputs[name], dtype=np.float32)
        except (ValueError, TypeError):
            continue
        if arr.ndim != 4:
            continue
        _, rh, rw, rc = arr.shape
        key = (rh, rw)
        if key not in stride_groups:
            stride_groups[key] = {}
        if rc == 64:
            stride_groups[key]["bbox"] = arr
        elif rc == num_classes:
            stride_groups[key]["cls"] = arr
        elif rc == 1 and num_classes == 1:
            stride_groups[key]["cls"] = arr

    if not stride_groups:
        return []

    all_boxes, all_scores, all_cls = [], [], []
    for (gh, gw), grp in stride_groups.items():
        if "bbox" not in grp or "cls" not in grp:
            continue
        stride = model_h // gh

        cls_raw = grp["cls"][0].reshape(-1, num_classes)
        cls_scores = _sigmoid(cls_raw)
        max_scores = np.max(cls_scores, axis=1)
        valid = max_scores >= conf_thr
        if not valid.any():
            continue

        valid_scores  = max_scores[valid]
        valid_cls_ids = np.argmax(cls_scores[valid], axis=1)

        bbox_dfl = grp["bbox"][0].reshape(-1, 64)[valid].reshape(-1, 4, 16)
        exp_b = np.exp(bbox_dfl - bbox_dfl.max(axis=2, keepdims=True))
        sfmx  = exp_b / exp_b.sum(axis=2, keepdims=True)
        dist  = (sfmx * dfl_weights).sum(axis=2)

        xg, yg = np.meshgrid(np.arange(gw), np.arange(gh))
        xg = xg.reshape(-1)[valid]
        yg = yg.reshape(-1)[valid]

        x1 = (xg + 0.5 - dist[:, 0]) * stride * (orig_w / model_w)
        y1 = (yg + 0.5 - dist[:, 1]) * stride * (orig_h / model_h)
        x2 = (xg + 0.5 + dist[:, 2]) * stride * (orig_w / model_w)
        y2 = (yg + 0.5 + dist[:, 3]) * stride * (orig_h / model_h)

        all_boxes.append(np.column_stack([x1, y1, x2, y2]))
        all_scores.append(valid_scores)
        all_cls.append(valid_cls_ids)

    if not all_boxes:
        return []

    boxes   = np.clip(np.concatenate(all_boxes), 0, max(orig_w, orig_h))
    scores  = np.concatenate(all_scores)
    cls_ids = np.concatenate(all_cls)

    if len(scores) > PRE_NMS_TOPK:
        top = np.argsort(scores)[::-1][:PRE_NMS_TOPK]
        boxes, scores, cls_ids = boxes[top], scores[top], cls_ids[top]

    keep = _nms(boxes, scores, iou_thr)
    if len(keep) == 0:
        return []

    return [{
        "cls_id": int(cls_ids[i]),
        "score":  float(scores[i]),
        "bbox":   (int(boxes[i][0]), int(boxes[i][1]),
                   int(boxes[i][2]), int(boxes[i][3])),
    } for i in keep]


def _decode_raw_cxcywh(raw_outputs: dict, output_names: list[str],
                        conf_thr: float, iou_thr: float,
                        model_h: int, model_w: int, orig_h: int, orig_w: int) -> list[dict]:
    """cx,cy,w,h,cls_scores... formatı (stereo_detector_node)."""
    all_boxes, all_scores, all_cls = [], [], []
    sx, sy = orig_w / model_w, orig_h / model_h

    for name in output_names:
        try:
            arr = np.asarray(raw_outputs[name], dtype=np.float32)
        except (ValueError, TypeError):
            continue
        if arr.ndim == 4:
            arr = arr[0].reshape(-1, arr.shape[-1])
        elif arr.ndim == 3:
            arr = arr[0]
        elif arr.ndim == 2:
            pass
        else:
            continue

        if arr.shape[1] < 5:
            continue

        cx, cy = arr[:, 0], arr[:, 1]
        bw, bh = arr[:, 2], arr[:, 3]
        cls_raw = arr[:, 4:]
        scores  = np.max(cls_raw, axis=1)
        cls_ids = np.argmax(cls_raw, axis=1)

        valid = scores >= conf_thr
        if not valid.any():
            continue

        x1 = (cx[valid] - bw[valid] / 2) * sx
        y1 = (cy[valid] - bh[valid] / 2) * sy
        x2 = (cx[valid] + bw[valid] / 2) * sx
        y2 = (cy[valid] + bh[valid] / 2) * sy

        all_boxes.append(np.column_stack([x1, y1, x2, y2]))
        all_scores.append(scores[valid])
        all_cls.append(cls_ids[valid])

    if not all_boxes:
        return []

    boxes   = np.clip(np.concatenate(all_boxes), 0, max(orig_w, orig_h))
    scores  = np.concatenate(all_scores)
    cls_ids = np.concatenate(all_cls)

    keep = _nms(boxes, scores, iou_thr)
    if len(keep) == 0:
        return []

    return [{
        "cls_id": int(cls_ids[i]),
        "score":  float(scores[i]),
        "bbox":   (int(boxes[i][0]), int(boxes[i][1]),
                   int(boxes[i][2]), int(boxes[i][3])),
    } for i in keep]


def decode(raw_outputs: dict, output_names: list[str], output_shapes: dict,
           conf_thr: float, iou_thr: float, num_classes: int,
           model_h: int, model_w: int, orig_h: int, orig_w: int) -> list[dict]:
    """Çıkış formatını otomatik algıla, uygun decoder'ı çalıştır.

    Öncelik sırası:
      1) Hailo entegre NMS  — inhomogeneous list (np.asarray ValueError verir)
      2) DFL conv split     — 64-kanallı bbox tensörü var
      3) cx,cy,w,h          — birleşik ham çıkış
    """
    # Format testi: herhangi bir çıkış np.asarray ile dönüştürülemiyorsa NMS formatı
    is_nms = False
    for name in output_names:
        try:
            np.asarray(raw_outputs[name], dtype=np.float32)
        except (ValueError, TypeError):
            is_nms = True
            break

    if is_nms:
        return _decode_hailo_nms(raw_outputs, output_names, conf_thr,
                                 model_h, model_w, orig_h, orig_w)

    # DFL decode
    has_dfl = any(s[-1] == 64 for s in output_shapes.values() if len(s) >= 1)
    if has_dfl:
        dets = _decode_dfl(raw_outputs, output_names, output_shapes,
                           conf_thr, iou_thr, num_classes, model_h, model_w, orig_h, orig_w)
        if dets:
            return dets

    # cx,cy,w,h fallback
    return _decode_raw_cxcywh(raw_outputs, output_names, conf_thr, iou_thr,
                               model_h, model_w, orig_h, orig_w)


# ═══════════════════════════════════════════════════════════════════════════
# RealSense yardımcıları
# ═══════════════════════════════════════════════════════════════════════════

def _rs_temp(pipeline) -> Optional[float]:
    """RealSense depth sensör sıcaklığını oku (°C). Desteklenmiyorsa None."""
    try:
        import pyrealsense2 as rs  # type: ignore
        device = pipeline.get_active_profile().get_device()
        for sensor in device.query_sensors():
            if sensor.supports(rs.option.temperature):
                return float(sensor.get_option(rs.option.temperature))
    except Exception:
        pass
    return None


# ═══════════════════════════════════════════════════════════════════════════
# Görüntü üzerine çizim
# ═══════════════════════════════════════════════════════════════════════════

def _draw_detections(frame: np.ndarray, dets: list[dict],
                     class_names: list[str], depth_img: Optional[np.ndarray]) -> None:
    for d in dets:
        x1, y1, x2, y2 = d["bbox"]
        cls_id = d["cls_id"]
        score  = d["score"]
        color  = _color(cls_id)
        name   = class_names[cls_id] if cls_id < len(class_names) else f"cls{cls_id}"

        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

        # Mesafe (depth_img varsa bbox merkezinden)
        dist_str = ""
        if depth_img is not None:
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2
            h, w = depth_img.shape
            cy_c = max(0, min(cy, h - 1))
            cx_c = max(0, min(cx, w - 1))
            patch = depth_img[max(0, cy_c-2):cy_c+3, max(0, cx_c-2):cx_c+3]
            vals  = patch[patch > 0.05]
            if len(vals):
                dist_str = f"  {np.median(vals):.2f}m"

        label = f"{name} {score:.2f}{dist_str}"
        (lw, lh), base = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
        ty = max(y1 - 6, lh + 4)
        cv2.rectangle(frame, (x1, ty - lh - 4), (x1 + lw + 4, ty + base), color, -1)
        cv2.putText(frame, label, (x1 + 2, ty - 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 1, cv2.LINE_AA)


def _draw_hud(frame: np.ndarray, inf_fps: float, disp_fps: float,
              hailo_temp: Optional[float], cam_temp: Optional[float],
              n_dets: int, inf_ms: float) -> None:
    h, w = frame.shape[:2]

    def put(text: str, pos: tuple, color=(255, 255, 255)):
        cv2.putText(frame, text, pos, cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(frame, text, pos, cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, color, 1, cv2.LINE_AA)

    hailo_str = f"{hailo_temp:.1f}" if hailo_temp is not None else "---"
    cam_str   = f"{cam_temp:.1f}"   if cam_temp   is not None else "---"

    put(f"INF  {inf_fps:5.1f} FPS  ({inf_ms:.1f} ms)", (10, 28), (100, 255, 100))
    put(f"DSP  {disp_fps:5.1f} FPS",                   (10, 56), (200, 200, 255))
    put(f"HAILO {hailo_str} C",                         (10, 84), (0, 220, 220))
    put(f"CAM   {cam_str} C",                           (10, 112),(0, 220, 220))
    put(f"TESPIT {n_dets}",                             (10, 140),(255, 220, 0))

    # Sağ alt: tuş yardımı
    put("q=cikis  d=katmanlar  s=kaydet", (10, h - 10), (180, 180, 180))


# ═══════════════════════════════════════════════════════════════════════════
# Ana akış
# ═══════════════════════════════════════════════════════════════════════════

def main() -> None:
    ap = argparse.ArgumentParser(description="RealSense D415 + Hailo HEF tespit testi")
    ap.add_argument("--hef",     default=DEFAULT_HEF,
                    help=f"HEF model dosyası (varsayılan: {DEFAULT_HEF})")
    ap.add_argument("--conf",    type=float, default=DEFAULT_CONF,
                    help=f"Güven eşiği (varsayılan: {DEFAULT_CONF})")
    ap.add_argument("--iou",     type=float, default=DEFAULT_IOU,
                    help=f"NMS IoU eşiği (varsayılan: {DEFAULT_IOU})")
    ap.add_argument("--classes", nargs="+", default=DEFAULT_CLASSES,
                    help="Sınıf adları (sırasıyla, cls_id=0'dan başlar)")
    ap.add_argument("--width",   type=int, default=640, help="Kamera genişliği")
    ap.add_argument("--height",  type=int, default=480, help="Kamera yüksekliği")
    ap.add_argument("--fps",     type=int, default=30,  help="Kamera FPS hedefi")
    ap.add_argument("--no-depth", action="store_true",  help="Depth akışını devre dışı bırak")
    args = ap.parse_args()

    # ── Hailo başlat ──────────────────────────────────────────────────────
    print(f"[Başlangıç] HEF yükleniyor: {args.hef}")
    hailo = HailoDetector(args.hef)
    if not hailo.ok:
        print("[HATA] Hailo başlatılamadı. Çıkılıyor.", file=sys.stderr)
        sys.exit(1)

    num_classes = len(args.classes)

    # ── RealSense başlat ──────────────────────────────────────────────────
    try:
        import pyrealsense2 as rs  # type: ignore
    except ImportError:
        print("[HATA] pyrealsense2 bulunamadı. Kurmak için: pip install pyrealsense2",
              file=sys.stderr)
        hailo.close()
        sys.exit(1)

    pipeline = rs.pipeline()
    config   = rs.config()
    config.enable_stream(rs.stream.color, args.width, args.height,
                         rs.format.bgr8, args.fps)
    if not args.no_depth:
        config.enable_stream(rs.stream.depth, args.width, args.height,
                             rs.format.z16, args.fps)

    try:
        profile = pipeline.start(config)
        print(f"[RealSense] Başlatıldı  {args.width}x{args.height}@{args.fps}fps")
    except Exception as e:
        print(f"[HATA] RealSense başlatılamadı: {e}", file=sys.stderr)
        hailo.close()
        sys.exit(1)

    # Derinlik ölçeği (m cinsine çevirmek için)
    depth_scale = 1.0
    if not args.no_depth:
        try:
            depth_sensor = profile.get_device().first_depth_sensor()
            depth_scale = depth_sensor.get_depth_scale()
        except Exception:
            depth_scale = 0.001

    # ── Sıcaklık monitörü ─────────────────────────────────────────────────
    temp_mon = TemperatureMonitor()

    # ── FPS sayaçları ─────────────────────────────────────────────────────
    inf_times:  deque[float] = deque(maxlen=FPS_WINDOW)  # ms
    disp_times: deque[float] = deque(maxlen=FPS_WINDOW)  # s
    last_disp_t = time.perf_counter()

    depth_img:      Optional[np.ndarray] = None
    depth_stamp:    float = 0.0
    cam_temp:       Optional[float] = None
    last_cam_temp_t = 0.0
    frame_count = 0

    print("\n[Çalışıyor] Pencere açıldığında: q=çıkış  d=katmanlar  s=kaydet\n")

    align = rs.align(rs.stream.color) if not args.no_depth else None

    try:
        while True:
            # ── Frame al ──────────────────────────────────────────────────
            try:
                frames = pipeline.wait_for_frames(timeout_ms=2000)
            except RuntimeError:
                print("[UYARI] Frame zaman aşımı — devam ediliyor")
                continue

            if align is not None:
                frames = align.process(frames)

            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            frame = np.asanyarray(color_frame.get_data())
            orig_h, orig_w = frame.shape[:2]

            # Depth frame
            if not args.no_depth:
                df = frames.get_depth_frame()
                if df:
                    raw_d = np.asanyarray(df.get_data()).astype(np.float32)
                    depth_img   = raw_d * depth_scale   # metre cinsinden
                    depth_stamp = time.perf_counter()
                elif time.perf_counter() - depth_stamp > DEPTH_TIMEOUT_S:
                    depth_img = None

            # ── Hailo inference ───────────────────────────────────────────
            raw_outputs, inf_ms = hailo.infer(frame)
            inf_times.append(inf_ms)

            # ── Decode ────────────────────────────────────────────────────
            dets = decode(
                raw_outputs,
                hailo._output_names,
                hailo._output_shapes,
                args.conf, args.iou,
                num_classes,
                hailo._input_h, hailo._input_w,
                orig_h, orig_w,
            )

            # ── Kamera sıcaklığı (her 5 saniyede) ────────────────────────
            now = time.perf_counter()
            if now - last_cam_temp_t > 5.0:
                cam_temp = _rs_temp(pipeline)
                last_cam_temp_t = now

            # ── FPS hesapla ───────────────────────────────────────────────
            elapsed = now - last_disp_t
            last_disp_t = now
            disp_times.append(elapsed)

            inf_fps  = 1000.0 / (sum(inf_times)  / len(inf_times))  if inf_times  else 0.0
            disp_fps = 1.0    / (sum(disp_times) / len(disp_times)) if disp_times else 0.0

            # ── Çizim ─────────────────────────────────────────────────────
            vis = frame.copy()
            _draw_detections(vis, dets, args.classes,
                             depth_img if not args.no_depth else None)
            _draw_hud(vis, inf_fps, disp_fps,
                      temp_mon.hailo_temp, cam_temp,
                      len(dets), inf_ms)

            cv2.imshow("DEOS — HEF Tespit Testi", vis)

            frame_count += 1
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):   # q veya ESC
                break
            elif key == ord('d'):
                hailo.dump_outputs()
            elif key == ord('s'):
                fname = f"frame_{frame_count:04d}.jpg"
                cv2.imwrite(fname, vis)
                print(f"[Kaydet] {fname}")

    except KeyboardInterrupt:
        print("\n[Durduruldu] Ctrl+C")
    finally:
        cv2.destroyAllWindows()
        pipeline.stop()
        temp_mon.stop()
        hailo.close()
        print("[Kapatıldı]")


if __name__ == "__main__":
    main()
