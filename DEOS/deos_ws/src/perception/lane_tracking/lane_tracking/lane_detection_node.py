from __future__ import annotations

import json
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, String


def _preprocess(frame_bgr: np.ndarray, *, img_w: int, img_h: int) -> np.ndarray:
    img = cv2.resize(frame_bgr, (img_w, img_h), interpolation=cv2.INTER_LINEAR)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = img.astype(np.float32) / 255.0
    return np.expand_dims(img, axis=0)  # NHWC


def _postprocess_yolov8_seg_hailo(
    outputs: list[np.ndarray],
    *,
    orig_w: int,
    orig_h: int,
    img_w: int,
    img_h: int,
    conf_thresh: float,
) -> tuple[np.ndarray, np.ndarray, list[np.ndarray], np.ndarray]:
    # This follows your existing script’s convention.
    pred = outputs[0][0].T
    protos = outputs[1][0]
    num_masks = protos.shape[0]
    proto_h, proto_w = protos.shape[1], protos.shape[2]
    num_classes = pred.shape[1] - 4 - num_masks

    boxes = pred[:, :4]
    class_probs = pred[:, 4 : 4 + num_classes]
    mask_coeffs = pred[:, -num_masks:]

    scores = np.max(class_probs, axis=1)
    class_ids = np.argmax(class_probs, axis=1)
    valid = scores > conf_thresh
    boxes, scores, class_ids, mask_coeffs = boxes[valid], scores[valid], class_ids[valid], mask_coeffs[valid]
    if len(boxes) == 0:
        return np.zeros((0, 4), dtype=np.int32), np.zeros((0,), dtype=np.float32), [], np.zeros((0,), dtype=np.int32)

    x1 = (boxes[:, 0] - boxes[:, 2] / 2) * orig_w / img_w
    y1 = (boxes[:, 1] - boxes[:, 3] / 2) * orig_h / img_h
    x2 = (boxes[:, 0] + boxes[:, 2] / 2) * orig_w / img_w
    y2 = (boxes[:, 1] + boxes[:, 3] / 2) * orig_h / img_h
    bboxes = np.stack([x1, y1, x2, y2], axis=1).astype(int)

    idx = cv2.dnn.NMSBoxes(bboxes.tolist(), scores.tolist(), conf_thresh, 0.45)
    if len(idx) == 0:
        return np.zeros((0, 4), dtype=np.int32), np.zeros((0,), dtype=np.float32), [], np.zeros((0,), dtype=np.int32)
    idx = idx.flatten()
    bboxes, scores, mask_coeffs, class_ids = bboxes[idx], scores[idx], mask_coeffs[idx], class_ids[idx]

    proto_flat = protos.reshape(num_masks, -1)
    masks: list[np.ndarray] = []
    for coeff in mask_coeffs:
        m = (coeff @ proto_flat).reshape(proto_h, proto_w)
        m = 1.0 / (1.0 + np.exp(-m))
        m = cv2.resize(m, (orig_w, orig_h), interpolation=cv2.INTER_LINEAR)
        masks.append(m > 0.5)

    return bboxes, scores, masks, class_ids


def _extract_center_points_from_masks(
    frame_bgr: np.ndarray,
    masks: list[np.ndarray],
    class_ids: np.ndarray,
    *,
    roi_top_ratio: float,
    step_px: int = 15,
) -> list[tuple[int, int]]:
    h, w = frame_bgr.shape[:2]
    if not masks:
        return []

    # Use class_id==0 as lane/drivable area (same as your script).
    kernel = np.ones((7, 7), np.uint8)
    out: list[tuple[int, int]] = []
    for mask, cls_id in zip(masks, class_ids):
        if int(cls_id) != 0:
            continue
        m = cv2.resize(mask.astype(np.uint8), (w, h), interpolation=cv2.INTER_NEAREST)
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, kernel)
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, kernel)

        y_start, y_end = h - 1, int(h * roi_top_ratio)
        for y in range(y_start, y_end, -int(step_px)):
            xs = np.where(m[y, :] > 0)[0]
            if xs.size:
                cx = int(np.mean(xs))
                out.append((cx, int(y)))
        break  # one mask is enough

    return out


class LaneDetectionNode(Node):
    """
    Raspberry hedefi: Hailo HEF ile segmentation inference yapıp centerline noktaları üretir.
    Çıktı:
    - /perception/center_pts (Float32MultiArray): [x0,y0,x1,y1,...] pixel koordinatları
    - /perception/lane_debug (String): basit JSON debug (opsiyonel)
    """

    def __init__(self):
        super().__init__("lane_detection_node")

        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("out_center_pts_topic", "/perception/center_pts")
        self.declare_parameter("publish_debug", False)
        self.declare_parameter("debug_topic", "/perception/lane_debug")

        self.declare_parameter("hef_path", "model.hef")
        self.declare_parameter("img_w", 640)
        self.declare_parameter("img_h", 640)
        self.declare_parameter("conf_thresh", 0.40)
        self.declare_parameter("roi_top_ratio", 0.50)
        self.declare_parameter("step_px", 15)

        self._bridge = CvBridge()
        self._pub_pts = self.create_publisher(Float32MultiArray, str(self.get_parameter("out_center_pts_topic").value), 10)
        self._pub_dbg = self.create_publisher(String, str(self.get_parameter("debug_topic").value), 10)

        image_topic = str(self.get_parameter("image_topic").value)
        self.create_subscription(Image, image_topic, self._image_cb, 10)

        self._hailo_ok = False
        self._hailo = None
        self._input_name: Optional[str] = None
        self._output_infos = None
        self._network_group = None
        self._network_group_params = None
        self._input_vstreams_params = None
        self._output_vstreams_params = None

        self._init_hailo()

    def _init_hailo(self) -> None:
        hef_path = Path(str(self.get_parameter("hef_path").value))
        if not hef_path.exists():
            self.get_logger().error(f"HEF bulunamadı: {hef_path}")
            return

        try:
            from hailo_platform import (  # type: ignore
                HEF,
                VDevice,
                HailoStreamInterface,
                InferVStreams,
                ConfigureParams,
                InputVStreamParams,
                OutputVStreamParams,
                FormatType,
            )
        except Exception as e:
            self.get_logger().error(f"hailo_platform import edilemedi (Raspberry/Hailo gerekli): {e}")
            return

        try:
            target = VDevice()
            hef = HEF(str(hef_path))
            configure_params = ConfigureParams.create_from_hef(hef=hef, interface=HailoStreamInterface.PCIe)
            network_group = target.configure(hef, configure_params)[0]
            network_group_params = network_group.create_params()

            input_vstreams_params = InputVStreamParams.make_from_network_group(network_group, quantized=False, format_type=FormatType.FLOAT32)
            output_vstreams_params = OutputVStreamParams.make_from_network_group(network_group, quantized=False, format_type=FormatType.FLOAT32)

            input_name = hef.get_input_vstream_infos()[0].name
            output_infos = hef.get_output_vstream_infos()

            self._hailo = {
                "InferVStreams": InferVStreams,
                "target": target,
                "hef": hef,
            }
            self._network_group = network_group
            self._network_group_params = network_group_params
            self._input_vstreams_params = input_vstreams_params
            self._output_vstreams_params = output_vstreams_params
            self._input_name = input_name
            self._output_infos = output_infos
            self._hailo_ok = True
            self.get_logger().info("lane_detection_node: Hailo hazır")
        except Exception as e:
            self.get_logger().error(f"Hailo init hatası: {e}")

    def _image_cb(self, msg: Image) -> None:
        if not self._hailo_ok:
            return

        try:
            frame = self._bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge: {e}")
            return

        img_w = int(self.get_parameter("img_w").value)
        img_h = int(self.get_parameter("img_h").value)
        conf = float(self.get_parameter("conf_thresh").value)
        roi_top = float(self.get_parameter("roi_top_ratio").value)
        step_px = int(self.get_parameter("step_px").value)

        inp = _preprocess(frame, img_w=img_w, img_h=img_h)
        input_data = {str(self._input_name): inp}

        InferVStreams = self._hailo["InferVStreams"]
        try:
            with InferVStreams(self._network_group, self._input_vstreams_params, self._output_vstreams_params) as infer_pipeline:
                with self._network_group.activate(self._network_group_params):
                    infer_results = infer_pipeline.infer(input_data)
        except Exception as e:
            self.get_logger().error(f"Hailo inference: {e}")
            return

        outs = [infer_results[info.name] for info in self._output_infos]
        bboxes, scores, masks, class_ids = _postprocess_yolov8_seg_hailo(
            outs,
            orig_w=frame.shape[1],
            orig_h=frame.shape[0],
            img_w=img_w,
            img_h=img_h,
            conf_thresh=conf,
        )
        _ = (bboxes, scores)  # unused in centerline extraction

        pts = _extract_center_points_from_masks(
            frame,
            masks,
            class_ids,
            roi_top_ratio=roi_top,
            step_px=step_px,
        )

        msg_pts = Float32MultiArray()
        msg_pts.data = [float(v) for (x, y) in pts for v in (x, y)]
        self._pub_pts.publish(msg_pts)

        if bool(self.get_parameter("publish_debug").value):
            self._pub_dbg.publish(
                String(
                    data=json.dumps(
                        {
                            "n_pts": len(pts),
                            "roi_top_ratio": roi_top,
                            "img": {"w": frame.shape[1], "h": frame.shape[0]},
                        },
                        ensure_ascii=False,
                    )
                )
            )


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

