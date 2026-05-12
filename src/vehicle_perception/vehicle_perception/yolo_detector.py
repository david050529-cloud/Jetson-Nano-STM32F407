# yolo_detector.py
"""
YOLO 通用视频识别节点 (YoloDetector)

使用 YOLOv8 进行通用目标检测，针对 Jetson Nano 优化：
  - CUDA / TensorRT 加速推理
  - 订阅 raw Image (BGR8) 避免 JPEG 解码开销
  - 帧跳过机制防止处理积压
  - FP16 推理支持 (Jetson Maxwell GPU)

检测类别 (COCO 80 类):
  人(person)、自行车(bicycle)、汽车(car)、摩托车(motorcycle)、
  公交车(bus)、卡车(truck)、交通灯(traffic light)、停止标志(stop sign) 等

输出:
  /perception/detections        (vision_msgs/Detection2DArray)  检测结果
  /perception/detections/image  (sensor_msgs/Image)             标注图(调试用)
"""

import os
import time
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, CompressedImage
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from ultralytics import YOLO


# COCO 2017 class names (YOLOv8 default)
COCO_NAMES = {
    0: 'person', 1: 'bicycle', 2: 'car', 3: 'motorcycle', 4: 'airplane',
    5: 'bus', 6: 'train', 7: 'truck', 8: 'boat', 9: 'traffic light',
    10: 'fire hydrant', 11: 'stop sign', 12: 'parking meter', 13: 'bench',
    14: 'bird', 15: 'cat', 16: 'dog', 17: 'horse', 18: 'sheep', 19: 'cow',
    20: 'elephant', 21: 'bear', 22: 'zebra', 23: 'giraffe', 24: 'backpack',
    25: 'umbrella', 26: 'handbag', 27: 'tie', 28: 'suitcase', 29: 'frisbee',
    30: 'skis', 31: 'snowboard', 32: 'sports ball', 33: 'kite', 34: 'baseball bat',
    35: 'baseball glove', 36: 'skateboard', 37: 'surfboard', 38: 'tennis racket',
    39: 'bottle', 40: 'wine glass', 41: 'cup', 42: 'fork', 43: 'knife',
    44: 'spoon', 45: 'bowl', 46: 'banana', 47: 'apple', 48: 'sandwich',
    49: 'orange', 50: 'broccoli', 51: 'carrot', 52: 'hot dog', 53: 'pizza',
    54: 'donut', 55: 'cake', 56: 'chair', 57: 'couch', 58: 'potted plant',
    59: 'bed', 60: 'dining table', 61: 'toilet', 62: 'tv', 63: 'laptop',
    64: 'mouse', 65: 'remote', 66: 'keyboard', 67: 'cell phone', 68: 'microwave',
    69: 'oven', 70: 'toaster', 71: 'sink', 72: 'refrigerator', 73: 'book',
    74: 'clock', 75: 'vase', 76: 'scissors', 77: 'teddy bear', 78: 'hair drier',
    79: 'toothbrush',
}


class YoloDetector(Node):
    """YOLO 通用目标检测节点 — 针对 Jetson Nano 优化"""

    def __init__(self):
        super().__init__('yolo_detector')

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('conf_threshold', 0.35)
        self.declare_parameter('nms_iou', 0.45)
        self.declare_parameter('input_size', 640)
        self.declare_parameter('device', 'auto')  # auto / cuda:0 / cpu
        self.declare_parameter('half_precision', True)  # FP16 on CUDA
        self.declare_parameter('publish_annotated', False)
        self.declare_parameter('processing_skip', 0)  # 0=process every frame
        self.declare_parameter('engine_path', '')  # TensorRT .engine file (optional)

        self.conf_threshold   = float(self.get_parameter('conf_threshold').value)
        self.nms_iou          = float(self.get_parameter('nms_iou').value)
        self.input_size       = int(self.get_parameter('input_size').value)
        self.device           = self.get_parameter('device').value
        self.half_precision   = self.get_parameter('half_precision').value
        self.publish_annotated = self.get_parameter('publish_annotated').value
        self.processing_skip  = int(self.get_parameter('processing_skip').value)
        self.engine_path      = self.get_parameter('engine_path').value

        # Resolve model path — try multiple locations
        model_path = self.get_parameter('model_path').value
        if not os.path.isabs(model_path):
            # Try: 1) package share  2) cwd  3) home
            candidates = []
            try:
                from ament_index_python.packages import get_package_share_directory
                candidates.append(os.path.join(
                    get_package_share_directory('vehicle_perception'),
                    'config', model_path))
            except Exception:
                pass
            candidates.append(os.path.join(os.getcwd(), model_path))
            candidates.append(os.path.join(os.path.expanduser('~'), model_path))
            # pick the first that exists; fall back to cwd path (auto-download will rescue)
            model_path = next((p for p in candidates if os.path.exists(p)),
                              candidates[1] if len(candidates) > 1 else candidates[0])

        # Resolve device
        if self.device == 'auto':
            try:
                import torch
                self.device = 'cuda:0' if torch.cuda.is_available() else 'cpu'
            except ImportError:
                self.device = 'cpu'

        # ── Load model ─────────────────────────────────────────────────
        self.get_logger().info(f'Device: {self.device}  FP16: {self.half_precision}  '
                               f'Input: {self.input_size}  Conf: {self.conf_threshold}')

        if self.engine_path and os.path.exists(self.engine_path):
            # Use pre-exported TensorRT engine for maximum Jetson performance
            self.get_logger().info(f'Using TensorRT engine: {self.engine_path}')
            self.model = YOLO(self.engine_path, task='detect')
        else:
            # Try local model file first
            model_loaded = False
            if os.path.exists(model_path):
                self.get_logger().info(f'Loading YOLO model: {model_path}')
                try:
                    self.model = YOLO(model_path, task='detect')
                    model_loaded = True
                except Exception as e:
                    self.get_logger().warn(
                        f'Failed to load {model_path}: {e}\n'
                        f'  File may be corrupted — trying auto-download...')
                    # Remove the bad file so it doesn't block future attempts
                    try:
                        os.remove(model_path)
                    except OSError:
                        pass

            if not model_loaded:
                # Auto-download from ultralytics (yolov8n.pt ≈ 6 MB)
                self.get_logger().info(
                    f'Downloading yolov8n.pt from ultralytics (one-time)...')
                try:
                    self.model = YOLO('yolov8n.pt', task='detect')
                except Exception as e2:
                    self.get_logger().fatal(
                        f'Cannot load or download YOLO model: {e2}')
                    raise RuntimeError(
                        'YOLO model unavailable. Place a valid yolov8n.pt in '
                        'the workspace root or ensure internet access for '
                        'auto-download.') from e2

            # On Jetson: auto-export hint for TensorRT
            if self.device != 'cpu' and not self.engine_path:
                self.get_logger().info(
                    'Tip: export to TensorRT for 2-3x faster inference on Jetson:\n'
                    '  python3 -c "from ultralytics import YOLO; '
                    'm=YOLO(\'yolov8n.pt\'); '
                    f'm.export(format=\'engine\', device=0, half={self.half_precision}, '
                    f'imgsz={self.input_size})"')

        self.get_logger().info(f'Model classes: {list(self.model.names.values())[:5]}... '
                               f'({len(self.model.names)} total)')

        # ── CvBridge ───────────────────────────────────────────────────
        self.bridge = CvBridge()

        # ── Publishers ─────────────────────────────────────────────────
        self.pub_detections = self.create_publisher(
            Detection2DArray, '/perception/detections', 10)

        if self.publish_annotated:
            self.pub_annotated = self.create_publisher(
                Image, '/perception/detections/image', 5)

        # ── Subscriptions ──────────────────────────────────────────────
        # Primary: raw BGR8 Image (zero decode overhead — preferred)
        self.sub_raw = self.create_subscription(
            Image, '/camera/image_raw', self.image_raw_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

        # Fallback: CompressedImage (JPEG — requires software decode)
        self.sub_comp = self.create_subscription(
            CompressedImage, '/camera/image_raw/compressed',
            self.image_compressed_callback, 10)

        # ── State ──────────────────────────────────────────────────────
        self.frame_count = 0
        self.infer_count = 0
        self.total_infer_time = 0.0
        self.fps_log_time = time.time()
        self.last_stamp = None  # dedup: avoid processing same frame twice

        self.get_logger().info('YoloDetector ready — '
                               'subscribing /camera/image_raw + /camera/image_raw/compressed')

    # ── Callback: raw BGR8 Image (fast path — no decoding) ────────────
    def image_raw_callback(self, msg: Image):
        self.frame_count += 1

        # Frame skip for consistent throughput when inference < capture rate
        if self.processing_skip > 0 and (self.frame_count % (self.processing_skip + 1)) != 0:
            return

        # Dedup: skip if same timestamp as last processed (raw+compressed pub on same frame)
        if self.last_stamp is not None and msg.header.stamp == self.last_stamp:
            return
        self.last_stamp = msg.header.stamp

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().debug(f'Bridge error (raw): {e}')
            return

        self._process_frame(cv_image, msg.header)

    # ── Callback: CompressedImage (fallback — software JPEG decode) ───
    def image_compressed_callback(self, msg: CompressedImage):
        self.frame_count += 1

        if self.processing_skip > 0 and (self.frame_count % (self.processing_skip + 1)) != 0:
            return

        # Dedup (same as above)
        if self.last_stamp is not None and msg.header.stamp == self.last_stamp:
            return
        self.last_stamp = msg.header.stamp

        np_arr = np.frombuffer(msg.data, dtype=np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if cv_image is None:
            return
        self._process_frame(cv_image, msg.header)

    # ── Core inference ─────────────────────────────────────────────────
    def _process_frame(self, cv_image, header):
        t0 = time.time()

        # Run YOLO inference
        results = self.model(
            cv_image,
            imgsz=self.input_size,
            conf=self.conf_threshold,
            iou=self.nms_iou,
            device=self.device,
            half=self.half_precision and self.device != 'cpu',
            verbose=False,
        )[0]

        infer_ms = (time.time() - t0) * 1000.0
        self.infer_count += 1
        self.total_infer_time += infer_ms

        # ── Build Detection2DArray ─────────────────────────────────────
        detections_msg = Detection2DArray()
        detections_msg.header = header

        for box in results.boxes:
            cls_id = int(box.cls[0])
            score  = float(box.conf[0])

            x1, y1, x2, y2 = (float(v) for v in box.xyxy[0])

            detection = Detection2D()
            detection.header = header
            detection.bbox.center.position.x = (x1 + x2) / 2.0
            detection.bbox.center.position.y = (y1 + y2) / 2.0
            detection.bbox.size_x = max(x2 - x1, 1.0)
            detection.bbox.size_y = max(y2 - y1, 1.0)

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = COCO_NAMES.get(cls_id, self.model.names.get(cls_id, f'class_{cls_id}'))
            hyp.hypothesis.score    = score
            detection.results.append(hyp)
            detections_msg.detections.append(detection)

        self.pub_detections.publish(detections_msg)

        # ── Optional annotated image (debug) ───────────────────────────
        if self.publish_annotated:
            annotated = results.plot()
            ann_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            ann_msg.header = header
            self.pub_annotated.publish(ann_msg)

        # ── FPS logging ────────────────────────────────────────────────
        now = time.time()
        elapsed = now - self.fps_log_time
        if elapsed >= 10.0:
            avg_ms = self.total_infer_time / max(self.infer_count, 1)
            fps_in  = self.frame_count / elapsed
            fps_out = self.infer_count / elapsed
            self.get_logger().info(
                f'FPS: in={fps_in:.1f}  infer={fps_out:.1f}  '
                f'avg_latency={avg_ms:.1f}ms  device={self.device}  '
                f'detections={len(detections_msg.detections)}')
            self.frame_count = 0
            self.infer_count = 0
            self.total_infer_time = 0.0
            self.fps_log_time = now


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
