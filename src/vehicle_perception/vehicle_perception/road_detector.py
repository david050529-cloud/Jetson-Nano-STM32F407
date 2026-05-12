# road_detector.py
"""
道路信息检测节点 (RoadDetector)

使用 YOLO 目标检测模型 (best.pt) 识别道路关键信号:
  - Zebra            斑马线(人行横道)
  - red / green / yellow   交通信号灯三色

订阅:
  /camera/image_raw (主通道, raw BGR8 — 零解码开销)
  /camera/image_raw/compressed (回退通道, JPEG)

输出:
  /perception/road_signs   (vision_msgs/Detection2DArray)
      包含本帧所有满足置信度阈值的检测框, class_id 字段为类别名称
      (Zebra / red / green / yellow), 供 local_planner 做减速/停车决策
"""

import os
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, CompressedImage
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory


class RoadDetector(Node):
    """道路信号检测节点: 使用 best.pt 同时检测斑马线与红/绿/黄交通灯"""

    def __init__(self):
        super().__init__('road_detector')

        # 检测置信度阈值
        self.declare_parameter('conf_threshold', 0.4)
        self.conf_threshold = float(
            self.get_parameter('conf_threshold').value)

        # 加载 best.pt (安装后位于 share/vehicle_perception/config/)
        pkg_share = get_package_share_directory('vehicle_perception')
        model_path = os.path.join(pkg_share, 'config', 'best.pt')
        self.get_logger().info(f'Loading road detection model: {model_path}')
        self.model = YOLO(model_path)
        self.get_logger().info(
            f'Model classes: {self.model.names}   task: {self.model.task}')

        # 发布检测结果
        self.pub = self.create_publisher(
            Detection2DArray, '/perception/road_signs', 10)

        # CvBridge (raw Image → cv::Mat)
        self.bridge = CvBridge()

        # ── 订阅 ──────────────────────────────────────────────────────
        # 主通道: raw BGR8 Image (零解码开销 — 性能最优)
        self.sub_raw = self.create_subscription(
            Image, '/camera/image_raw', self.image_raw_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

        # 回退通道: CompressedImage (JPEG — 向后兼容)
        self.sub_comp = self.create_subscription(
            CompressedImage, '/camera/image_raw/compressed',
            self.image_compressed_callback, 10)

        # 去重: 同一帧可能同时触发 raw 和 compressed 回调
        self.last_stamp = None

    def image_raw_callback(self, msg: Image):
        """Raw BGR8 快速通道 — 无需 JPEG 解码"""
        if self.last_stamp is not None and msg.header.stamp == self.last_stamp:
            return
        self.last_stamp = msg.header.stamp

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().debug(f'Bridge error (raw): {e}')
            return
        self._detect_and_publish(cv_image, msg.header)

    def image_compressed_callback(self, msg: CompressedImage):
        """JPEG 回退通道 — 软件解码后检测"""
        if self.last_stamp is not None and msg.header.stamp == self.last_stamp:
            return
        self.last_stamp = msg.header.stamp

        np_arr = np.frombuffer(msg.data, dtype=np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if cv_image is None:
            self.get_logger().warn('JPEG decode failed, frame dropped')
            return
        self._detect_and_publish(cv_image, msg.header)

    def _detect_and_publish(self, cv_image, header):
        """执行推理并发布结果"""
        results = self.model(cv_image, verbose=False)[0]

        detections = Detection2DArray()
        detections.header = header

        for box in results.boxes:
            score = float(box.conf[0])
            if score < self.conf_threshold:
                continue

            x1, y1, x2, y2 = (float(v) for v in box.xyxy[0])
            detection = Detection2D()
            detection.header = header
            detection.bbox.center.position.x = (x1 + x2) / 2.0
            detection.bbox.center.position.y = (y1 + y2) / 2.0
            detection.bbox.size_x = x2 - x1
            detection.bbox.size_y = y2 - y1

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = self.model.names[int(box.cls[0])]
            hyp.hypothesis.score = score
            detection.results.append(hyp)
            detections.detections.append(detection)

        self.pub.publish(detections)

        if detections.detections:
            cls_summary = ', '.join(
                f"{d.results[0].hypothesis.class_id}({d.results[0].hypothesis.score:.2f})"
                for d in detections.detections)
            self.get_logger().debug(
                f'Detected {len(detections.detections)} targets: {cls_summary}')


def main(args=None):
    rclpy.init(args=args)
    node = RoadDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
