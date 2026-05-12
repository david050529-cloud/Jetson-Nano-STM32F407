# road_detector.py
"""
道路信息检测节点(RoadDetector)

使用 YOLO 目标检测模型(best.pt)识别道路上的关键信号:
  - Zebra            斑马线(人行横道)
  - red / green / yellow   交通信号灯三色

输出:
  /perception/road_signs   (vision_msgs/Detection2DArray)
      包含本帧所有满足置信度阈值的检测框,class_id 字段为类别名称
      (Zebra / red / green / yellow),供 local_planner 做减速/停车决策
"""
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory


class RoadDetector(Node):
    """道路信号检测节点：使用 best.pt 同时检测斑马线与红/绿/黄交通灯"""

    def __init__(self):
        super().__init__('road_detector')
        self.bridge = CvBridge()

        # 检测置信度阈值，低于该值的框不发布
        self.declare_parameter('conf_threshold', 0.4)
        self.conf_threshold = float(
            self.get_parameter('conf_threshold').value)

        # 加载 best.pt（安装后位于 share/vehicle_perception/config/）
        pkg_share = get_package_share_directory('vehicle_perception')
        model_path = os.path.join(pkg_share, 'config', 'best.pt')
        self.get_logger().info(f'加载道路检测模型: {model_path}')
        self.model = YOLO(model_path)
        self.get_logger().info(
            f'模型类别: {self.model.names}  任务: {self.model.task}')

        # 发布检测结果（斑马线 + 交通灯）
        self.pub = self.create_publisher(
            Detection2DArray, '/perception/road_signs', 10)
        # 订阅摄像头原始图像
        self.sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)

    def image_callback(self, msg: Image):
        """对单帧图像执行目标检测并发布结果"""
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        results = self.model(cv_image, verbose=False)[0]

        detections = Detection2DArray()
        detections.header = msg.header

        for box in results.boxes:
            score = float(box.conf[0])
            if score < self.conf_threshold:
                continue

            x1, y1, x2, y2 = (float(v) for v in box.xyxy[0])
            detection = Detection2D()
            detection.header = msg.header
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
            self.get_logger().debug(f'检测到 {len(detections.detections)} 个目标: {cls_summary}')


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
