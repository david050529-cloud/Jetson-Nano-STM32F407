# traffic_sign_detector.py
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory

class TrafficSignDetector(Node):
    """交通标志检测节点：使用YOLO检测交通标识，发布检测结果数组"""
    def __init__(self):
        super().__init__('traffic_sign_detector')
        self.bridge = CvBridge()
        # 使用 ament_index 获取模型文件的绝对路径（安装后位于 share/vehicle_perception/config/）
        pkg_share = get_package_share_directory('vehicle_perception')
        model_path = os.path.join(pkg_share, 'config', 'sign_yolo_model.pt')
        self.get_logger().info(f'加载交通标志检测模型: {model_path}')
        self.model = YOLO(model_path)
        # 发布检测结果
        self.pub = self.create_publisher(Detection2DArray, '/perception/traffic_signs', 10)
        # 订阅图像
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.callback, 10)

    def callback(self, msg):
        """处理图像，提取检测到的交通标志边界框"""
        cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        results = self.model(cv_img)[0]  # YOLO检测结果
        detections = Detection2DArray()
        detections.header = msg.header
        for box in results.boxes:
            detection = Detection2D()
            # 计算边界框中心坐标和尺寸
            detection.bbox.center.position.x = float((box.xyxy[0][0] + box.xyxy[0][2]) / 2)
            detection.bbox.center.position.y = float((box.xyxy[0][1] + box.xyxy[0][3]) / 2)
            detection.bbox.size_x = float(box.xyxy[0][2] - box.xyxy[0][0])
            detection.bbox.size_y = float(box.xyxy[0][3] - box.xyxy[0][1])
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = self.model.names[int(box.cls[0])]  # 类别名称
            hypothesis.hypothesis.score = float(box.conf[0])                     # 置信度
            detection.results.append(hypothesis)
            detections.detections.append(detection)
        self.pub.publish(detections)

def main(args=None):
    rclpy.init(args=args)
    node = TrafficSignDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()