# road_detector.py
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO  # YOLO分割模型
from std_msgs.msg import String, Float32MultiArray
from ament_index_python.packages import get_package_share_directory

class RoadDetector(Node):
    """道路检测节点:使用YOLO分割模型识别铺装/非铺装路面，发布道路类型和掩膜"""
    def __init__(self):
        super().__init__('road_detector')
        self.bridge = CvBridge()
        # 使用 ament_index 获取模型文件的绝对路径（安装后位于 share/vehicle_perception/config/）
        pkg_share = get_package_share_directory('vehicle_perception')
        model_path = os.path.join(pkg_share, 'config', 'road_seg_model.pt')
        self.get_logger().info(f'加载道路分割模型: {model_path}')
        self.model = YOLO(model_path)
        # 发布道路类型字符串
        self.pub_road_type = self.create_publisher(String, '/perception/road_type', 10)
        # 发布道路掩膜图像
        self.pub_road_mask = self.create_publisher(Image, '/perception/road_mask', 10)
        # 订阅摄像头原始图像
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

    def image_callback(self, msg):
        """处理图像，进行分割并判断道路类型"""
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        results = self.model(cv_image, task='segment')  # 执行分割
        road_type = 'unknown'   # 默认值
        if len(results) > 0 and results[0].masks is not None:
            mask = results[0].masks.data[0].cpu().numpy()  # 获取第一个类别的掩膜（假设0类为铺装）
            # 计算铺装路面像素占比
            paved_ratio = np.sum(mask > 0.5) / mask.size
            road_type = 'paved' if paved_ratio > 0.7 else 'unpaved'
            # 发布道路类型
            self.pub_road_type.publish(String(data=road_type))
            # 将掩膜转换为图像消息并发布
            mask_img = (mask * 255).astype(np.uint8)
            self.pub_road_mask.publish(self.bridge.cv2_to_imgmsg(mask_img, 'mono8'))
        self.get_logger().info(f'Road type: {road_type}')

def main(args=None):
    rclpy.init(args=args)
    node = RoadDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()