import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO  # 使用YOLO分割模型
from std_msgs.msg import String, Float32MultiArray

class RoadDetector(Node):
    def __init__(self):
        super().__init__('road_detector')
        self.bridge = CvBridge()
        # 加载预训练分割模型（输出两类：铺装、非铺装）
        self.model = YOLO('config/road_seg_model.pt')  
        self.pub_road_type = self.create_publisher(String, '/perception/road_type', 10)
        self.pub_road_mask = self.create_publisher(Image, '/perception/road_mask', 10)
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        results = self.model(cv_image, task='segment')
        road_type = 'unknown'   # 默认值
        if len(results) > 0 and results[0].masks is not None:
            mask = results[0].masks.data[0].cpu().numpy()
            paved_ratio = np.sum(mask > 0.5) / mask.size
            road_type = 'paved' if paved_ratio > 0.7 else 'unpaved'
            self.pub_road_type.publish(String(data=road_type))
            mask_img = (mask * 255).astype(np.uint8)
            self.pub_road_mask.publish(self.bridge.cv2_to_imgmsg(mask_img, 'mono8'))
        self.get_logger().info(f'Road type: {road_type}')

def main(args=None):
    rclpy.init(args=args)
    node = RoadDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()