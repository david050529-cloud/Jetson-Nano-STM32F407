#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import numpy as np
from image_transport import Subscriber

class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')
        self.declare_parameter('model', 'yolov8n.pt')
        model_path = self.get_parameter('model').get_parameter_value().string_value
        self.get_logger().info(f'Loading YOLO model: {model_path}')
        self.model = YOLO(model_path)
        self.bridge = CvBridge()

        # 使用 image_transport 订阅（自动解压 H.264）
        self.subscription = Subscriber(self, Image, '/camera/image_raw', self.image_callback)

        # 发布检测结果图像
        self.publisher = self.create_publisher(Image, '/camera/image_detected', 10)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        results = self.model(cv_image)
        annotated_frame = results[0].plot()

        out_msg = self.bridge.cv2_to_imgmsg(annotated_frame, 'bgr8')
        out_msg.header = msg.header
        self.publisher.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()