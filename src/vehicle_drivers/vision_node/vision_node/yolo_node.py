#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import numpy as np

class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')
        # 参数：模型路径（默认yolov8n.pt）
        self.declare_parameter('model', 'yolov8n.pt')
        model_path = self.get_parameter('model').get_parameter_value().string_value
        self.get_logger().info(f'Loading YOLO model: {model_path}')
        self.model = YOLO(model_path)  # 自动下载模型若不存在

        self.bridge = CvBridge()
        # 订阅图像话题
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        # 发布检测结果图像
        self.publisher = self.create_publisher(Image, '/camera/image_detected', 10)

    def image_callback(self, msg):
        try:
            # 将ROS图像转换为OpenCV图像
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'jpeg')
        except Exception as e:
            self.get_logger().error(f'cv_bridge conversion error: {e}')
            return

        # 运行YOLO检测
        results = self.model(cv_image)

        # 在图像上绘制检测框
        annotated_frame = results[0].plot()  # 返回带框的BGR图像

        # 将OpenCV图像转换为ROS图像并发布
        out_msg = self.bridge.cv2_to_imgmsg(annotated_frame, 'jpeg')
        out_msg.header = msg.header  # 保持时间戳同步
        self.publisher.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()