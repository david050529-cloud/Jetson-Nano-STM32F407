#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class YoloVideoSubscriber(Node):
    def __init__(self):
        super().__init__('yolo_video_subscriber')
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)

        # 需要 ONNX 格式模型，例如通过 `yolo export model=yolo11n.pt format=onnx` 导出
        model_path = 'yolo11n.onnx'
        self.net = cv2.dnn.readNet(model_path)
        if self.net.empty():
            self.get_logger().error(f'Failed to load YOLO model from {model_path}')

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

            blob = cv2.dnn.blobFromImage(
                frame, 1.0 / 255.0, (640, 640), swapRB=True, crop=False)
            self.net.setInput(blob)
            outputs = self.net.forward(self.net.getUnconnectedOutLayersNames())

            for output in outputs:
                for detection in output:
                    confidence = detection[4]
                    if confidence > 0.5:
                        x = int(detection[0] * frame.shape[1])
                        y = int(detection[1] * frame.shape[0])
                        w = int(detection[2] * frame.shape[1])
                        h = int(detection[3] * frame.shape[0])
                        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

            cv2.imshow('YOLO Detection', frame)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = YoloVideoSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
