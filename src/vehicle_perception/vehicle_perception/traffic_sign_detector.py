import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from ultralytics import YOLO

class TrafficSignDetector(Node):
    def __init__(self):
        super().__init__('traffic_sign_detector')
        self.bridge = CvBridge()
        self.model = YOLO('config/sign_yolo_model.pt')  # 训练好的交通标识检测模型
        self.pub = self.create_publisher(Detection2DArray, '/perception/traffic_signs', 10)
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.callback, 10)

    def callback(self, msg):
        cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        results = self.model(cv_img)[0]  # YOLO检测结果
        detections = Detection2DArray()
        detections.header = msg.header
        for box in results.boxes:
            detection = Detection2D()
            detection.bbox.center.position.x = float((box.xyxy[0][0] + box.xyxy[0][2]) / 2)
            detection.bbox.center.position.y = float((box.xyxy[0][1] + box.xyxy[0][3]) / 2)
            detection.bbox.size_x = float(box.xyxy[0][2] - box.xyxy[0][0])
            detection.bbox.size_y = float(box.xyxy[0][3] - box.xyxy[0][1])
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = self.model.names[int(box.cls[0])]
            hypothesis.hypothesis.score = float(box.conf[0])
            detection.results.append(hypothesis)
            detections.detections.append(detection)
        self.pub.publish(detections)

def main(args=None):
    rclpy.init(args=args)
    node = TrafficSignDetector()
    rclpy.spin(node)