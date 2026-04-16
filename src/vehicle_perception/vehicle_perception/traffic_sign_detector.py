# traffic_sign_detector.py
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory

# 将 YOLO 模型输出的原始类别名统一映射到 local_planner 期望的标准名称。
# 若模型类别名不在此表中，则原样透传。
# 可根据实际模型的 model.names 内容扩充此表。
CLASS_NAME_MAP = {
    # 红灯 — 常见别名
    'red':                'red_light',
    'red_light':          'red_light',
    'traffic_light_red':  'red_light',
    'redlight':           'red_light',
    # 绿灯 — 常见别名
    'green':              'green_light',
    'green_light':        'green_light',
    'traffic_light_green':'green_light',
    'greenlight':         'green_light',
    # 停止标志
    'stop':               'stop_sign',
    'stop_sign':          'stop_sign',
    'stopsign':           'stop_sign',
    # 禁止进入
    'no_entry':           'no_entry',
    'no_entry_sign':      'no_entry',
    'do_not_enter':       'no_entry',
}


class TrafficSignDetector(Node):
    """交通标志检测节点：使用YOLO检测交通标识，发布检测结果数组"""

    # 检测框面积最小阈值（像素²）：过小的框视为距离过远，忽略
    MIN_BBOX_AREA = 1000.0

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

    def _normalize_class(self, raw_name: str) -> str:
        """将模型原始类别名映射到标准名称（不区分大小写）"""
        return CLASS_NAME_MAP.get(raw_name.lower(), raw_name.lower())

    def callback(self, msg):
        """处理图像，提取检测到的交通标志边界框"""
        cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        results = self.model(cv_img)[0]  # YOLO检测结果
        detections = Detection2DArray()
        detections.header = msg.header
        for box in results.boxes:
            w = float(box.xyxy[0][2] - box.xyxy[0][0])
            h = float(box.xyxy[0][3] - box.xyxy[0][1])
            # 过滤面积过小的检测框（标志距离太远，不响应）
            if w * h < self.MIN_BBOX_AREA:
                continue
            detection = Detection2D()
            # 计算边界框中心坐标和尺寸
            detection.bbox.center.position.x = float((box.xyxy[0][0] + box.xyxy[0][2]) / 2)
            detection.bbox.center.position.y = float((box.xyxy[0][1] + box.xyxy[0][3]) / 2)
            detection.bbox.size_x = w
            detection.bbox.size_y = h
            hypothesis = ObjectHypothesisWithPose()
            # 将原始类别名映射为 local_planner 期望的标准名称
            raw_class = self.model.names[int(box.cls[0])]
            hypothesis.hypothesis.class_id = self._normalize_class(raw_class)
            hypothesis.hypothesis.score = float(box.conf[0])
            detection.results.append(hypothesis)
            detections.detections.append(detection)
        self.pub.publish(detections)

def main(args=None):
    rclpy.init(args=args)
    node = TrafficSignDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()