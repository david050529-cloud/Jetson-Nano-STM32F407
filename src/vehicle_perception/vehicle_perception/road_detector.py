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

    # 模型类别ID定义（须与训练时 data.yaml 中的 names 顺序一致）
    CLASS_PAVED   = 0   # 铺装路面
    CLASS_UNPAVED = 1   # 非铺装路面

    def image_callback(self, msg):
        """处理图像，按类别过滤掩膜并判断道路类型"""
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        results = self.model(cv_image, task='segment')
        road_type = 'unknown'
        combined_mask = None

        if len(results) > 0 and results[0].masks is not None:
            h, w = cv_image.shape[:2]
            paved_mask   = np.zeros((h, w), dtype=np.float32)
            unpaved_mask = np.zeros((h, w), dtype=np.float32)

            for i, cls_tensor in enumerate(results[0].boxes.cls):
                cls_id = int(cls_tensor.item())
                # masks.data 形状为 (N, mask_h, mask_w)，需要 resize 回原图尺寸
                raw_mask = results[0].masks.data[i].cpu().numpy()
                resized  = cv2.resize(raw_mask, (w, h), interpolation=cv2.INTER_LINEAR)
                if cls_id == self.CLASS_PAVED:
                    paved_mask   = np.maximum(paved_mask,   resized)
                elif cls_id == self.CLASS_UNPAVED:
                    unpaved_mask = np.maximum(unpaved_mask, resized)

            paved_pixels   = np.sum(paved_mask   > 0.5)
            unpaved_pixels = np.sum(unpaved_mask > 0.5)
            total_pixels   = h * w

            # 铺装面积占画面 30% 以上则判定为铺装路面
            if paved_pixels / total_pixels > 0.3:
                road_type    = 'paved'
                combined_mask = paved_mask
            elif unpaved_pixels / total_pixels > 0.3:
                road_type    = 'unpaved'
                combined_mask = unpaved_mask

        self.pub_road_type.publish(String(data=road_type))
        if combined_mask is not None:
            mask_img = (combined_mask * 255).astype(np.uint8)
            self.pub_road_mask.publish(self.bridge.cv2_to_imgmsg(mask_img, 'mono8'))
        self.get_logger().debug(f'Road type: {road_type}')

def main(args=None):
    rclpy.init(args=args)
    node = RoadDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()