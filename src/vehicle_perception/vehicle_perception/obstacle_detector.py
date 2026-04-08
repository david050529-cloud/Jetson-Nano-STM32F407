# obstacle_detector.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray
import numpy as np
import cv2

class ObstacleDetector(Node):
    """障碍物检测节点：处理LiDAR点云，生成可视化标记（示例实现）"""
    def __init__(self):
        super().__init__('obstacle_detector')
        # 订阅激光雷达点云
        self.subscription = self.create_subscription(
            PointCloud2,
            'lidar_points',
            self.lidar_callback,
            10
        )
        # 发布障碍物标记（用于可视化）
        self.publisher_ = self.create_publisher(MarkerArray, 'obstacle_markers', 10)
        self.get_logger().info('Obstacle Detector Node has been started.')

    def lidar_callback(self, msg):
        self.get_logger().info('Processing LiDAR data...')
        # 处理点云数据（实际实现需根据点云格式转换）
        points = self.process_lidar_data(msg)
        markers = self.generate_markers(points)
        self.publisher_.publish(markers)

    def process_lidar_data(self, msg):
        """将PointCloud2转换为numpy数组并处理（此处为占位示例，随机生成10个点）"""
        # 实际代码应使用 sensor_msgs.point_cloud2 模块读取点云
        points = np.random.rand(10, 3)  # 示例数据
        return points

    def generate_markers(self, points):
        """为每个障碍物点生成一个球体标记"""
        markers = MarkerArray()
        from visualization_msgs.msg import Marker  # 局部导入避免依赖
        for i, point in enumerate(points):
            marker = Marker()
            marker.id = i
            marker.type = Marker.SPHERE
            marker.pose.position.x = point[0]
            marker.pose.position.y = point[1]
            marker.pose.position.z = point[2]
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            markers.markers.append(marker)
        return markers

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()