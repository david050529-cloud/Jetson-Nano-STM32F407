import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import MarkerArray, Marker
import numpy as np
import math

class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/perception/obstacles_markers', 10)

    def scan_callback(self, msg):
        # 将激光雷达数据转换为障碍物点（笛卡尔坐标）
        obstacles = []
        angle = msg.angle_min
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max and r < 1.5:  # 1.5米内障碍物
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                obstacles.append((x, y))
            angle += msg.angle_increment

        # 简单聚类：距离小于0.3米的点合并为一个障碍物
        clusters = []
        for obs in obstacles:
            found = False
            for cluster in clusters:
                if math.hypot(obs[0] - cluster[0], obs[1] - cluster[1]) < 0.3:
                    # 更新聚类中心
                    cluster[0] = (cluster[0] + obs[0]) / 2
                    cluster[1] = (cluster[1] + obs[1]) / 2
                    found = True
                    break
            if not found:
                clusters.append([obs[0], obs[1]])

        marker_array = MarkerArray()
        for i, (cx, cy) in enumerate(clusters):
            marker = Marker()
            marker.header = msg.header
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = cx
            marker.pose.position.y = cy
            marker.pose.position.z = 0.2
            marker.scale.x = marker.scale.y = marker.scale.z = 0.3
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker_array.markers.append(marker)
        self.pub_markers.publish(marker_array)
        self.get_logger().debug(f"Detected {len(clusters)} obstacles")

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    rclpy.spin(node)