# obstacle_detector.py
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray

class ObstacleDetector(Node):
    """障碍物检测节点：处理2D激光雷达扫描数据，生成可视化标记"""
    def __init__(self):
        super().__init__('obstacle_detector')
        # 订阅激光雷达2D扫描（lidar_node 实际发布的 topic 和类型）
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )
        # 发布障碍物标记，topic 名称与 local_planner 订阅保持一致
        self.publisher_ = self.create_publisher(MarkerArray, '/perception/obstacles_markers', 10)
        # 有效测距范围（米）
        self.declare_parameter('min_range', 0.1)
        self.declare_parameter('max_range', 10.0)
        self.get_logger().info('Obstacle Detector Node has been started.')

    def lidar_callback(self, msg: LaserScan):
        """将 LaserScan 转换为笛卡尔坐标并发布障碍物标记"""
        min_range = self.get_parameter('min_range').value
        max_range = self.get_parameter('max_range').value

        points = []
        angle = msg.angle_min
        for r in msg.ranges:
            if min_range < r < max_range and math.isfinite(r):
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                points.append((x, y))
            angle += msg.angle_increment

        markers = self._build_markers(points, msg.header)
        self.publisher_.publish(markers)

    def _build_markers(self, points, header) -> MarkerArray:
        """为每个有效障碍物点生成球体 Marker"""
        markers = MarkerArray()
        for i, (x, y) in enumerate(points):
            marker = Marker()
            marker.header = header
            marker.ns = 'obstacles'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
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
