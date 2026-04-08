# path_planner.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import heapq
import math
from .waypoint_parser import parse_waypoint_file, parse_cartesian_waypoint_file
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from geographiclib.geodesic import Geodesic

class PathPlanner(Node):
    """基于航点文件的全局路径规划节点（将GPS航点转换为笛卡尔坐标并发布路径）"""
    def __init__(self):
        super().__init__('path_planner')
        # 声明参数：航点文件路径
        self.declare_parameter('waypoint_file', '/path/to/waypoint.txt')
        # 发布规划好的路径（话题名 planned_path）
        self.publisher_ = self.create_publisher(Path, 'planned_path', QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE
        ))
        # 订阅目标点（来自RViz或其他规划器）
        self.subscription = self.create_subscription(
            PoseStamped,
            'goal_pose',
            self.goal_callback,
            10
        )
        self.get_logger().info('Path Planner Node has been started.')
        # 加载航点文件
        self.waypoints = self.load_waypoints()

    def load_waypoints(self):
        """从参数指定的文件加载航点（纬度、经度）"""
        waypoint_file = self.get_parameter('waypoint_file').get_parameter_value().string_value
        try:
            waypoints = parse_waypoint_file(waypoint_file)
            self.get_logger().info(f'Loaded {len(waypoints)} waypoints from {waypoint_file}.')
            return waypoints
        except FileNotFoundError as e:
            self.get_logger().error(str(e))
            return []
        except ValueError as e:
            self.get_logger().error(f"Error parsing waypoint file: {e}")
            return []

    def convert_to_cartesian(self, latitude, longitude):
        """利用地理库将经纬度转换为笛卡尔坐标（以第一个航点为原点）"""
        geod = Geodesic.WGS84
        origin = self.waypoints[0]  # 原点为第一个航点
        result = geod.Inverse(origin['latitude'], origin['longitude'], latitude, longitude)
        x = result['s12'] * math.cos(math.radians(result['azi1']))
        y = result['s12'] * math.sin(math.radians(result['azi1']))
        return x, y

    def goal_callback(self, msg):
        """当接收到目标点时，将所有航点转换为路径并发布（示例实现）"""
        self.get_logger().info(f'Received goal pose: {msg.pose.position}')

        if not self.waypoints:
            self.get_logger().error('No waypoints loaded. Cannot plan path.')
            return

        # 将所有航点转换为笛卡尔坐标
        cartesian_waypoints = []
        for wp in self.waypoints:
            x, y = self.convert_to_cartesian(wp['latitude'], wp['longitude'])
            cartesian_waypoints.append((x, y, wp['attribute']))

        # 发布路径消息
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        for x, y, attr in cartesian_waypoints:
            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            path_msg.poses.append(pose)

        self.publisher_.publish(path_msg)
        self.get_logger().info('Published path from waypoints.')

def main(args=None):
    rclpy.init(args=args)
    node = PathPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()