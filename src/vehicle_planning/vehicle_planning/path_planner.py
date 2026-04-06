import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import heapq
import math
from .waypoint_parser import parse_waypoint_file, parse_cartesian_waypoint_file
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSReliabilityPolicy
from geographiclib.geodesic import Geodesic

class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')
        self.declare_parameter('waypoint_file', '/path/to/waypoint.txt')
        self.publisher_ = self.create_publisher(Path, 'planned_path', QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE
        ))
        self.subscription = self.create_subscription(
            PoseStamped,
            'goal_pose',
            self.goal_callback,
            10
        )
        self.get_logger().info('Path Planner Node has been started.')

        # Load waypoints from file
        self.waypoints = self.load_waypoints()

    def load_waypoints(self):
        """Load waypoints from a file."""
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
        """Convert latitude and longitude to Cartesian coordinates using UTM projection."""
        geod = Geodesic.WGS84
        origin = self.waypoints[0]  # Assuming the first waypoint is the origin
        result = geod.Inverse(origin['latitude'], origin['longitude'], latitude, longitude)
        x = result['s12'] * math.cos(math.radians(result['azi1']))
        y = result['s12'] * math.sin(math.radians(result['azi1']))
        return x, y

    def goal_callback(self, msg):
        self.get_logger().info(f'Received goal pose: {msg.pose.position}')

        # Example: Use waypoints for path planning
        if not self.waypoints:
            self.get_logger().error('No waypoints loaded. Cannot plan path.')
            return

        # Convert waypoints to Cartesian coordinates
        cartesian_waypoints = []
        for wp in self.waypoints:
            x, y = self.convert_to_cartesian(wp['latitude'], wp['longitude'])
            cartesian_waypoints.append((x, y, wp['attribute']))

        # Example: Publish the waypoints as a path
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