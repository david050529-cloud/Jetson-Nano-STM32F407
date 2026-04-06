import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import heapq
import math
from .waypoint_parser import parse_waypoint_file, parse_cartesian_waypoint_file

class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')
        self.publisher_ = self.create_publisher(Path, 'planned_path', 10)
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
        waypoint_file = '/path/to/waypoint.txt'  # Update with actual path
        try:
            waypoints = parse_waypoint_file(waypoint_file)
            self.get_logger().info(f'Loaded {len(waypoints)} waypoints.')
            return waypoints
        except FileNotFoundError as e:
            self.get_logger().error(str(e))
            return []

    def dijkstra(self, start, goal, graph):
        # Priority queue for Dijkstra's algorithm
        queue = [(0, start)]  # (cost, node)
        visited = set()
        costs = {start: 0}
        parents = {start: None}

        while queue:
            current_cost, current_node = heapq.heappop(queue)

            if current_node in visited:
                continue

            visited.add(current_node)

            if current_node == goal:
                break

            for neighbor, weight in graph.get(current_node, []):
                new_cost = current_cost + weight
                if neighbor not in costs or new_cost < costs[neighbor]:
                    costs[neighbor] = new_cost
                    parents[neighbor] = current_node
                    heapq.heappush(queue, (new_cost, neighbor))

        # Reconstruct path
        path = []
        node = goal
        while node is not None:
            path.append(node)
            node = parents.get(node)
        path.reverse()
        return path

    def goal_callback(self, msg):
        self.get_logger().info(f'Received goal pose: {msg.pose.position}')

        # Example: Use waypoints for path planning
        if not self.waypoints:
            self.get_logger().error('No waypoints loaded. Cannot plan path.')
            return

        # Example: Publish the waypoints as a path
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        for wp in self.waypoints:
            pose = PoseStamped()
            pose.pose.position.x = wp['latitude']  # Replace with actual conversion if needed
            pose.pose.position.y = wp['longitude']
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