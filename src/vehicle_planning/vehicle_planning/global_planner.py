import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
import math

class GlobalPlanner(Node):
    def __init__(self):
        super().__init__('global_planner')
        self.sub_goal = self.create_subscription(PoseStamped, '/planning/goal', self.goal_callback, 10)
        self.pub_path = self.create_publisher(Path, '/planning/global_path', 10)
        self.current_pose = None  # 需要订阅 /localization/odometry 更新
        self.create_subscription(Odometry, '/localization/odometry', self.pose_callback, 10)

    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose

    def goal_callback(self, goal_msg):
        if self.current_pose is None:
            return
        # 简单直线路径，实际可用A*网格
        path = Path()
        path.header = goal_msg.header
        # 插值10个点
        for i in range(11):
            t = i / 10.0
            pose = PoseStamped()
            pose.header = goal_msg.header
            pose.pose.position.x = self.current_pose.position.x * (1-t) + goal_msg.pose.position.x * t
            pose.pose.position.y = self.current_pose.position.y * (1-t) + goal_msg.pose.position.y * t
            path.poses.append(pose)
        self.pub_path.publish(path)