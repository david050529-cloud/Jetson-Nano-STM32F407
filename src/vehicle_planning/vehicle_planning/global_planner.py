# global_planner.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path, Odometry   # 导入Odometry用于获取当前位姿
import math

class GlobalPlanner(Node):
    """全局路径规划节点：接收目标点，基于当前位姿生成直线路径（示例）"""
    def __init__(self):
        super().__init__('global_planner')
        # 订阅目标点（来自上层决策或手动设置）
        self.sub_goal = self.create_subscription(PoseStamped, '/planning/goal', self.goal_callback, 10)
        # 发布全局路径（供局部规划器使用）
        self.pub_path = self.create_publisher(Path, '/planning/global_path', 10)
        self.current_pose = None  # 存储当前机器人位姿
        # 订阅里程计以更新当前位姿
        self.create_subscription(Odometry, '/localization/odometry', self.pose_callback, 10)

    def pose_callback(self, msg):
        """里程计回调：保存当前位姿"""
        self.current_pose = msg.pose.pose

    def goal_callback(self, goal_msg):
        """目标点回调：生成从当前位置到目标点的直线路径（插值10个点）"""
        if self.current_pose is None:
            return  # 未收到里程计，无法规划
        path = Path()
        path.header = goal_msg.header
        # 线性插值生成11个点（包含起点和终点）
        for i in range(11):
            t = i / 10.0
            pose = PoseStamped()
            pose.header = goal_msg.header
            pose.pose.position.x = self.current_pose.position.x * (1-t) + goal_msg.pose.position.x * t
            pose.pose.position.y = self.current_pose.position.y * (1-t) + goal_msg.pose.position.y * t
            path.poses.append(pose)
        self.pub_path.publish(path)