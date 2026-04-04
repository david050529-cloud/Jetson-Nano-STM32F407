import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Path, Odometry
from visualization_msgs.msg import MarkerArray
import numpy as np
import math

class SimpleDWAPlanner(Node):
    def __init__(self):
        super().__init__('local_planner')
        self.sub_path = self.create_subscription(Path, '/planning/global_path', self.path_callback, 10)
        self.sub_odom = self.create_subscription(Odometry, '/localization/odometry', self.odom_callback, 10)
        self.sub_obstacles = self.create_subscription(MarkerArray, '/perception/obstacles_markers', self.obstacle_callback, 10)
        self.sub_road_type = self.create_subscription(String, '/perception/road_type', self.road_callback, 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)
        self.global_path = []
        self.current_pose = None
        self.obstacle_positions = []
        self.max_speed = 0.5  # m/s
        self.road_type = 'paved'

    def path_callback(self, msg):
        self.global_path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def obstacle_callback(self, msg):
        self.obstacle_positions = []
        for marker in msg.markers:
            self.obstacle_positions.append((marker.pose.position.x, marker.pose.position.y))

    def road_callback(self, msg):
        self.road_type = msg.data
        if self.road_type == 'unpaved':
            self.max_speed = 0.3
        else:
            self.max_speed = 0.8

    def control_loop(self):
        if self.current_pose is None or not self.global_path:
            return
        # 查找最近路径点
        px, py = self.current_pose.position.x, self.current_pose.position.y
        dists = [math.hypot(x-px, y-py) for x, y in self.global_path]
        closest_idx = np.argmin(dists)
        if closest_idx + 1 >= len(self.global_path):
            return  # 到达终点
        target_x, target_y = self.global_path[closest_idx+1]
        # 计算角度差
        dx = target_x - px
        dy = target_y - py
        angle_to_target = math.atan2(dy, dx)
        yaw = self.get_yaw_from_quaternion(self.current_pose.orientation)
        angle_diff = angle_to_target - yaw
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
        # 避障检测：若有障碍物距离<1米，则减速并转向
        avoid = False
        for ox, oy in self.obstacle_positions:
            dist = math.hypot(ox-px, oy-py)
            if dist < 1.0:
                avoid = True
                # 简单避障：转向远离障碍物方向
                angle_to_obs = math.atan2(oy-py, ox-px)
                diff_obs = angle_to_obs - yaw
                if diff_obs > 0:
                    angle_diff = -0.5  # 左转
                else:
                    angle_diff = 0.5
                break
        # 输出速度
        twist = Twist()
        twist.linear.x = self.max_speed * (1.0 - 0.5*avoid)  # 避障时降速
        twist.angular.z = max(-0.5, min(0.5, angle_diff * 1.5))  # P控制
        self.pub_cmd.publish(twist)

    def get_yaw_from_quaternion(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)