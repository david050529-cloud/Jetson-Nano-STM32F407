# local_planner.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path, Odometry
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import String
from stm32_serial_bridge.msg import MotorCommand   # 自定义电机舵机指令消息
import numpy as np
import math

class SimpleDWAPlanner(Node):
    """局部路径规划节点：基于DWA思想，跟踪全局路径并避障，输出电机舵机指令"""
    def __init__(self):
        super().__init__('local_planner')
        # 订阅全局路径
        self.sub_path = self.create_subscription(Path, '/planning/global_path', self.path_callback, 10)
        # 订阅里程计
        self.sub_odom = self.create_subscription(Odometry, '/localization/odometry', self.odom_callback, 10)
        # 订阅障碍物标记（来自感知）
        self.sub_obstacles = self.create_subscription(MarkerArray, '/perception/obstacles_markers', self.obstacle_callback, 10)
        # 订阅道路类型（铺装/非铺装）
        self.sub_road_type = self.create_subscription(String, '/perception/road_type', self.road_callback, 10)
        # 发布电机舵机指令
        self.pub_cmd = self.create_publisher(MotorCommand, 'motor_commands', 10)
        # 定时控制循环（10Hz）
        self.timer = self.create_timer(0.1, self.control_loop)

        self.global_path = []          # 存储全局路径点 [(x,y), ...]
        self.current_pose = None       # 当前位姿
        self.obstacle_positions = []   # 障碍物位置列表
        self.max_speed = 0.5           # 最大线速度(m/s)，根据道路类型动态调整
        self.road_type = 'paved'       # 当前道路类型

    def path_callback(self, msg):
        """接收全局路径，提取二维坐标点"""
        self.global_path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def odom_callback(self, msg):
        """保存里程计位姿"""
        self.current_pose = msg.pose.pose

    def obstacle_callback(self, msg):
        """提取障碍物位置（MarkerArray中的每个标记的位置）"""
        self.obstacle_positions = []
        for marker in msg.markers:
            self.obstacle_positions.append((marker.pose.position.x, marker.pose.position.y))

    def road_callback(self, msg):
        """根据道路类型调整最大速度"""
        self.road_type = msg.data
        if self.road_type == 'unpaved':
            self.max_speed = 0.3
        else:
            self.max_speed = 0.8

    def control_loop(self):
        """主控制循环：计算线速度和角速度，并发布电机舵机指令"""
        if self.current_pose is None or not self.global_path:
            return
        px, py = self.current_pose.position.x, self.current_pose.position.y
        # 找到全局路径上离当前位置最近的点
        dists = [math.hypot(x-px, y-py) for x, y in self.global_path]
        closest_idx = np.argmin(dists)
        # 如果已经是最后一个点，则停止控制
        if closest_idx + 1 >= len(self.global_path):
            return
        # 选取下一个路径点作为目标点
        target_x, target_y = self.global_path[closest_idx+1]
        dx = target_x - px
        dy = target_y - py
        angle_to_target = math.atan2(dy, dx)          # 目标方向角
        yaw = self.get_yaw_from_quaternion(self.current_pose.orientation)  # 当前偏航角
        angle_diff = angle_to_target - yaw
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))  # 归一化到[-pi, pi]

        # 避障逻辑：如果障碍物距离小于1米，则根据障碍物方向强制转向
        avoid = False
        for ox, oy in self.obstacle_positions:
            dist = math.hypot(ox-px, oy-py)
            if dist < 1.0:
                avoid = True
                angle_to_obs = math.atan2(oy-py, ox-px)
                diff_obs = angle_to_obs - yaw
                if diff_obs > 0:
                    angle_diff = -0.5   # 障碍物在右侧，向左转
                else:
                    angle_diff = 0.5    # 障碍物在左侧，向右转
                break

        # 线速度：有障碍物时减速，最大速度受道路类型限制
        linear = self.max_speed * (1.0 - 0.5*avoid)
        # 角速度限制在[-0.5, 0.5] rad/s，并乘比例系数
        angular = max(-0.5, min(0.5, angle_diff * 1.5))

        # 构造 MotorCommand 消息
        cmd = MotorCommand()
        cmd.motor1_target_rps = linear   # 假设线速度映射到电机1目标转速（转/秒）
        cmd.motor2_target_rps = linear   # 映射到电机2
        cmd.servo_angle = int(angular * 180 / math.pi)  # 角速度转舵机角度（度），具体映射需根据硬件调整
        self.pub_cmd.publish(cmd)

    def get_yaw_from_quaternion(self, q):
        """从四元数计算偏航角（yaw）"""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleDWAPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()