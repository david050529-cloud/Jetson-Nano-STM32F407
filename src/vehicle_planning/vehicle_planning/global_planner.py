# global_planner.py
"""
基于目标点的全局路径规划节点(GlobalPlanner)

职责：
  接收来自上层决策或手动指定的目标坐标，
  从当前位置到目标点生成一条平滑的直线插值路径，
  发布到 /planning/global_path 供局部规划器执行。

与 path_planner 的区别：
  - path_planner:加载比赛路点文件，管理完整竞赛路线（含重规划）
  - global_planner:接收任意目标点，快速生成点对点路径（用于测试或辅助任务）
  两者均发布到同一话题，比赛时通常只运行 path_planner。

话题：
  订阅：/planning/goal          (geometry_msgs/PoseStamped) — 目标点
        /localization/odometry  (nav_msgs/Odometry)         — 当前位姿
  发布：/planning/global_path   (nav_msgs/Path)             — 规划路径
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry


class GlobalPlanner(Node):
    """全局目标点路径规划节点（线性插值实现）"""

    # 路径插值分辨率：每隔此距离生成一个路点（米）
    INTERPOLATION_STEP = 0.5

    def __init__(self):
        super().__init__('global_planner')

        # ── 订阅器 ────────────────────────────────────────────────────────
        self.sub_goal = self.create_subscription(
            PoseStamped, '/planning/goal',
            self._goal_callback, 10)
        self.sub_odom = self.create_subscription(
            Odometry, '/localization/odometry',
            self._odom_callback, 10)

        # ── 发布器 ────────────────────────────────────────────────────────
        self.pub_path = self.create_publisher(Path, '/planning/global_path', 10)

        # ── 内部状态 ──────────────────────────────────────────────────────
        self.current_pose = None   # 最新的机器人位姿（来自里程计）

        self.get_logger().info('全局目标规划节点已启动（线性插值模式）')

    # ══════════════════════════════════════════════════════════════════════
    # 回调函数
    # ══════════════════════════════════════════════════════════════════════

    def _odom_callback(self, msg: Odometry):
        """保存最新位姿"""
        self.current_pose = msg.pose.pose

    def _goal_callback(self, goal_msg: PoseStamped):
        """
        接收目标点，生成从当前位置到目标的路径并发布

        插值策略：
          - 每 INTERPOLATION_STEP 米放置一个路点
          - 所有中间路点属性设为 1(直行)
          - z 坐标编码属性值（与 path_planner 格式一致）
        """
        if self.current_pose is None:
            self.get_logger().warn('尚未收到里程计数据，无法规划路径')
            return

        sx = self.current_pose.position.x
        sy = self.current_pose.position.y
        gx = goal_msg.pose.position.x
        gy = goal_msg.pose.position.y

        total_dist = math.hypot(gx - sx, gy - sy)

        if total_dist < 0.05:
            self.get_logger().info('当前位置已在目标附近（<5cm),无需规划')
            return

        # 根据总距离计算插值点数（至少包含起点和终点）
        n_points = max(2, int(total_dist / self.INTERPOLATION_STEP) + 1)

        path_msg = Path()
        path_msg.header.stamp    = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'

        for i in range(n_points):
            t = i / (n_points - 1)   # 参数 t ∈ [0, 1]
            pose = PoseStamped()
            pose.header           = path_msg.header
            pose.pose.position.x  = sx + t * (gx - sx)
            pose.pose.position.y  = sy + t * (gy - sy)
            pose.pose.position.z  = 1.0   # 属性：直行（编码在 z 坐标）
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.pub_path.publish(path_msg)
        self.get_logger().info(
            f'已发布路径：({sx:.2f}, {sy:.2f}) → ({gx:.2f}, {gy:.2f})，'
            f'全程 {total_dist:.1f}m，{n_points} 个路点')


def main(args=None):
    rclpy.init(args=args)
    node = GlobalPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
