# path_planner.py
"""
比赛路点路径规划节点(PathPlanner)

职责：
  1. 节点启动时从 waypoint_file 参数指定的文件加载比赛路点
  2. 将 GPS 经纬度坐标转换为以第一个路点为原点的局部直角坐标系ENU,单位(米)
  3. 立即将完整竞赛路径发布到 /planning/global_path(路点属性编码到 z 坐标)
  4. 监听 /planning/replan_request,在路线被堵塞时利用路网图重新规划绕行路径

话题：
  发布：/planning/global_path   (nav_msgs/Path)  — 完整竞赛路径（含属性）
  订阅：/planning/replan_request (std_msgs/Int32) — 重规划请求（目标路点序号）
        /planning/current_waypoint_index (std_msgs/Int32) — 局部规划器汇报的进度

参数：
  waypoint_file      (str,  默认 '/tmp/waypoint.txt') : 路点文件路径
  road_network_file  (str,  默认 '')                  : 路网文件路径（空则自动构建）
  use_cartesian      (bool, 默认 False)               : 是否使用直角坐标格式路点
"""

import os
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy,
                        QoSHistoryPolicy, QoSReliabilityPolicy)
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Int32
from geographiclib.geodesic import Geodesic

from .waypoint_parser import parse_waypoint_file, parse_cartesian_waypoint_file
from .road_network_parser import (RoadNetwork,
                                   parse_road_network_file,
                                   build_network_from_waypoints)


class PathPlanner(Node):
    """
    比赛路点路径规划节点

    GPS 坐标转换说明：
      使用 WGS84 大地测量模型（geographiclib）计算反向大地线问题，
      以第一个路点为局部坐标系原点，东向为 +X，北向为 +Y（ENU 坐标系）。

    路点属性传递方式：
      将整数属性值编码到 PoseStamped.pose.position.z 中（z 字段在 2D 场景无用）。
      局部规划器读取 z 字段还原属性值，无需自定义消息类型。
    """

    def __init__(self):
        super().__init__('path_planner')

        # ── 参数声明 ──────────────────────────────────────────────────────
        self.declare_parameter('waypoint_file',     '/tmp/waypoint.txt')
        self.declare_parameter('road_network_file', '')
        self.declare_parameter('use_cartesian',     False)

        # ── 发布器（RELIABLE + TRANSIENT_LOCAL：新订阅者能收到最近一次路径）──
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub_path = self.create_publisher(Path, '/planning/global_path', qos)

        # ── 订阅器 ────────────────────────────────────────────────────────
        # 接收局部规划器的路点进度反馈
        self.sub_progress = self.create_subscription(
            Int32, '/planning/current_waypoint_index',
            self._progress_callback, 10)
        # 接收重规划请求（参数为目标路点序号）
        self.sub_replan = self.create_subscription(
            Int32, '/planning/replan_request',
            self._replan_callback, 10)

        # ── 内部状态 ──────────────────────────────────────────────────────
        # 原始路点字典列表（来自解析器）
        self.waypoints: list = []
        # 转换后的局部坐标路点：[(x_m, y_m, attribute), ...]
        self.cart_waypoints: list = []
        # 路网图对象（用于重规划）
        self.road_network: RoadNetwork = None
        # 局部规划器当前执行到的路点序号（由反馈话题更新）
        self.current_wp_idx: int = 0

        # ── 初始化：加载路点与路网，立即发布路径 ────────────────────────────
        self._load_waypoints()
        self._load_road_network()

        if self.cart_waypoints:
            self._publish_path(self.cart_waypoints)
            self.get_logger().info(
                f'路径规划器启动完成：共 {len(self.cart_waypoints)} 个路点，'
                f'路网节点数 {self.road_network.num_nodes if self.road_network else 0}')
        else:
            self.get_logger().warn('路点为空，请检查 waypoint_file 参数路径是否正确')

    # ══════════════════════════════════════════════════════════════════════
    # 初始化辅助方法
    # ══════════════════════════════════════════════════════════════════════

    def _load_waypoints(self):
        """加载路点文件并转换为局部直角坐标（米）"""
        wp_file    = self.get_parameter('waypoint_file').value
        use_cart   = self.get_parameter('use_cartesian').value

        try:
            if use_cart:
                # 直角坐标格式（单位厘米），以第一个路点为原点转换到米
                raw = parse_cartesian_waypoint_file(wp_file)
                self.waypoints = raw
                if raw:
                    ox = raw[0]['x'] / 100.0  # 原点 X（米）
                    oy = raw[0]['y'] / 100.0  # 原点 Y（米）
                    for wp in raw:
                        x = wp['x'] / 100.0 - ox
                        y = wp['y'] / 100.0 - oy
                        self.cart_waypoints.append((x, y, wp['attribute']))
            else:
                # 经纬度格式，使用 WGS84 转换到局部 ENU 坐标
                raw = parse_waypoint_file(wp_file)
                self.waypoints = raw
                if raw:
                    for wp in raw:
                        x, y = self._gps_to_local(wp['latitude'], wp['longitude'])
                        self.cart_waypoints.append((x, y, wp['attribute']))

            self.get_logger().info(
                f'已加载 {len(self.waypoints)} 个路点 ({wp_file})')
        except FileNotFoundError:
            self.get_logger().error(f'路点文件不存在: {wp_file}')
        except Exception as e:
            self.get_logger().error(f'路点文件解析异常: {e}')

    def _load_road_network(self):
        """加载路网文件；若未指定则从路点序列自动构建顺序路网"""
        net_file = self.get_parameter('road_network_file').value

        if net_file and os.path.exists(net_file):
            try:
                self.road_network = parse_road_network_file(net_file)
                # 将局部坐标注入路网节点，启用 A* 启发式
                for i, (x, y, _) in enumerate(self.cart_waypoints):
                    self.road_network.set_node_position(i, x, y)
                self.get_logger().info(
                    f'路网文件加载成功: {net_file}，'
                    f'边数 {self.road_network.num_edges}')
            except Exception as e:
                self.get_logger().warn(f'路网文件加载失败，退回自动构建路网: {e}')
                self._build_default_network()
        elif self.waypoints:
            self.get_logger().info('未指定路网文件，从路点序列自动构建顺序路网')
            self._build_default_network()

    def _build_default_network(self):
        """从 cart_waypoints 构建默认顺序路网（相邻路点双向连接）"""
        enriched = [
            {'index': i, 'x': x, 'y': y, 'attribute': attr}
            for i, (x, y, attr) in enumerate(self.cart_waypoints)
        ]
        # 双向连接，支持路线重规划时双向通行
        self.road_network = build_network_from_waypoints(enriched, bidirectional=True)

    def _gps_to_local(self, latitude: float, longitude: float) -> tuple:
        """
        将 GPS 经纬度转换为局部 ENU 坐标（东向 +X，北向 +Y，单位米）
        以第一个路点为原点，使用 WGS84 大地测量模型

        :param latitude:  目标点纬度（度）
        :param longitude: 目标点经度（度）
        :return: (x_east_m, y_north_m)
        """
        if not self.waypoints:
            return 0.0, 0.0

        origin = self.waypoints[0]
        geod   = Geodesic.WGS84
        # Inverse：计算两点间大地线（azimuth 从北顺时针，s12 为距离）
        result = geod.Inverse(
            origin['latitude'], origin['longitude'],
            latitude,           longitude)

        # azi1：起点处大地线方位角（北偏东，度），顺时针为正
        azi_rad = math.radians(result['azi1'])
        x = result['s12'] * math.sin(azi_rad)   # 东向分量（经度方向）
        y = result['s12'] * math.cos(azi_rad)   # 北向分量（纬度方向）
        return x, y

    # ══════════════════════════════════════════════════════════════════════
    # 路径发布
    # ══════════════════════════════════════════════════════════════════════

    def _publish_path(self, waypoints: list, start_idx: int = 0):
        """
        将路点列表打包为 nav_msgs/Path 并发布

        属性值编码规则：path.poses[i].pose.position.z = float(attribute)
        局部规划器通过 int(round(z)) 还原整数属性。

        :param waypoints:  [(x_m, y_m, attribute), ...]
        :param start_idx:  从哪个路点开始发布（重规划时跳过已走过的路点）
        """
        path_msg = Path()
        path_msg.header.stamp    = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'

        for x, y, attr in waypoints[start_idx:]:
            pose = PoseStamped()
            pose.header           = path_msg.header
            pose.pose.position.x  = float(x)
            pose.pose.position.y  = float(y)
            pose.pose.position.z  = float(attr)   # 属性编码到 z
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.pub_path.publish(path_msg)

    # ══════════════════════════════════════════════════════════════════════
    # 订阅回调
    # ══════════════════════════════════════════════════════════════════════

    def _progress_callback(self, msg: Int32):
        """更新局部规划器反馈的当前路点进度"""
        self.current_wp_idx = msg.data

    def _replan_callback(self, msg: Int32):
        """
        处理重规划请求

        当局部规划器判断前方路线被长时间堵塞时，
        发布此请求（msg.data = 目标路点序号）。
        本节点将使用路网图的 A* 算法，
        从当前进度路点搜索到目标路点的绕行路径，并重新发布。

        :param msg: Int32，值为目标路点序号
        """
        target_idx  = msg.data
        current_idx = self.current_wp_idx

        if self.road_network is None:
            self.get_logger().warn('路网未加载，无法执行重规划')
            return

        if target_idx <= current_idx:
            self.get_logger().warn(
                f'重规划目标路点 {target_idx} ≤ 当前进度 {current_idx}，忽略')
            return

        self.get_logger().info(
            f'收到重规划请求：路点 {current_idx} → {target_idx}')

        # 优先使用 A*（更快），失败时退回 Dijkstra
        node_path = self.road_network.astar(current_idx, target_idx)
        if not node_path:
            node_path = self.road_network.dijkstra(current_idx, target_idx)

        if not node_path:
            self.get_logger().error(
                f'路网中无法找到 {current_idx} → {target_idx} 的可行路径，保持原路线')
            return

        # 将路网节点序列转换为坐标路点列表
        replanned = []
        for node_id in node_path:
            if node_id < len(self.cart_waypoints):
                # 路网节点对应已知路点，直接取坐标和属性
                replanned.append(self.cart_waypoints[node_id])
            else:
                # 路网中存在不在路点文件中的节点（如路网文件有额外节点）
                pos = self.road_network.node_positions.get(node_id)
                if pos:
                    replanned.append((pos[0], pos[1], 1))   # 默认直行属性

        if replanned:
            self._publish_path(replanned)
            self.get_logger().info(
                f'重规划完成：新路径经过 {len(replanned)} 个节点')


def main(args=None):
    rclpy.init(args=args)
    node = PathPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
