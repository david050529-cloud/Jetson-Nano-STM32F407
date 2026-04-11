# road_network_parser.py
"""
路网文件解析器
将路网文件解析为图结构，并提供 Dijkstra 和 A* 最短路径搜索算法。
用于当前路线被障碍物堵塞时，在路网地图上重新规划绕行路径。

支持的路网文件格式（每行一条边）：
  格式1(含权重):起点序号  终点序号  权重(float,通常为两点间距离/米)
  格式2(无权重):起点序号  终点序号      (默认权重 1.0)
  以 '#' 开头的行为注释，空行和注释均被忽略

文件示例：
  # 比赛路网文件(无向图)
  0 1 12.5
  1 2  8.0
  1 3 15.3
  2 4  9.7
  3 4 11.0
"""

import os
import math
import heapq
from collections import defaultdict


# ── 路点动作属性常量(与 waypoint.txt 路点文件中的 属性(int) 列对应) ─────────────
ATTR_UNKNOWN     = 0   # 未知，默认直行
ATTR_STRAIGHT    = 1   # 直行
ATTR_TURN_RIGHT  = 2   # 右转
ATTR_TURN_LEFT   = 3   # 左转
ATTR_LANE_LEFT   = 4   # 左换道
ATTR_LANE_RIGHT  = 5   # 右换道
ATTR_OVERTAKE    = 6   # 超车
ATTR_U_TURN      = 7   # 掉头
ATTR_PARK        = 8   # 泊车


class RoadNetwork:
    """
    路网图数据结构（邻接表实现）

    功能：
      - 管理节点连接关系（有向或无向边，带权重）
      - 存储节点坐标（用于 A* 启发式计算）
      - 提供 Dijkstra 和 A* 最短路径算法

    典型用法：
      net = RoadNetwork()
      net.add_edge(0, 1, 12.5)
      path = net.astar(0, 4)   # 返回 [0, 1, 2, 4]
    """

    def __init__(self):
        # 邻接表：{节点ID(int): [(邻居ID, 权重), ...]}
        self.adjacency: dict = defaultdict(list)
        # 节点坐标缓存：{节点ID: (x, y)}，用于 A* 启发函数
        self.node_positions: dict = {}

    # ──────────────────────────────────────────────────────────────────────
    # 图构建接口
    # ──────────────────────────────────────────────────────────────────────

    def add_edge(self, from_id: int, to_id: int,
                 weight: float = 1.0, bidirectional: bool = True):
        """
        添加一条路网边

        :param from_id:       起点节点 ID
        :param to_id:         终点节点 ID
        :param weight:        边权重（通常为两点间实际距离，单位：米）
        :param bidirectional: True = 双向可通行（无向边），False = 单向
        """
        self.adjacency[from_id].append((to_id, weight))
        if bidirectional:
            self.adjacency[to_id].append((from_id, weight))

    def set_node_position(self, node_id: int, x: float, y: float):
        """
        设置节点的局部坐标（米），供 A* 启发式函数使用。
        坐标系应与局部规划器的里程计坐标系一致。
        """
        self.node_positions[node_id] = (x, y)

    # ──────────────────────────────────────────────────────────────────────
    # 路径搜索算法
    # ──────────────────────────────────────────────────────────────────────

    def dijkstra(self, start: int, goal: int) -> list:
        """
        Dijkstra 最短路径算法(适用于所有正权重图，保证最优解)

        算法步骤：
          1. 以 start 为源节点，初始化距离表
          2. 用最小堆每次取出当前最短距离的节点
          3. 松弛相邻边（更新更短路径）
          4. 找到 goal 后回溯前驱节点得到完整路径

        :param start: 起点节点 ID
        :param goal:  终点节点 ID
        :return: 节点 ID 列表(包含 start 和 goal),不可达时返回 []
        """
        # dist[v] 存储从 start 到 v 的最短已知距离
        dist = defaultdict(lambda: float('inf'))
        dist[start] = 0.0
        # prev[v] 存储 v 的最短路径前驱节点（用于最终回溯完整路径）
        prev = {start: None}
        # 优先队列：(已知最短距离, 节点ID)
        heap = [(0.0, start)]

        while heap:
            d, u = heapq.heappop(heap)

            # 跳过已被更优路径替代的过时条目
            if d > dist[u]:
                continue

            # 到达目标，无需继续搜索
            if u == goal:
                break

            # 松弛所有出边：若经过 u 到达 v 的路径更短，更新并加入堆
            for v, w in self.adjacency.get(u, []):
                new_dist = d + w
                if new_dist < dist[v]:
                    dist[v] = new_dist
                    prev[v] = u
                    heapq.heappush(heap, (new_dist, v))

        # 目标不可达（未被加入 prev）
        if goal not in prev:
            return []

        # 从终点沿前驱链回溯至起点，反转得到正向路径
        path, node = [], goal
        while node is not None:
            path.append(node)
            node = prev[node]
        return list(reversed(path))

    def astar(self, start: int, goal: int) -> list:
        """
        A* 路径搜索算法(在 Dijkstra 基础上加入启发函数，搜索效率更高)

        启发函数:欧氏直线距离(h(v) = ||pos(v) - pos(goal)||)
        满足可接受性(不高估),保证最优解。
        若节点坐标未设置，启发值退化为 0(等同于 Dijkstra)。

        :param start: 起点节点 ID
        :param goal:  终点节点 ID
        :return: 节点 ID 列表，不可达时返回 []
        """
        def heuristic(node_id: int) -> float:
            """估算从 node_id 到 goal 的最短距离下界"""
            if node_id not in self.node_positions or goal not in self.node_positions:
                return 0.0  # 无坐标信息时等同 Dijkstra
            ax, ay = self.node_positions[node_id]
            bx, by = self.node_positions[goal]
            return math.hypot(ax - bx, ay - by)

        # g_score[v]：从 start 到 v 的最优实际代价
        g_score = defaultdict(lambda: float('inf'))
        g_score[start] = 0.0
        prev = {start: None}
        # 优先队列：(f = g + h, g, 节点ID)
        # 同时存 g 是为了正确识别过时条目
        heap = [(heuristic(start), 0.0, start)]

        while heap:
            f, g, u = heapq.heappop(heap)

            # 跳过过时条目
            if g > g_score[u]:
                continue

            if u == goal:
                break

            for v, w in self.adjacency.get(u, []):
                new_g = g + w
                if new_g < g_score[v]:
                    g_score[v] = new_g
                    prev[v] = u
                    heapq.heappush(heap, (new_g + heuristic(v), new_g, v))

        if goal not in prev:
            return []

        path, node = [], goal
        while node is not None:
            path.append(node)
            node = prev[node]
        return list(reversed(path))

    # ──────────────────────────────────────────────────────────────────────
    # 图信息查询
    # ──────────────────────────────────────────────────────────────────────

    @property
    def num_nodes(self) -> int:
        """返回图中节点总数"""
        all_nodes = set(self.adjacency.keys())
        for neighbors in self.adjacency.values():
            for v, _ in neighbors:
                all_nodes.add(v)
        return len(all_nodes)

    @property
    def num_edges(self) -> int:
        """返回有向边总数(无向边计为两条)"""
        return sum(len(v) for v in self.adjacency.values())


# ══════════════════════════════════════════════════════════════════════════
# 路网文件解析函数
# ══════════════════════════════════════════════════════════════════════════

def parse_road_network_file(file_path: str, bidirectional: bool = True) -> RoadNetwork:
    """
    解析路网文件，构建 RoadNetwork 图对象

    文件格式说明（每行一条边）：
      起点序号  终点序号  [权重]
      - 序号与路点文件中的序号对应
      - 权重为可选项(float),代表两点间距离(米)
      - 省略权重时默认为 1.0
      - '#' 开头的行为注释，空行忽略
      - 支持行内注释：边数据后加 # 及说明

    :param file_path:     路网文件绝对路径
    :param bidirectional: 是否将每条边解析为双向可通行(默认 True)
    :return: 构建好的 RoadNetwork 对象
    :raises FileNotFoundError: 文件不存在时抛出
    """
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"路网文件未找到: {file_path}")

    network = RoadNetwork()
    edge_count = 0

    with open(file_path, 'r', encoding='utf-8') as f:
        for lineno, raw_line in enumerate(f, 1):
            # 去除行内注释及首尾空白
            line = raw_line.split('#')[0].strip()
            if not line:
                continue  # 跳过空行或纯注释行

            parts = line.split()
            if len(parts) < 2:
                continue  # 数据不足，跳过

            try:
                from_id = int(parts[0])
                to_id   = int(parts[1])
                # 提供了权重则使用，否则默认 1.0
                weight  = float(parts[2]) if len(parts) >= 3 else 1.0
                network.add_edge(from_id, to_id, weight, bidirectional)
                edge_count += 1
            except ValueError:
                # 格式不合法，静默跳过（避免一行错误导致整个文件失败）
                continue

    return network


def build_network_from_waypoints(waypoints: list,
                                  bidirectional: bool = False) -> RoadNetwork:
    """
    从路点列表自动构建顺序路网(相邻路点按序号顺序连接)

    适用场景：比赛未提供独立路网文件时，用路点序列作为回退路网。
    边权重为两点间欧氏距离，节点坐标同步存入图中供 A* 使用。

    :param waypoints:     路点字典列表，每个元素须含：
                            'index' (int),
                            'x'/'longitude' (float),
                            'y'/'latitude'  (float)
    :param bidirectional: 是否双向连接(默认单向，沿路点行驶方向)
    :return: 构建好的 RoadNetwork 对象
    """
    network = RoadNetwork()

    # 建立 ID → 坐标 映射，兼容经纬度字段名和直角坐标字段名
    id_to_pos: dict = {}
    for wp in waypoints:
        idx = wp['index']
        x = wp.get('x', wp.get('longitude', 0.0))
        y = wp.get('y', wp.get('latitude',  0.0))
        id_to_pos[idx] = (x, y)
        network.set_node_position(idx, x, y)

    # 按序号从小到大，连接相邻路点
    sorted_wps = sorted(waypoints, key=lambda w: w['index'])
    for i in range(len(sorted_wps) - 1):
        a_id = sorted_wps[i]['index']
        b_id = sorted_wps[i + 1]['index']
        ax, ay = id_to_pos[a_id]
        bx, by = id_to_pos[b_id]
        # 权重取欧氏距离，至少 0.001 避免零权重
        dist = max(math.hypot(ax - bx, ay - by), 0.001)
        network.add_edge(a_id, b_id, dist, bidirectional)

    return network
