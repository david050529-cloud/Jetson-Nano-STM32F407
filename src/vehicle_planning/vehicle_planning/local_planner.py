# local_planner.py
"""
局部路径规划与底层控制节点(LocalPlanner)

核心功能：
  1. 路径跟踪   — 纯追踪算法(Pure Pursuit),跟踪全局路径
  2. 属性行为   — 根据路点属性执行直行/转弯/换道/超车/掉头/泊车等动作
  3. 交通标志   — 检测红灯/停止牌后自动停车；绿灯或超时后恢复行驶
  4. 障碍物避障 — 基于激光雷达8扇区最短距离,执行绕行/紧急制动/超车
  5. 路线重规划 — 前方长时间堵塞时发布重规划请求给 path_planner
  6. 底层指令   — 将速度/转向指令映射为电机转速和舵机角度

驾驶状态机(DriveState):
  NORMAL         → 正常路径跟踪(纯追踪)
  TURN_ACTION    → 执行属性触发的转弯/掉头动作
  LANE_CHANGE    → 执行换道动作(左/右换道，三阶段)
  OVERTAKE       → 执行超车动作(左绕行→超越→归位，四阶段)
  TRAFFIC_STOP   → 因交通标志停车等待
  PARK           → 低速泊车
  EMERGENCY_STOP → 紧急制动(障碍物极近)
  FINISHED       → 到达终点，停止所有输出

话题：
  订阅：/planning/global_path         (nav_msgs/Path)          — 全局路径(含属性)
        /localization/odometry         (nav_msgs/Odometry)       — 里程计位姿
        /scan                          (sensor_msgs/LaserScan)   — 激光雷达扫描
        /perception/road_signs         (vision_msgs/Detection2DArray)
                                       — 道路信号: 斑马线 Zebra + 红/绿/黄交通灯
  发布：/motor_commands               (stm32_serial_bridge/MotorCommand) — 电机舵机
        /planning/current_waypoint_index (std_msgs/Int32)        — 路点进度反馈
        /planning/replan_request       (std_msgs/Int32)          — 重规划请求
"""

import math
from enum import Enum

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import Int32

from stm32_serial_bridge.msg import MotorCommand


# ══════════════════════════════════════════════════════════════════════════
# 路点属性常量（与 waypoint_parser.py 保持一致）
# ══════════════════════════════════════════════════════════════════════════
ATTR_UNKNOWN     = 0   # 未知，按直行处理
ATTR_STRAIGHT    = 1   # 直行
ATTR_TURN_RIGHT  = 2   # 右转
ATTR_TURN_LEFT   = 3   # 左转
ATTR_LANE_LEFT   = 4   # 左换道
ATTR_LANE_RIGHT  = 5   # 右换道
ATTR_OVERTAKE    = 6   # 超车
ATTR_U_TURN      = 7   # 掉头
ATTR_PARK        = 8   # 泊车

# ══════════════════════════════════════════════════════════════════════════
# 交通信号类别名称（须与 best.pt 模型输出的 class_id 字符串一致）
# best.pt 类别: {0: 'Zebra', 1: 'green', 2: 'red', 3: 'yellow'}
# ══════════════════════════════════════════════════════════════════════════
SIGN_RED_LIGHT    = 'red'           # 红灯
SIGN_GREEN_LIGHT  = 'green'         # 绿灯
SIGN_YELLOW_LIGHT = 'yellow'        # 黄灯
SIGN_ZEBRA        = 'Zebra'         # 斑马线 / 人行横道

# ══════════════════════════════════════════════════════════════════════════
# 驾驶状态机枚举
# ══════════════════════════════════════════════════════════════════════════
class DriveState(Enum):
    NORMAL         = 'normal'          # 正常路径跟踪
    TURN_ACTION    = 'turn_action'     # 执行转向/掉头动作
    LANE_CHANGE    = 'lane_change'     # 执行换道动作
    OVERTAKE       = 'overtake'        # 执行超车动作
    TRAFFIC_STOP   = 'traffic_stop'    # 交通标志停车（内部状态，由 traffic_stop_active 驱动）
    PARK           = 'park'            # 泊车
    EMERGENCY_STOP = 'emergency_stop'  # 紧急制动
    FINISHED       = 'finished'        # 任务完成

# ══════════════════════════════════════════════════════════════════════════
# 激光雷达扇区索引定义（8个扇区，每个45°，0°=正前方）
# 扇区按逆时针排列，与 ROS 角度规范一致（左为正）
# ══════════════════════════════════════════════════════════════════════════
SECTOR_FRONT       = 0   # 正前方      (-22.5° ~ +22.5°)
SECTOR_FRONT_LEFT  = 1   # 左前方      (+22.5° ~ +67.5°)
SECTOR_LEFT        = 2   # 正左方      (+67.5° ~ +112.5°)
SECTOR_REAR_LEFT   = 3   # 左后方      (+112.5° ~ +157.5°)
SECTOR_REAR        = 4   # 正后方      (±157.5° ~ ±180°)
SECTOR_REAR_RIGHT  = 5   # 右后方      (-157.5° ~ -112.5°)
SECTOR_RIGHT       = 6   # 正右方      (-112.5° ~ -67.5°)
SECTOR_FRONT_RIGHT = 7   # 右前方      (-67.5° ~ -22.5°)


class LocalPlanner(Node):
    """局部规划与底层控制节点（状态机架构）"""

    # ── 速度参数（m/s）─────────────────────────────────────────────────────
    SPEED_NORMAL      = 0.8   # 铺装路面正常行驶速度
    SPEED_OFFROAD     = 0.3   # 非铺装（越野）路面最大速度
    SPEED_TURN        = 0.35  # 执行转弯时的速度
    SPEED_LANE_CHANGE = 0.5   # 换道时的速度
    SPEED_PARK        = 0.2   # 泊车时的速度
    SPEED_SLOW        = 0.25  # 接近障碍物时的减速目标

    # ── 距离阈值（m）──────────────────────────────────────────────────────
    WAYPOINT_REACH_DIST = 1.5   # 判定"到达路点"的半径
    OBSTACLE_EMERGENCY  = 0.4   # 触发紧急制动的前方障碍物距离
    OBSTACLE_AVOID      = 1.0   # 触发主动避障的前方障碍物距离
    OBSTACLE_SLOW       = 2.0   # 开始减速的前方障碍物距离
    OVERTAKE_CLEAR_DIST = 1.8   # 判定侧方净空的最小安全距离
    REPLAN_BLOCK_TIME   = 5.0   # 持续被堵超过此时间（秒）后触发重规划请求

    # ── 控制参数 ─────────────────────────────────────────────────────────
    STEERING_KP    = 2.0   # 转向 P 控制增益（增大则转向更激进）
    MAX_STEERING   = 0.6   # 最大转向角速度（rad/s），对应舵机最大偏转
    LOOKAHEAD_DIST = 2.0   # 纯追踪算法前视距离（m）；增大则路径跟踪更平滑

    def __init__(self):
        super().__init__('local_planner')

        # ── 订阅器 ────────────────────────────────────────────────────────
        self.sub_path = self.create_subscription(
            Path, '/planning/global_path', self._path_callback, 10)
        self.sub_odom = self.create_subscription(
            Odometry, '/localization/odometry', self._odom_callback, 10)
        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self._scan_callback, 10)
        # 道路信号(斑马线 + 红/绿/黄交通灯)，由 road_detector 基于 best.pt 发布
        self.sub_signs = self.create_subscription(
            Detection2DArray, '/perception/road_signs',
            self._sign_callback, 10)

        # ── 发布器 ────────────────────────────────────────────────────────
        self.pub_cmd     = self.create_publisher(MotorCommand, '/motor_commands', 10)
        self.pub_wp_idx  = self.create_publisher(
            Int32, '/planning/current_waypoint_index', 10)
        self.pub_replan  = self.create_publisher(
            Int32, '/planning/replan_request', 10)

        # ── 控制定时器（10 Hz）────────────────────────────────────────────
        self.timer = self.create_timer(0.1, self._control_loop)

        # ── 路径与位姿 ────────────────────────────────────────────────────
        # 全局路径点列表：[(x_m, y_m, attribute), ...]
        self.global_path: list  = []
        self.current_pose       = None
        self.current_wp_idx: int = 0   # 当前追踪的目标路点索引

        # ── 激光扇区距离（8个方向，float('inf')=该方向无障碍）────────────────
        self.sector_dists: list = [float('inf')] * 8

        # ── 道路与速度 ────────────────────────────────────────────────────
        # best.pt 不再判断铺装/非铺装，速度上限统一使用 SPEED_NORMAL；
        # 若未来恢复路面分类，可在此重新引入 road_type 与 SPEED_OFFROAD。
        self.max_speed  = self.SPEED_NORMAL

        # ── 交通标志状态 ──────────────────────────────────────────────────
        self.traffic_stop_active: bool  = False   # 是否处于停车等待状态
        self.traffic_stop_timer: float  = 0.0     # 等待计时器（秒）
        self.traffic_stop_max: float    = 3.0     # 最长等待时间（秒），红灯30秒
        self.detected_signs: list       = []      # 当前帧检测到的标志列表

        # ── 驾驶状态机 ────────────────────────────────────────────────────
        self.state       = DriveState.NORMAL
        self.state_timer = 0.0    # 当前状态已持续时间（秒）
        self.block_timer = 0.0    # 被前方障碍物持续阻塞的时间（秒）

        # 换道/超车共用方向：+1=左，-1=右
        self.action_side:  int = +1
        # 超车/换道动作阶段索引（不同状态下含义不同）
        self.action_phase: int = 0

        self.get_logger().info('局部规划控制节点已启动')

    # ══════════════════════════════════════════════════════════════════════
    # 话题回调函数
    # ══════════════════════════════════════════════════════════════════════

    def _path_callback(self, msg: Path):
        """
        接收全局路径
        从 PoseStamped.pose.position.z 中还原路点属性值（int）
        收到新路径后重置路点索引和状态机
        """
        self.global_path = []
        for pose in msg.poses:
            x    = pose.pose.position.x
            y    = pose.pose.position.y
            attr = int(round(pose.pose.position.z))   # z 中编码的属性
            self.global_path.append((x, y, attr))

        # 重置追踪状态（保留交通标志停车状态）
        self.current_wp_idx = 0
        if self.state not in (DriveState.FINISHED, DriveState.TRAFFIC_STOP):
            self.state       = DriveState.NORMAL
            self.state_timer = 0.0

        self.get_logger().info(
            f'收到新路径，共 {len(self.global_path)} 个路点')

    def _odom_callback(self, msg: Odometry):
        """保存最新里程计位姿"""
        self.current_pose = msg.pose.pose

    def _scan_callback(self, msg: LaserScan):
        """
        处理激光雷达扫描数据,更新8个扇区的最短障碍物距离

        扇区划分(以机器人正前方为0°,逆时针为正):
          扇区0:正前方    [-22.5°, +22.5°)
          扇区1:左前方    [+22.5°, +67.5°)
          扇区2:正左方    [+67.5°, +112.5°)
          扇区3:左后方    [+112.5°, +157.5°)
          扇区4:正后方    [+157.5°, -157.5°)(跨越±180°)
          扇区5:右后方    [-157.5°, -112.5°)
          扇区6:正右方    [-112.5°, -67.5°)
          扇区7:右前方    [-67.5°, -22.5°)
        """
        new_dists = [float('inf')] * 8
        angle = msg.angle_min   # 当前点对应的角度（弧度）

        for r in msg.ranges:
            if msg.range_min < r < msg.range_max and math.isfinite(r):
                # 将角度归一化到 [-π, π]
                a = math.atan2(math.sin(angle), math.cos(angle))
                # 转换为 [0°, 360°) 范围，0°=正前方
                a_deg = math.degrees(a) % 360.0
                # 每个扇区跨45°，偏移22.5°使扇区0对称于正前方
                sector = int((a_deg + 22.5) / 45.0) % 8
                if r < new_dists[sector]:
                    new_dists[sector] = r
            angle += msg.angle_increment

        self.sector_dists = new_dists

    def _sign_callback(self, msg: Detection2DArray):
        """
        接收道路信号检测结果（来自 road_detector + best.pt）
        仅保留置信度 > 0.6 的检测，结果在下一控制周期被 _process_traffic_signs 消费
        """
        self.detected_signs = []
        for det in msg.detections:
            for hyp in det.results:
                if hyp.hypothesis.score > 0.6:
                    self.detected_signs.append({
                        'class': hyp.hypothesis.class_id,
                        'score': hyp.hypothesis.score,
                        # 检测框中心像素坐标（仅供参考，未用于控制决策）
                        'cx': det.bbox.center.position.x,
                        'cy': det.bbox.center.position.y,
                    })

    # ══════════════════════════════════════════════════════════════════════
    # 主控制循环（10 Hz）
    # ══════════════════════════════════════════════════════════════════════

    def _control_loop(self):
        """
        主控制循环，执行顺序：
          1. 数据就绪检查
          2. 交通标志处理（最高优先级，可覆盖其他状态）
          3. 更新路点追踪进度
          4. 终点检测
          5. 状态机分派（根据当前状态调用对应处理函数）
        """
        dt = 0.1   # 控制周期（秒）

        # ── 1. 数据就绪检查 ───────────────────────────────────────────────
        if self.current_pose is None or not self.global_path:
            return

        # ── 2. 终点已到达 ─────────────────────────────────────────────────
        if self.state == DriveState.FINISHED:
            self._publish_stop()
            return

        # ── 3. 交通标志处理（优先于其他状态，可中断行驶） ─────────────────
        self._process_traffic_signs(dt)
        if self.traffic_stop_active:
            self._publish_stop()
            return

        # ── 4. 更新路点进度（检查是否到达当前目标路点）──────────────────
        self._update_waypoint_progress()

        # 路点全部通过，到达终点
        if self.current_wp_idx >= len(self.global_path):
            self.state = DriveState.FINISHED
            self._publish_stop()
            self.get_logger().info('已到达终点，任务完成！')
            return

        # ── 5. 状态机分派 ─────────────────────────────────────────────────
        self.state_timer += dt

        if   self.state == DriveState.EMERGENCY_STOP:
            self._handle_emergency_stop()
        elif self.state == DriveState.TURN_ACTION:
            self._handle_turn(dt)
        elif self.state == DriveState.LANE_CHANGE:
            self._handle_lane_change(dt)
        elif self.state == DriveState.OVERTAKE:
            self._handle_overtake(dt)
        elif self.state == DriveState.PARK:
            self._handle_park()
        else:
            # NORMAL 状态：路径跟踪 + 动态避障
            self._handle_normal(dt)

    # ══════════════════════════════════════════════════════════════════════
    # 辅助计算方法
    # ══════════════════════════════════════════════════════════════════════

    def _get_xy(self) -> tuple:
        """获取当前位置坐标 (x, y)（单位：米）"""
        return (self.current_pose.position.x,
                self.current_pose.position.y)

    def _get_yaw(self) -> float:
        """从四元数姿态提取偏航角（yaw，弧度，范围 [-π, π]）"""
        q = self.current_pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _dist_to_wp(self, idx: int) -> float:
        """计算当前位置到 global_path[idx] 路点的欧氏距离（米）"""
        px, py = self._get_xy()
        wx, wy, _ = self.global_path[idx]
        return math.hypot(wx - px, wy - py)

    def _compute_steering(self, tx: float, ty: float) -> float:
        """
        计算指向目标点 (tx, ty) 所需的转向角速度（rad/s）
        使用 P 控制：steering = kp × angle_error，并限幅到 ±MAX_STEERING

        :return: 转向角速度（正值左转，负值右转）
        """
        px, py = self._get_xy()
        yaw    = self._get_yaw()
        # 目标方向角
        angle_to_target = math.atan2(ty - py, tx - px)
        # 计算角度误差，归一化到 [-π, π] 避免360°跳变
        error = angle_to_target - yaw
        error = math.atan2(math.sin(error), math.cos(error))
        # P 控制并限幅
        steering = max(-self.MAX_STEERING,
                       min(self.MAX_STEERING, error * self.STEERING_KP))
        return steering

    def _find_lookahead_target(self) -> tuple:
        """
        纯追踪（Pure Pursuit）目标点搜索：
        在全局路径上，从 current_wp_idx 开始向后搜索，
        找到第一个距当前位置超过 LOOKAHEAD_DIST 的路点作为追踪目标。
        若所有后续路点都在前视圆内，则返回最后一个路点。

        :return: (x, y, attribute)
        """
        px, py = self._get_xy()
        for i in range(self.current_wp_idx, len(self.global_path)):
            wx, wy, attr = self.global_path[i]
            if math.hypot(wx - px, wy - py) >= self.LOOKAHEAD_DIST:
                return wx, wy, attr
        # 所有路点均在前视圆内，返回最后一个
        return self.global_path[-1]

    # ══════════════════════════════════════════════════════════════════════
    # 路点进度管理
    # ══════════════════════════════════════════════════════════════════════

    def _update_waypoint_progress(self):
        """
        检查是否已到达当前目标路点（距离 < WAYPOINT_REACH_DIST）
        到达后：
          1. 读取该路点的属性，触发对应驾驶行为
          2. 路点索引前进到下一个
          3. 向 path_planner 发布进度反馈
        """
        while self.current_wp_idx < len(self.global_path):
            if self._dist_to_wp(self.current_wp_idx) < self.WAYPOINT_REACH_DIST:
                _, _, attr = self.global_path[self.current_wp_idx]
                self._trigger_waypoint_action(attr)
                self.current_wp_idx += 1

                # 反馈当前进度给 path_planner（用于重规划定位）
                idx_msg      = Int32()
                idx_msg.data = self.current_wp_idx
                self.pub_wp_idx.publish(idx_msg)
            else:
                break  # 尚未到达，等待下一个控制周期

    def _trigger_waypoint_action(self, attr: int):
        """
        根据路点属性值触发对应的驾驶行为状态切换
        只在 NORMAL 或 TURN_ACTION 状态下触发，避免中断正在执行的动作

        属性值 → 行为映射：
          1（直行）：继续 NORMAL，无特殊动作
          2（右转）：切换 TURN_ACTION，向右转向
          3（左转）：切换 TURN_ACTION，向左转向
          4（左换道）：切换 LANE_CHANGE，左侧换道
          5（右换道）：切换 LANE_CHANGE，右侧换道
          6（超车）：切换 OVERTAKE，从左侧超车
          7（掉头）：切换 TURN_ACTION（大角度左转）
          8（泊车）：切换 PARK
        """
        # 不在可打断的状态中则跳过
        if self.state not in (DriveState.NORMAL, DriveState.TURN_ACTION):
            return

        if attr in (ATTR_STRAIGHT, ATTR_UNKNOWN):
            # 直行或未知：无需切换状态
            return

        elif attr == ATTR_TURN_RIGHT:
            self.get_logger().info('路点行为：右转')
            self.state       = DriveState.TURN_ACTION
            self.action_side = -1     # 右转方向为负（右）
            self.state_timer = 0.0

        elif attr == ATTR_TURN_LEFT:
            self.get_logger().info('路点行为：左转')
            self.state       = DriveState.TURN_ACTION
            self.action_side = +1     # 左转方向为正（左）
            self.state_timer = 0.0

        elif attr == ATTR_LANE_LEFT:
            self.get_logger().info('路点行为：左换道')
            self.state        = DriveState.LANE_CHANGE
            self.action_side  = +1    # 向左变道
            self.action_phase = 0
            self.state_timer  = 0.0

        elif attr == ATTR_LANE_RIGHT:
            self.get_logger().info('路点行为：右换道')
            self.state        = DriveState.LANE_CHANGE
            self.action_side  = -1    # 向右变道
            self.action_phase = 0
            self.state_timer  = 0.0

        elif attr == ATTR_OVERTAKE:
            self.get_logger().info('路点行为：超车')
            self.state        = DriveState.OVERTAKE
            self.action_side  = +1    # 从左侧超车
            self.action_phase = 0
            self.state_timer  = 0.0

        elif attr == ATTR_U_TURN:
            self.get_logger().info('路点行为：掉头')
            self.state       = DriveState.TURN_ACTION
            self.action_side = +1     # 掉头方向（左转）
            self.state_timer = 0.0

        elif attr == ATTR_PARK:
            self.get_logger().info('路点行为：泊车')
            self.state       = DriveState.PARK
            self.state_timer = 0.0

    # ══════════════════════════════════════════════════════════════════════
    # 交通标志处理
    # ══════════════════════════════════════════════════════════════════════

    def _process_traffic_signs(self, dt: float):
        """
        处理道路信号检测结果(best.pt 输出)，更新停车等待状态

        停车触发条件：
          - 检测到红灯(SIGN_RED_LIGHT)   ：停车,等待绿灯或超时(30秒)
          - 检测到黄灯(SIGN_YELLOW_LIGHT)：保守起见停车 5 秒
          - 检测到斑马线(SIGN_ZEBRA)     ：礼让行人,停车 3 秒后继续

        停车解除条件：
          - 检测到绿灯(SIGN_GREEN_LIGHT)：立即解除停车
          - 等待超时(traffic_stop_max 秒)：超时解除(防止永久停车)
        """
        if self.traffic_stop_active:
            # 当前处于停车等待，累计等待时间
            self.traffic_stop_timer += dt

            # 检查是否出现绿灯（立即通行）
            if any(s['class'] == SIGN_GREEN_LIGHT for s in self.detected_signs):
                self.traffic_stop_active = False
                self.traffic_stop_timer  = 0.0
                self.get_logger().info('检测到绿灯，恢复行驶')
                return

            # 超时保护：避免永久停车
            if self.traffic_stop_timer >= self.traffic_stop_max:
                self.traffic_stop_active = False
                self.traffic_stop_timer  = 0.0
                self.get_logger().info(
                    f'停车超时 ({self.traffic_stop_max:.0f}s)，强制恢复行驶')
        else:
            # 检查是否需要触发停车（红灯优先级高于黄灯/斑马线）
            classes = {s['class'] for s in self.detected_signs}
            if SIGN_RED_LIGHT in classes:
                self.traffic_stop_active = True
                self.traffic_stop_timer  = 0.0
                self.traffic_stop_max    = 30.0   # 红灯最长等待30秒
                self.get_logger().info('检测到红灯，停车等待')
            elif SIGN_YELLOW_LIGHT in classes:
                self.traffic_stop_active = True
                self.traffic_stop_timer  = 0.0
                self.traffic_stop_max    = 5.0    # 黄灯保守停车 5 秒
                self.get_logger().info('检测到黄灯，停车等待')
            elif SIGN_ZEBRA in classes:
                self.traffic_stop_active = True
                self.traffic_stop_timer  = 0.0
                self.traffic_stop_max    = 3.0    # 斑马线礼让 3 秒
                self.get_logger().info('检测到斑马线，礼让行人停车 3 秒')

    # ══════════════════════════════════════════════════════════════════════
    # 状态机处理函数
    # ══════════════════════════════════════════════════════════════════════

    def _handle_normal(self, dt: float):
        """
        NORMAL 状态：正常路径跟踪，含动态障碍物避障

        处理逻辑优先级（从高到低）：
          1. 紧急制动：前方 < OBSTACLE_EMERGENCY，直接制动
          2. 主动避障：前方 < OBSTACLE_AVOID，选择净空侧绕行
          3. 减速区   ：前方 < OBSTACLE_SLOW，线性减速
          4. 正常跟踪 ：纯追踪目标点，全速行驶

        阻塞超时：
          若前方持续被堵 REPLAN_BLOCK_TIME 秒，发布重规划请求
        """
        front       = self.sector_dists[SECTOR_FRONT]
        front_left  = self.sector_dists[SECTOR_FRONT_LEFT]
        front_right = self.sector_dists[SECTOR_FRONT_RIGHT]
        left        = self.sector_dists[SECTOR_LEFT]
        right       = self.sector_dists[SECTOR_RIGHT]

        # ── 1. 紧急制动 ────────────────────────────────────────────────
        if front < self.OBSTACLE_EMERGENCY:
            self.state       = DriveState.EMERGENCY_STOP
            self.state_timer = 0.0
            self._publish_stop()
            self.get_logger().warn(
                f'紧急制动！前方障碍 {front:.2f}m')
            return

        # ── 2. 主动避障（前方 < OBSTACLE_AVOID）──────────────────────
        if front < self.OBSTACLE_AVOID:
            self.block_timer += dt

            # 阻塞超时，请求路线重规划
            if self.block_timer > self.REPLAN_BLOCK_TIME:
                replan_msg      = Int32()
                replan_msg.data = len(self.global_path) - 1   # 重规划到终点
                self.pub_replan.publish(replan_msg)
                self.block_timer = 0.0   # 重置计时，避免重复请求
                self.get_logger().warn(
                    f'前方阻塞 >{self.REPLAN_BLOCK_TIME:.0f}s，已发出重规划请求')

            # 选择净空侧进行横向绕行
            if front_left > self.OVERTAKE_CLEAR_DIST:
                # 左侧有足够净空，向左绕行
                self._publish_cmd(self.SPEED_SLOW, self.MAX_STEERING * 0.5)
            elif front_right > self.OVERTAKE_CLEAR_DIST:
                # 右侧有足够净空，向右绕行
                self._publish_cmd(self.SPEED_SLOW, -self.MAX_STEERING * 0.5)
            else:
                # 前方及两侧均被堵：原地等待
                self._publish_stop()
            return

        # 前方畅通，重置阻塞计时器
        self.block_timer = 0.0

        # ── 3. 纯追踪路径跟踪 ─────────────────────────────────────────
        tx, ty, _ = self._find_lookahead_target()
        steering   = self._compute_steering(tx, ty)

        # 根据前方距离线性渐进减速（避免突然减速）
        speed = self.max_speed
        if front < self.OBSTACLE_SLOW:
            # 线性插值：front=OBSTACLE_AVOID → SPEED_SLOW，front=OBSTACLE_SLOW → max_speed
            factor = ((front - self.OBSTACLE_AVOID) /
                      (self.OBSTACLE_SLOW - self.OBSTACLE_AVOID))
            factor = max(0.0, min(1.0, factor))
            speed  = self.SPEED_SLOW + factor * (self.max_speed - self.SPEED_SLOW)

        self._publish_cmd(speed, steering)

    def _handle_emergency_stop(self):
        """
        EMERGENCY_STOP 状态：持续制动
        当前方净空超过 OBSTACLE_AVOID × 1.5 时自动恢复 NORMAL
        """
        self._publish_stop()

        # 检测前方净空，净空时恢复行驶
        if self.sector_dists[SECTOR_FRONT] > self.OBSTACLE_AVOID * 1.5:
            self.state       = DriveState.NORMAL
            self.state_timer = 0.0
            self.get_logger().info('前方净空，解除紧急制动')

    def _handle_turn(self, dt: float):
        """
        TURN_ACTION 状态：执行属性触发的转弯或掉头动作

        - 右转 / 左转（attr=2/3）：降速，施加 70% 最大转向，持续 2.5 秒
        - 掉头（attr=7）：降速，施加 100% 最大转向，持续 5 秒
        转向完成后自动切回 NORMAL 状态
        """
        # 读取当前或下一个路点的属性，判断是否为掉头
        wp_idx  = min(self.current_wp_idx, len(self.global_path) - 1)
        attr    = self.global_path[wp_idx][2]
        is_uturn = (attr == ATTR_U_TURN)

        turn_duration = 5.0 if is_uturn else 2.5
        # 掉头使用全转向，普通转弯使用70%转向
        steer_ratio   = 1.0 if is_uturn else 0.7
        steering = self.action_side * self.MAX_STEERING * steer_ratio

        self._publish_cmd(self.SPEED_TURN, steering)

        if self.state_timer >= turn_duration:
            self.state       = DriveState.NORMAL
            self.state_timer = 0.0
            self.get_logger().info('转向动作完成，切回正常行驶')

    def _handle_lane_change(self, dt: float):
        """
        LANE_CHANGE 状态：三阶段换道动作

        阶段 0（偏移阶段，2.0s）：以 50% 最大转向横向移动到目标车道
        阶段 1（回正阶段，1.0s）：切回纯追踪，稳定在新车道
        完成后切回 NORMAL

        action_side：+1=向左，-1=向右
        """
        PHASE_DUR = [2.0, 1.0]   # 各阶段持续时间（秒）

        if self.action_phase == 0:
            # 横向偏移阶段
            steering = self.action_side * self.MAX_STEERING * 0.5
            self._publish_cmd(self.SPEED_LANE_CHANGE, steering)
            if self.state_timer >= PHASE_DUR[0]:
                self.action_phase = 1
                self.state_timer  = 0.0

        else:
            # 回正稳定阶段：切换为纯追踪跟随路径
            tx, ty, _ = self._find_lookahead_target()
            steering   = self._compute_steering(tx, ty)
            self._publish_cmd(self.SPEED_LANE_CHANGE, steering)
            if self.state_timer >= PHASE_DUR[1]:
                self.state       = DriveState.NORMAL
                self.state_timer = 0.0
                self.get_logger().info('换道完成，切回正常行驶')

    def _handle_overtake(self, dt: float):
        """
        OVERTAKE 状态：四阶段超车动作

        阶段 0（左换道，2.0s）：向左偏移，检查左侧净空；不安全则中止
        阶段 1（超越阶段，3.0s）：纯追踪加速超越，仍检测前方障碍
        阶段 2（归位阶段，2.0s）：向右归位到原车道
        完成后切回 NORMAL

        左侧净空检查：若左方（SECTOR_LEFT）< OVERTAKE_CLEAR_DIST 则中止超车
        """
        PHASE_DUR = [2.0, 3.0, 2.0]

        front = self.sector_dists[SECTOR_FRONT]
        left  = self.sector_dists[SECTOR_LEFT]

        if self.action_phase == 0:
            # ── 阶段0：向左换道 ──────────────────────────────────────
            if left < self.OVERTAKE_CLEAR_DIST:
                # 左侧不安全，中止超车，退回正常避障
                self.state       = DriveState.NORMAL
                self.state_timer = 0.0
                self.get_logger().warn(
                    f'左侧净空不足 ({left:.2f}m)，中止超车')
                return
            steering = self.MAX_STEERING * 0.5   # 向左转向
            self._publish_cmd(self.SPEED_LANE_CHANGE, steering)
            if self.state_timer >= PHASE_DUR[0]:
                self.action_phase = 1
                self.state_timer  = 0.0

        elif self.action_phase == 1:
            # ── 阶段1：加速超越目标车辆 ─────────────────────────────
            if front < self.OBSTACLE_EMERGENCY:
                # 超车过程中出现新障碍，紧急制动
                self._publish_stop()
                self.state       = DriveState.EMERGENCY_STOP
                self.state_timer = 0.0
                return
            # 纯追踪前视目标
            tx, ty, _ = self._find_lookahead_target()
            steering  = self._compute_steering(tx, ty)
            # 超车阶段不超过正常速度上限（避免超速）
            self._publish_cmd(self.max_speed, steering)
            if self.state_timer >= PHASE_DUR[1]:
                self.action_phase = 2
                self.state_timer  = 0.0

        else:
            # ── 阶段2：向右归位到原车道 ─────────────────────────────
            right = self.sector_dists[SECTOR_RIGHT]
            if right < self.OBSTACLE_EMERGENCY:
                # 归位时右侧有障碍，暂停归位，继续直行
                tx, ty, _ = self._find_lookahead_target()
                steering  = self._compute_steering(tx, ty)
                self._publish_cmd(self.SPEED_LANE_CHANGE, steering)
            else:
                # 右侧净空，执行归位
                steering = -self.MAX_STEERING * 0.5   # 向右转向
                self._publish_cmd(self.SPEED_LANE_CHANGE, steering)
            if self.state_timer >= PHASE_DUR[2]:
                self.state       = DriveState.NORMAL
                self.state_timer = 0.0
                self.get_logger().info('超车完成，已归位原车道')

    def _handle_park(self):
        """
        PARK 状态：低速接近当前目标路点并停车

        到达路点（距离 < 0.5m）时停止并切换至 FINISHED
        """
        if self.current_wp_idx < len(self.global_path):
            wx, wy, _ = self.global_path[self.current_wp_idx]
            dist = self._dist_to_wp(self.current_wp_idx)

            if dist < 0.5:
                # 到达泊车位
                self._publish_stop()
                self.state = DriveState.FINISHED
                self.get_logger().info('泊车完成，任务结束')
            else:
                # 低速向泊车位靠近，精确转向
                steering = self._compute_steering(wx, wy)
                self._publish_cmd(self.SPEED_PARK, steering)
        else:
            self._publish_stop()
            self.state = DriveState.FINISHED

    # ══════════════════════════════════════════════════════════════════════
    # 指令发布
    # ══════════════════════════════════════════════════════════════════════

    def _publish_cmd(self, linear: float, angular: float):
        """
        将速度/转向指令映射为 MotorCommand 并发布

        电机转速映射：
          线速度 (m/s) → motor1/motor2_target_rps (转/秒)
          实际映射比例需根据车轮半径和减速比标定

        舵机角度映射（Ackermann 转向）：
          angular =  0          → servo_angle = 90°（直行，中位）
          angular = +MAX_STEER  → servo_angle = 180°（最大左转）
          angular = -MAX_STEER  → servo_angle =   0°（最大右转）

        :param linear:  期望线速度（m/s），负值时发送0（不支持倒车）
        :param angular: 期望转向角速度（rad/s）
        """
        cmd = MotorCommand()

        # 线速度映射（当前简单等比，根据实际轮径调整换算系数）
        rps = max(0.0, float(linear))
        cmd.motor1_target_rps = rps
        cmd.motor2_target_rps = rps

        # 舵机中位90°，线性映射转向角速度到0~180°
        servo_center = 90
        servo_range  = 90   # 从中位到极值的行程（度）
        servo_angle  = servo_center + int(
            (angular / self.MAX_STEERING) * servo_range)
        cmd.servo_angle = max(0, min(180, servo_angle))

        self.pub_cmd.publish(cmd)

    def _publish_stop(self):
        """
        发布停止指令：两个电机速度清零，舵机回中（90°）
        用于所有需要车辆静止的场景
        """
        cmd = MotorCommand()
        cmd.motor1_target_rps = 0.0
        cmd.motor2_target_rps = 0.0
        cmd.servo_angle       = 90
        self.pub_cmd.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = LocalPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
