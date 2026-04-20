#!/usr/bin/env python3
"""Nav2 任务执行器 —— 将比赛路点路径提交给 Nav2 NavigateThroughPoses 动作服务器。

工作流程：
  1. 订阅 /planning/global_path（来自 path_planner）
  2. 将路点序列提交给 Nav2 NavigateThroughPoses
  3. 持续监听交通标志（red_light/stop_sign → 暂停，green_light → 恢复）
  4. 监听道路类型，动态调整目标速度（铺装/非铺装路面）
  5. 向 path_planner 反馈当前进度索引

架构说明：
  - 本节点替代 local_planner 的"任务调度"部分
  - 底层跟踪控制由 Nav2 Controller Server (RegulatedPurePursuitController) 负责
  - /cmd_vel 由 cmd_vel_to_motor_node 转换为 STM32 MotorCommand
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from nav2_msgs.action import NavigateThroughPoses
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32, String
from vision_msgs.msg import Detection2DArray


class Nav2MissionExecutor(Node):
    def __init__(self):
        super().__init__('nav2_mission_executor')

        self._cb = ReentrantCallbackGroup()

        # Nav2 NavigateThroughPoses 动作客户端
        self._nav_client = ActionClient(
            self,
            NavigateThroughPoses,
            'navigate_through_poses',
            callback_group=self._cb,
        )

        # ── 内部状态 ─────────────────────────────────────────────────
        self._global_path: Path = None
        self._current_idx = 0
        self._goal_handle = None
        self._traffic_stop = False
        self._stop_timer = None

        # ── 发布 ─────────────────────────────────────────────────────
        self._wp_idx_pub = self.create_publisher(
            Int32, '/planning/current_waypoint_index', 10
        )

        # ── 订阅 ─────────────────────────────────────────────────────
        self.create_subscription(
            Path, '/planning/global_path',
            self._path_cb, 10, callback_group=self._cb,
        )
        self.create_subscription(
            Detection2DArray, '/perception/traffic_signs',
            self._sign_cb, 10, callback_group=self._cb,
        )
        self.create_subscription(
            String, '/perception/road_type',
            self._road_type_cb, 10, callback_group=self._cb,
        )

        self.get_logger().info('Nav2MissionExecutor 已启动，等待全局路径...')

    # ──────────────────────────────────────────────────────────────────
    # 订阅回调
    # ──────────────────────────────────────────────────────────────────

    def _path_cb(self, msg: Path):
        """收到新路径时重置状态并提交 Nav2 目标。"""
        self._global_path = msg
        self._current_idx = 0
        self.get_logger().info(
            f'收到全局路径：{len(msg.poses)} 个路点'
        )
        if not self._traffic_stop:
            self._send_nav_goal()

    def _sign_cb(self, msg: Detection2DArray):
        """交通标志响应：红灯/停止牌→暂停，绿灯→恢复。"""
        for det in msg.detections:
            if not det.results:
                continue
            cls  = det.results[0].hypothesis.class_id
            conf = det.results[0].hypothesis.score
            if conf < 0.6:
                continue

            if cls == 'red_light' and not self._traffic_stop:
                self.get_logger().warn('检测到红灯，暂停导航（等待绿灯）')
                self._traffic_stop = True
                self._cancel_goal()

            elif cls == 'stop_sign' and not self._traffic_stop:
                self.get_logger().warn('检测到停止标志，暂停 3 秒')
                self._traffic_stop = True
                self._cancel_goal()
                # 3 秒后自动恢复，one-shot timer
                self._stop_timer = self.create_timer(3.0, self._resume_after_stop)

            elif cls == 'green_light' and self._traffic_stop:
                self.get_logger().info('绿灯亮起，立即恢复导航')
                self._clear_stop()
                self._send_nav_goal()

    def _road_type_cb(self, msg: String):
        """道路类型变化时记录日志（速度限制由 Nav2 参数动态调整）。"""
        road = msg.data
        if road == 'unpaved':
            self.get_logger().info('检测到非铺装路面，导航保持低速')
        elif road == 'paved':
            self.get_logger().debug('铺装路面，正常速度')

    # ──────────────────────────────────────────────────────────────────
    # Nav2 动作接口
    # ──────────────────────────────────────────────────────────────────

    def _send_nav_goal(self):
        """构建 NavigateThroughPoses 目标并发送。"""
        if self._global_path is None or len(self._global_path.poses) == 0:
            return
        if not self._nav_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('Nav2 动作服务器不可用，跳过本次提交')
            return

        remaining = self._global_path.poses[self._current_idx:]
        if not remaining:
            self.get_logger().info('所有路点已完成，任务结束')
            return

        # 清除路点 z 字段中编码的属性值（Nav2 需要标准 3D 坐标）
        clean_poses = []
        for p in remaining:
            ps = PoseStamped()
            ps.header           = p.header
            ps.pose.position.x  = p.pose.position.x
            ps.pose.position.y  = p.pose.position.y
            ps.pose.position.z  = 0.0
            ps.pose.orientation = p.pose.orientation
            clean_poses.append(ps)

        goal = NavigateThroughPoses.Goal()
        goal.poses        = clean_poses
        goal.behavior_tree = ''  # 使用 nav2_params.yaml 指定的默认 BT

        self.get_logger().info(
            f'提交 Nav2 目标：从路点 {self._current_idx} 起，'
            f'共 {len(clean_poses)} 个路点'
        )
        fut = self._nav_client.send_goal_async(
            goal, feedback_callback=self._feedback_cb
        )
        fut.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Nav2 目标被拒绝')
            return
        self._goal_handle = handle
        self.get_logger().info('Nav2 已接受目标，开始导航')
        handle.get_result_async().add_done_callback(self._result_cb)

    def _feedback_cb(self, feedback_msg):
        """更新当前路点进度并反馈给 path_planner。"""
        remaining_count = feedback_msg.feedback.number_of_poses_remaining
        total = len(self._global_path.poses) if self._global_path else 0
        # 当前已完成的路点数（近似值）
        completed = max(0, total - remaining_count)
        self._current_idx = completed

        idx_msg = Int32()
        idx_msg.data = completed
        self._wp_idx_pub.publish(idx_msg)

    def _result_cb(self, future):
        result = future.result()
        if result.status == 4:  # action_msgs/GoalStatus.STATUS_SUCCEEDED
            self.get_logger().info('Nav2 导航成功完成')
        else:
            self.get_logger().warn(
                f'Nav2 导航结束，GoalStatus={result.status}'
            )

    def _cancel_goal(self):
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None

    def _clear_stop(self):
        if self._stop_timer is not None:
            self._stop_timer.cancel()
            self._stop_timer = None
        self._traffic_stop = False

    def _resume_after_stop(self):
        """停车计时结束后自动恢复。"""
        self._clear_stop()
        self.get_logger().info('停车时间结束，恢复导航')
        self._send_nav_goal()


def main(args=None):
    rclpy.init(args=args)
    node = Nav2MissionExecutor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
