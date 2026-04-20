#!/usr/bin/env python3
"""STM32 遥测数据 → wheel odometry 转换节点。

将 /telemetry 中的电机实际转速（RPS）和偏航角转换为
nav_msgs/Odometry，发布到 /odometry/wheel，供 EKF 融合 vx。

注意：
  - EKF 仅融合 vx（线速度），不融合积分位置（位置由 GPS 提供）
  - yaw 来自 STM32 内置 MPU6050 卡尔曼滤波，质量优于纯积分
  - wheel_radius 需根据实车轮胎直径标定
"""
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from stm32_serial_bridge.msg import Telemetry


class TelemetryToOdomNode(Node):
    def __init__(self):
        super().__init__('telemetry_to_odom_node')

        self.declare_parameter('wheel_radius', 0.05)     # 车轮半径（米）
        self.declare_parameter('wheel_separation', 0.20) # 左右轮距（米，差速用）

        self._r = self.get_parameter('wheel_radius').value
        self._d = self.get_parameter('wheel_separation').value

        # 积分位姿（仅供 pose 字段使用，EKF 不融合此位置）
        self._x = 0.0
        self._y = 0.0
        self._yaw_rad = 0.0

        self._last_time = None
        self._last_yaw_deg = None

        self._odom_pub = self.create_publisher(Odometry, '/odometry/wheel', 10)
        self.create_subscription(Telemetry, '/telemetry', self._telemetry_cb, 10)

        self.get_logger().info(
            f'TelemetryToOdomNode 已启动  wheel_radius={self._r:.3f}m'
        )

    def _telemetry_cb(self, msg: Telemetry):
        now = self.get_clock().now()

        if self._last_time is None:
            self._last_time = now
            self._last_yaw_deg = msg.yaw
            return

        dt = (now - self._last_time).nanoseconds * 1e-9
        if dt < 1e-6:
            return

        # ── 线速度：双电机平均 RPS × 车轮周长 ─────────────────────────
        circ = 2.0 * math.pi * self._r
        v_linear = (msg.motor1_actual_rps + msg.motor2_actual_rps) * 0.5 * circ

        # ── 偏航角速度：STM32 偏航差分 ─────────────────────────────────
        dyaw = math.radians(msg.yaw - self._last_yaw_deg)
        # 角度环绕归一化到 [-π, π]
        while dyaw > math.pi:
            dyaw -= 2.0 * math.pi
        while dyaw < -math.pi:
            dyaw += 2.0 * math.pi
        omega = dyaw / dt

        # ── 位姿积分（dead reckoning，仅供 pose 字段）─────────────────
        self._yaw_rad = math.radians(msg.yaw)
        self._x += v_linear * math.cos(self._yaw_rad) * dt
        self._y += v_linear * math.sin(self._yaw_rad) * dt

        # ── 构建 Odometry 消息 ──────────────────────────────────────────
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = math.sin(self._yaw_rad / 2.0)
        odom.pose.pose.orientation.w = math.cos(self._yaw_rad / 2.0)

        # 位姿协方差（积分漂移较大，设置较高不确定性）
        odom.pose.covariance[0]  = 0.5   # x
        odom.pose.covariance[7]  = 0.5   # y
        odom.pose.covariance[35] = 0.3   # yaw

        odom.twist.twist.linear.x  = v_linear
        odom.twist.twist.angular.z = omega

        # 速度协方差（电机转速计相对稳定）
        odom.twist.covariance[0]  = 0.02  # vx
        odom.twist.covariance[35] = 0.1   # vyaw

        self._odom_pub.publish(odom)
        self._last_time = now
        self._last_yaw_deg = msg.yaw


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryToOdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
