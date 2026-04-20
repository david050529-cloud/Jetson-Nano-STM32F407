#!/usr/bin/env python3
"""Nav2 /cmd_vel (Twist) → STM32 MotorCommand 转换节点。

Nav2 控制器（RegulatedPurePursuitController）输出标准 geometry_msgs/Twist：
  linear.x  → 期望前进速度 (m/s)
  angular.z → 期望偏航角速度 (rad/s)

本节点根据 Ackermann 转向几何将其转换为：
  motor1/2_target_rps → 电机目标转速 (转/秒)
  servo_angle         → 舵机角度 (0-180°，90°=直行)

参数（需根据实车标定）：
  wheel_radius     - 驱动轮半径 (m)，默认 0.05
  wheelbase        - 前后轴距 (m)，默认 0.30
  max_steering_deg - 最大机械转向角 (°)，默认 30
  servo_center     - 舵机中位角 (°)，默认 90
  servo_range      - 舵机单侧最大行程 (°)，默认 90
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from stm32_serial_bridge.msg import MotorCommand


class CmdVelToMotorNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_motor_node')

        self.declare_parameter('wheel_radius',     0.05)
        self.declare_parameter('wheelbase',        0.30)
        self.declare_parameter('max_steering_deg', 30.0)
        self.declare_parameter('servo_center',     90)
        self.declare_parameter('servo_range',      90)

        self._r          = self.get_parameter('wheel_radius').value
        self._L          = self.get_parameter('wheelbase').value
        self._max_steer  = math.radians(self.get_parameter('max_steering_deg').value)
        self._sc         = self.get_parameter('servo_center').value
        self._sr         = self.get_parameter('servo_range').value

        self._motor_pub = self.create_publisher(MotorCommand, '/motor_commands', 10)
        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_cb, 10)

        self.get_logger().info(
            f'CmdVelToMotorNode 已启动  '
            f'r={self._r:.3f}m  L={self._L:.2f}m  max_steer={math.degrees(self._max_steer):.1f}°'
        )

    def _cmd_vel_cb(self, msg: Twist):
        v     = msg.linear.x
        omega = msg.angular.z

        # Ackermann 转向角：δ = arctan(ω·L / v)
        if abs(v) > 0.01:
            steer = math.atan(omega * self._L / v)
        else:
            steer = 0.0
        steer = max(-self._max_steer, min(self._max_steer, steer))

        # 舵机映射：90°=直行，正角=左转，负角=右转
        servo_angle = int(self._sc + (steer / self._max_steer) * self._sr)
        servo_angle = max(0, min(180, servo_angle))

        # 电机 RPS：线速度 / 车轮周长
        rps = v / (2.0 * math.pi * self._r)

        cmd = MotorCommand()
        cmd.motor1_target_rps = float(rps)
        cmd.motor2_target_rps = float(rps)
        cmd.servo_angle       = servo_angle
        self._motor_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToMotorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
