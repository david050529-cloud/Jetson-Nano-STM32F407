import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import struct

class CmdToSTM32(Node):
    def __init__(self):
        super().__init__('cmd_converter')
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        # 修改串口设备名
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
        self.get_logger().info('STM32 serial opened')

    def cmd_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z
        # 打包为4字节float两个，加校验和简单处理
        data = struct.pack('<ff', linear, angular)
        checksum = sum(data) & 0xFF
        self.ser.write(data + bytes([checksum]))

    def __del__(self):
        if self.ser.is_open:
            self.ser.close()