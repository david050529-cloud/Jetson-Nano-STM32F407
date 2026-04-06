import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class CommandConverter(Node):
    def __init__(self):
        super().__init__('cmd_converter')
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 9600)

        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value

        try:
            self.serial = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info(f"Serial port {self.serial_port} opened at {self.baud_rate} baud.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            self.serial = None

        self.subscription = self.create_subscription(
            String,
            'cmd_vel',
            self.cmd_callback,
            10
        )

    def cmd_callback(self, msg):
        if self.serial:
            try:
                self.serial.write(msg.data.encode())
                self.get_logger().info(f"Sent command: {msg.data}")
            except serial.SerialException as e:
                self.get_logger().error(f"Failed to send command: {e}")

    def destroy_node(self):
        if self.serial:
            self.serial.close()
            self.get_logger().info("Serial port closed.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CommandConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()