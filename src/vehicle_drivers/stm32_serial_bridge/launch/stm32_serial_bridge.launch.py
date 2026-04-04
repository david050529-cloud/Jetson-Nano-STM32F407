from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'port', default_value='/dev/ttyACM0',
            description='Serial port to communicate with STM32'
        ),
        DeclareLaunchArgument(
            'baudrate', default_value='115200',
            description='Serial baudrate'
        ),
        Node(
            package='stm32_serial_bridge',
            executable='stm32_serial_bridge_node',
            name='stm32_serial_bridge_node',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'baudrate': LaunchConfiguration('baudrate'),
                'timeout_ms': 100
            }]
        )
    ])