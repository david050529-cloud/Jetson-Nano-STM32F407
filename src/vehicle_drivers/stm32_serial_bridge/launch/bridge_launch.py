from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port', default_value='/dev/ttyACM0',
            description='Serial port device'
        ),
        DeclareLaunchArgument(
            'baud_rate', default_value='115200',
            description='Baud rate'
        ),
        Node(
            package='stm32_serial_bridge',
            executable='stm32_serial_bridge_node',
            name='stm32_serial_bridge',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'baud_rate': LaunchConfiguration('baud_rate'),
                'serial_timeout_ms': 100
            }]
        )
    ])