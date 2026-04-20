from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/GPS',
                              description='GPS serial port'),
        DeclareLaunchArgument('baudrate', default_value='9600',
                              description='Baudrate'),
        Node(
            package='gps_node',
            executable='gps_publisher_node',
            name='gps_publisher_node',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'baudrate': LaunchConfiguration('baudrate')
            }]
        ),
        Node(
            package='gps_node',
            executable='gps_subscriber_node',
            name='gps_subscriber_node',
            output='screen'
        )
    ])