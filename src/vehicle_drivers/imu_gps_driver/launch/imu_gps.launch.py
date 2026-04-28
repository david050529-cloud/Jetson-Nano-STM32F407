from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('imu_port', default_value='/dev/IMU',
                              description='IMU serial device'),
        DeclareLaunchArgument('imu_baud', default_value='9600',
                              description='IMU baud rate'),
        DeclareLaunchArgument('gps_port', default_value='/dev/GPS',
                              description='GPS serial device'),
        DeclareLaunchArgument('gps_baud', default_value='9600',
                              description='GPS baud rate'),

        Node(
            package='imu_gps_driver',
            executable='imu_publisher_node',
            name='imu_publisher_node',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('imu_port'),
                'baudrate': LaunchConfiguration('imu_baud')
            }]
        ),

        Node(
            package='imu_gps_driver',
            executable='gps_publisher_node',
            name='gps_publisher_node',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('gps_port'),
                'baudrate': LaunchConfiguration('gps_baud')
            }]
        ),
    ])