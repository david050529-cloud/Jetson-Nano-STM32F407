from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='imu_gps_parser',
            executable='imu_gps_parser_node',
            name='imu_gps_parser_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/IMU',   # 请修改为实际串口设备
                'baudrate': 9600
            }]
        ),
        Node(
            package='imu_gps_parser',
            executable='imu_gps_subscriber_node',
            name='imu_gps_subscriber_node',
            output='screen'
        )
    ])