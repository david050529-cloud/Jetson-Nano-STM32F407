from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # IMU驱动节点
        Node(
            package='imu_node',
            executable='imu_driver_node',
            name='imu_driver_node',
            output='screen',
            parameters=[{
                'use_sim_time': False
            }],
            remappings=[
                ('imu/data_raw', 'imu/data_raw')
            ]
        ),
        
        # IMU客户端节点（订阅并显示IMU数据）
        Node(
            package='imu_node',
            executable='imu_client_node',
            name='imu_client_node',
            output='screen',
            parameters=[{
                'use_sim_time': False
            }]
        ),
    ])