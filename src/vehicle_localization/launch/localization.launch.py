from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ekf_config = os.path.join(
        get_package_share_directory('vehicle_localization'),
        'config',
        'ekf.yaml'
    )
    return LaunchDescription([
        # navsat_transform_node：将 GPS NavSatFix 转换为局部坐标系 Odometry
        # 输入：/gps/fix (NavSatFix) + /imu/data_raw (Imu)
        # 输出：/odometry/gps (Odometry，局部 odom 坐标系下的 x/y 位置)
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            parameters=[{
                'magnetic_declination_radians': 0.0,
                'yaw_offset': 0.0,
                'zero_altitude': True,
                'broadcast_utm_transform': False,
                'publish_filtered_gps': False,
                'use_odometry_yaw': False,
                'wait_for_datum': False,
            }],
            remappings=[
                ('imu/data', '/imu/data_raw'),
                ('gps/fix',  '/gps/fix'),
                ('odometry/filtered', '/localization/odometry'),  # 来自 EKF 的反馈
            ],
        ),

        # ekf_filter_node：融合 IMU 姿态 + GPS 局部位置 → 发布里程计 + odom→base_link TF
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config],
            remappings=[('/odometry/filtered', '/localization/odometry')],
        ),
    ])
