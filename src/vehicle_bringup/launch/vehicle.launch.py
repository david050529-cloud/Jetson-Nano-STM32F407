from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_vision_share = get_package_share_directory('vision_node')
    pkg_localization_share = get_package_share_directory('vehicle_localization')

    vision_launch = os.path.join(pkg_vision_share, 'launch', 'vision.launch.py')
    localization_launch = os.path.join(pkg_localization_share, 'launch', 'localization.launch.py')

    return LaunchDescription([

        # ════════════════════════════════════════════════════════════════
        # 传感器驱动
        # ════════════════════════════════════════════════════════════════
        Node(package='imu_gps_parser', executable='imu_gps_parser_node',
             name='imu_gps_parser_node'),
        Node(package='lidar_node',     executable='lidar_publisher_node',
             name='lidar_publisher_node'),

        # ── 视觉子系统 (摄像头 + 道路检测 + YOLO 通用检测) ────────────
        IncludeLaunchDescription(PythonLaunchDescriptionSource(vision_launch)),

        # ════════════════════════════════════════════════════════════════
        # TF 静态变换 — 传感器在车体坐标系(base_link)中的安装位置
        # ════════════════════════════════════════════════════════════════
        # base_link → laser (激光雷达 — 车头正上方 0.2m, 高 0.1m)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_to_laser',
            arguments=['--x', '0.2', '--y', '0.0', '--z', '0.1',
                       '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                       '--frame-id', 'base_link', '--child-frame-id', 'laser'],
        ),
        # base_link → camera_link (摄像头 — 车头 0.25m, 高 0.15m)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_to_camera',
            arguments=['--x', '0.25', '--y', '0.0', '--z', '0.15',
                       '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                       '--frame-id', 'base_link', '--child-frame-id', 'camera_link'],
        ),
        # base_link → imu_link (IMU — 车体中心, 高 0.05m)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_to_imu',
            arguments=['--x', '0.0', '--y', '0.0', '--z', '0.05',
                       '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                       '--frame-id', 'base_link', '--child-frame-id', 'imu_link'],
        ),

        # ════════════════════════════════════════════════════════════════
        # 定位 (EKF: IMU + GPS → /localization/odometry, odom→base_link TF)
        # ════════════════════════════════════════════════════════════════
        IncludeLaunchDescription(PythonLaunchDescriptionSource(localization_launch)),

        # ════════════════════════════════════════════════════════════════
        # 感知 — LiDAR 障碍物检测
        # ════════════════════════════════════════════════════════════════
        Node(package='vehicle_perception', executable='obstacle_detector',
             name='obstacle_detector'),

        # ════════════════════════════════════════════════════════════════
        # 规划
        # ════════════════════════════════════════════════════════════════
        Node(package='vehicle_planning', executable='global_planner',
             name='global_planner'),
        Node(package='vehicle_planning', executable='local_planner',
             name='local_planner'),

        # ════════════════════════════════════════════════════════════════
        # 底盘控制 (串口 → STM32F407)
        # ════════════════════════════════════════════════════════════════
        Node(package='stm32_serial_bridge', executable='stm32_serial_bridge_node',
             name='stm32_bridge_node'),
    ])
