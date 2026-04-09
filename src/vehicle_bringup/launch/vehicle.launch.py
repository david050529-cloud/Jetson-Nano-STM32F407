from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    localization_launch = os.path.join(
        get_package_share_directory('vehicle_localization'),
        'launch',
        'localization.launch.py'
    )
    return LaunchDescription([
        # ── 传感器驱动 ──────────────────────────────────────────────
        Node(package='gps_node',        executable='gps_publisher_node',  name='gps_publisher_node'),
        Node(package='lidar_node',      executable='lidar_publisher_node', name='lidar_publisher_node'),
        Node(package='imu_node',        executable='imu_driver_node',      name='imu_driver_node'),
        Node(package='vision_package',  executable='camera_node',          name='camera_node'),

        # ── TF 静态变换：描述传感器在车体坐标系(base_link)中的安装位置 ──
        # base_link → laser（激光雷达安装在车头正上方，高度0.1m）
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_to_laser',
            arguments=['--x', '0.2', '--y', '0.0', '--z', '0.1',
                       '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                       '--frame-id', 'base_link', '--child-frame-id', 'laser'],
        ),
        # base_link → camera_link（摄像头安装在车头，高度0.15m）
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_to_camera',
            arguments=['--x', '0.25', '--y', '0.0', '--z', '0.15',
                       '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                       '--frame-id', 'base_link', '--child-frame-id', 'camera_link'],
        ),
        # base_link → imu_link（IMU 安装在车体中心）
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_to_imu',
            arguments=['--x', '0.0', '--y', '0.0', '--z', '0.05',
                       '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                       '--frame-id', 'base_link', '--child-frame-id', 'imu_link'],
        ),

        # ── 定位（EKF：IMU + GPS → /localization/odometry，发布 odom→base_link TF）──
        IncludeLaunchDescription(PythonLaunchDescriptionSource(localization_launch)),

        # ── 感知 ────────────────────────────────────────────────────
        Node(package='vehicle_perception', executable='road_detector',         name='road_detector'),
        Node(package='vehicle_perception', executable='traffic_sign_detector',  name='traffic_sign_detector'),
        Node(package='vehicle_perception', executable='obstacle_detector',      name='obstacle_detector'),
        # 通用 YOLO 检测节点（发布 /camera/image_detected，用于调试可视化）
        Node(
            package='vision_package',
            executable='yolo_node',
            name='yolo_node',
            parameters=[{'model': 'yolov8n.pt'}],
        ),

        # ── 规划 ────────────────────────────────────────────────────
        # global_planner：接收 /planning/goal，输出 /planning/global_path
        Node(package='vehicle_planning', executable='global_planner', name='global_planner'),
        # local_planner：跟踪全局路径 + 避障，输出 motor_commands → stm32
        Node(package='vehicle_planning', executable='local_planner',  name='local_planner'),

        # ── 底盘控制（serial bridge → STM32F407）──────────────────────
        Node(package='stm32_serial_bridge', executable='stm32_serial_bridge_node', name='stm32_bridge_node'),
    ])
