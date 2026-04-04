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
        # 传感器驱动（根据你的实际驱动包修改）
        Node(package='gps_driver', executable='gps_node', name='gps'),
        Node(package='lidar_driver', executable='lidar_node', name='lidar'),
        Node(package='imu_driver', executable='imu_node', name='imu'),
        Node(package='camera_driver', executable='camera_node', name='camera'),
        # 定位
        IncludeLaunchDescription(PythonLaunchDescriptionSource(localization_launch)),
        # 感知
        Node(package='vehicle_perception', executable='road_detector', name='road_detector'),
        Node(package='vehicle_perception', executable='traffic_sign_detector', name='traffic_sign_detector'),
        Node(package='vehicle_perception', executable='obstacle_detector', name='obstacle_detector'),
        # 规划
        Node(package='vehicle_planning', executable='global_planner', name='global_planner'),
        Node(package='vehicle_planning', executable='local_planner', name='local_planner'),
        # 控制
        Node(package='vehicle_control', executable='cmd_converter', name='cmd_converter'),
    ])