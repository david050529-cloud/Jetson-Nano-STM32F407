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
        # 传感器驱动
        Node(package='gps_node', executable='gps_publisher_node', name='gps_publisher_node'),
        Node(package='lidar_node', executable='lidar_publisher_node', name='lidar_publisher_node'),
        Node(package='imu_node', executable='imu_driver_node', name='imu_driver_node'),
        Node(package='vision_package', executable='camera_node', name='camera_node'),
        
        # 定位
        IncludeLaunchDescription(PythonLaunchDescriptionSource(localization_launch)),
        
        # 感知
        Node(package='vehicle_perception', executable='road_detector', name='road_detector'),
        Node(package='vehicle_perception', executable='traffic_sign_detector', name='traffic_sign_detector'),
        Node(package='vehicle_perception', executable='obstacle_detector', name='obstacle_detector'),
        
        # 规划
        Node(package='vehicle_planning', executable='global_planner', name='global_planner'),
        Node(package='vehicle_planning', executable='local_planner', name='local_planner'),
        
        # 控制（使用 stm32_serial_bridge，不再使用 cmd_converter）
        Node(package='stm32_serial_bridge', executable='stm32_serial_bridge_node', name='stm32_bridge_node'),
    ])