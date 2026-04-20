"""Nav2 集成启动文件。

在现有传感器驱动 + 定位 + 感知 + path_planner 基础上，叠加 Nav2 决策规划层。

整体架构（替代原 local_planner）：
  ┌─ 传感器 ─────────────────────────────────────────────────────┐
  │  GPS → navsat_transform → /odometry/gps                     │
  │  IMU → /imu/data_raw                                        │
  │  STM32 → telemetry_to_odom → /odometry/wheel                │
  │  三路 EKF → /localization/odometry + odom→base_link TF      │
  ├─ 感知 ───────────────────────────────────────────────────────┤
  │  /scan → Costmap2D（局部/全局代价地图）                       │
  │  /camera → road_detector / traffic_sign_detector            │
  ├─ 规划决策 ───────────────────────────────────────────────────┤
  │  path_planner → /planning/global_path                       │
  │  nav2_mission_executor → NavigateThroughPoses（动作客户端）  │
  │  Nav2 BT Navigator + Controller Server + Planner Server     │
  ├─ 控制 ───────────────────────────────────────────────────────┤
  │  Nav2 /cmd_vel → cmd_vel_to_motor_node → /motor_commands    │
  │  /motor_commands → STM32 串口桥                              │
  └──────────────────────────────────────────────────────────────┘

使用方法：
  ros2 launch vehicle_bringup vehicle_nav2.launch.py waypoint_file:=/tmp/wp.txt
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    localization_dir = get_package_share_directory('vehicle_localization')
    planning_dir     = get_package_share_directory('vehicle_planning')

    nav2_params  = os.path.join(planning_dir, 'config', 'nav2_params.yaml')
    bt_xml       = os.path.join(planning_dir, 'behavior_trees', 'vehicle_mission_bt.xml')
    ekf_config   = os.path.join(localization_dir, 'config', 'ekf.yaml')
    localization_launch = os.path.join(localization_dir, 'launch', 'localization.launch.py')

    declare_wp_file = DeclareLaunchArgument(
        'waypoint_file',
        default_value='/tmp/waypoint.txt',
        description='比赛路点文件路径',
    )

    # ── map → odom 静态变换（GPS 定位模式，无 SLAM 地图）────────────
    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    )

    # ── 传感器 TF ────────────────────────────────────────────────────
    tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_laser',
        arguments=['--x', '0.2', '--y', '0.0', '--z', '0.1',
                   '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                   '--frame-id', 'base_link', '--child-frame-id', 'laser'],
    )
    tf_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_camera',
        arguments=['--x', '0.25', '--y', '0.0', '--z', '0.15',
                   '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                   '--frame-id', 'base_link', '--child-frame-id', 'camera_link'],
    )
    tf_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_imu',
        arguments=['--x', '0.0', '--y', '0.0', '--z', '0.05',
                   '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                   '--frame-id', 'base_link', '--child-frame-id', 'imu_link'],
    )

    # ── 传感器驱动 ────────────────────────────────────────────────────
    gps_node = Node(
        package='gps_node', executable='gps_publisher_node', name='gps_publisher_node'
    )
    imu_node = Node(
        package='imu_node', executable='imu_driver_node', name='imu_driver_node'
    )
    lidar_node = Node(
        package='lidar_node', executable='lidar_publisher_node', name='lidar_publisher_node'
    )
    camera_node = Node(
        package='vision_package', executable='camera_node', name='camera_node'
    )
    stm32_bridge = Node(
        package='stm32_serial_bridge',
        executable='stm32_serial_bridge_node',
        name='stm32_bridge_node',
    )

    # ── 定位层（telemetry_to_odom + navsat_transform + EKF）──────────
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localization_launch)
    )

    # ── 感知层 ────────────────────────────────────────────────────────
    road_detector = Node(
        package='vehicle_perception', executable='road_detector', name='road_detector'
    )
    traffic_sign_detector = Node(
        package='vehicle_perception',
        executable='traffic_sign_detector',
        name='traffic_sign_detector',
    )

    # ── 全局路径规划（加载比赛路点）─────────────────────────────────────
    path_planner = Node(
        package='vehicle_planning',
        executable='path_planner',
        name='path_planner',
        output='screen',
        parameters=[{
            'waypoint_file':     LaunchConfiguration('waypoint_file'),
            'road_network_file': '',
            'use_cartesian':     False,
        }],
    )

    # ── Nav2 核心节点 ─────────────────────────────────────────────────
    nav2_controller = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params],
        remappings=[('odom', '/localization/odometry')],
    )
    nav2_planner = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params],
    )
    nav2_bt = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            nav2_params,
            {'default_nav_through_poses_bt_xml': bt_xml},
        ],
    )
    nav2_behaviors = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params],
    )
    nav2_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
            ],
        }],
    )

    # ── /cmd_vel → MotorCommand 转换器 ───────────────────────────────
    cmd_vel_to_motor = Node(
        package='vehicle_planning',
        executable='cmd_vel_to_motor',
        name='cmd_vel_to_motor_node',
        output='screen',
        parameters=[{
            'wheel_radius':     0.05,
            'wheelbase':        0.30,
            'max_steering_deg': 30.0,
        }],
    )

    # ── Nav2 任务执行器（路点→动作目标 + 交通标志响应）───────────────
    mission_executor = Node(
        package='vehicle_planning',
        executable='nav2_mission_executor',
        name='nav2_mission_executor',
        output='screen',
    )

    return LaunchDescription([
        declare_wp_file,
        # TF
        map_to_odom_tf, tf_laser, tf_camera, tf_imu,
        # 传感器驱动
        gps_node, imu_node, lidar_node, camera_node, stm32_bridge,
        # 定位
        localization,
        # 感知
        road_detector, traffic_sign_detector,
        # 规划决策
        path_planner,
        nav2_controller, nav2_planner, nav2_bt, nav2_behaviors, nav2_lifecycle,
        # 控制桥
        cmd_vel_to_motor,
        mission_executor,
    ])
