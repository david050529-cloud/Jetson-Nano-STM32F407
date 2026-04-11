from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    路径规划模块 Launch 文件
    启动三个节点：
      1. path_planner  — 加载比赛路点文件，管理竞赛路径和路网重规划
      2. global_planner — 接收任意目标点并规划路径（可选，测试用）
      3. local_planner  — 局部规划控制（路径跟踪、避障、交通标志）

    可调参数（通过命令行传入，例：ros2 launch ... waypoint_file:=/tmp/wp.txt）：
      waypoint_file      : 比赛路点文件路径（必须在比赛前15分钟更新）
      road_network_file  : 路网文件路径（空则自动从路点构建顺序路网）
      use_cartesian      : 是否使用直角坐标格式路点（true/false）
    """

    # ── 可调启动参数声明 ────────────────────────────────────────────────────
    declare_wp_file = DeclareLaunchArgument(
        'waypoint_file',
        default_value='/tmp/waypoint.txt',
        description='比赛路点文件路径（waypoint.txt）')

    declare_net_file = DeclareLaunchArgument(
        'road_network_file',
        default_value='',
        description='路网文件路径（留空则从路点自动构建）')

    declare_cartesian = DeclareLaunchArgument(
        'use_cartesian',
        default_value='false',
        description='是否使用直角坐标路点格式（true=直角坐标，false=GPS经纬度）')

    # ── 节点定义 ──────────────────────────────────────────────────────────

    # 路点路径规划节点：比赛核心节点，启动即加载并发布路径
    path_planner_node = Node(
        package='vehicle_planning',
        executable='path_planner',
        name='path_planner',
        output='screen',
        parameters=[{
            'waypoint_file':     LaunchConfiguration('waypoint_file'),
            'road_network_file': LaunchConfiguration('road_network_file'),
            'use_cartesian':     LaunchConfiguration('use_cartesian'),
        }]
    )

    # 全局目标规划节点（辅助/测试用，比赛中通常不需要启动）
    global_planner_node = Node(
        package='vehicle_planning',
        executable='global_planner',
        name='global_planner',
        output='screen',
    )

    # 局部规划控制节点：路径跟踪 + 障碍物避障 + 交通标志响应
    local_planner_node = Node(
        package='vehicle_planning',
        executable='local_planner',
        name='local_planner',
        output='screen',
    )

    return LaunchDescription([
        declare_wp_file,
        declare_net_file,
        declare_cartesian,
        path_planner_node,
        # global_planner_node,  # 比赛时注释掉，避免与 path_planner 话题冲突
        local_planner_node,
    ])
