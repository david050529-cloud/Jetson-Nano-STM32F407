from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vehicle_planning',
            executable='path_planner',
            name='path_planner',
            output='screen',
            parameters=[
                {'waypoint_file': '/path/to/waypoint.txt'}  # Dynamic parameter for waypoint file
            ]
        )
    ])