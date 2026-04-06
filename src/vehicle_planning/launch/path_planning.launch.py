from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vehicle_planning',
            executable='vehicle_planning.path_planner',  # Updated path to match new location
            name='path_planner',
            output='screen'
        )
    ])