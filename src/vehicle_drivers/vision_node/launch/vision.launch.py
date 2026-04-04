from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    camera_id_arg = DeclareLaunchArgument(
        'camera_id', default_value='0',
        description='Camera device ID'
    )

    camera_node = Node(
        package='vision_package',   # 注意包名应为 vision_package
        executable='camera_node',
        name='camera_node',
        output='screen',
        parameters=[{'camera_id': LaunchConfiguration('camera_id')}]
    )

    return LaunchDescription([
        camera_id_arg,
        camera_node,
    ])