from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_id', default_value='0',
            description='Camera device index (e.g. 0 for /dev/video0)'
        ),
        DeclareLaunchArgument(
            'width', default_value='640',
            description='Frame width in pixels'
        ),
        DeclareLaunchArgument(
            'height', default_value='480',
            description='Frame height in pixels'
        ),
        DeclareLaunchArgument(
            'fps', default_value='30',
            description='Target frame rate'
        ),
        DeclareLaunchArgument(
            'jpeg_quality', default_value='80',
            description='JPEG compression quality (0-100)'
        ),

        Node(
            package='vision_node',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[{
                'camera_id':    LaunchConfiguration('camera_id'),
                'width':        LaunchConfiguration('width'),
                'height':       LaunchConfiguration('height'),
                'fps':          LaunchConfiguration('fps'),
                'jpeg_quality': LaunchConfiguration('jpeg_quality'),
            }],
        ),
    ])
