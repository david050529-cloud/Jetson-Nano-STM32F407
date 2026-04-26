from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    camera_id_arg = DeclareLaunchArgument(
        'camera_id', default_value='0',
        description='Camera device ID'
    )

    transport_arg = DeclareLaunchArgument(
        'image_transport', default_value='ffmpeg',   # 使用 ffmpeg 插件
        description='Image transport plugin (raw, ffmpeg)'
    )

    camera_node = Node(
        package='vision_package',
        executable='camera_node',
        name='camera_node',
        output='screen',
        parameters=[{
            'camera_id': LaunchConfiguration('camera_id'),
            'image_transport': LaunchConfiguration('image_transport'),
            # ffmpeg_image_transport 特有参数
            'ffmpeg_image_transport.encoding': 'libx264',
            'ffmpeg_image_transport.bit_rate': 2000000,      # 2 Mbps
            'ffmpeg_image_transport.preset': 'ultrafast',    # 适合实时
            'ffmpeg_image_transport.profile': 'baseline',
        }]
    )

    return LaunchDescription([
        camera_id_arg,
        transport_arg,
        camera_node,
    ])