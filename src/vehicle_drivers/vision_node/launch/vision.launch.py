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
        'image_transport', default_value='raw',
        description='Image transport to use (e.g. raw, compressed, h264)'
    )

    camera_node = Node(
        package='vision_package',
        executable='camera_node',
        name='camera_node',
        output='screen',
        parameters=[{
            'camera_id': LaunchConfiguration('camera_id'),
        }],
        remappings=[
            # image_transport 会自动处理话题重映射，通常无需额外设置
        ]
    )

    # 若想强制传输层使用 H.264，可在启动时加参数：image_transport:=h264
    return LaunchDescription([
        camera_id_arg,
        transport_arg,
        camera_node,
    ])