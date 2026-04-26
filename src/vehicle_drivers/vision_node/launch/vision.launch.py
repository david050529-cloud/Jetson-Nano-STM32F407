from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    camera_id_arg = DeclareLaunchArgument(
        'camera_id', default_value='0',
        description='Camera device ID'
    )

    model_arg = DeclareLaunchArgument(
        'model', default_value='yolov8n.pt',
        description='YOLO model file path'
    )

    camera_node = Node(
        package='vision_package',
        executable='camera_node',
        name='camera_node',
        output='screen',
        parameters=[{
            'camera_id': LaunchConfiguration('camera_id'),
        }]
    )

    yolo_node = Node(
        package='vision_package',
        executable='yolo_node',
        name='yolo_node',
        output='screen',
        parameters=[{
            'model': LaunchConfiguration('model'),
        }]
    )

    return LaunchDescription([
        camera_id_arg,
        model_arg,
        camera_node,
        yolo_node,
    ])
