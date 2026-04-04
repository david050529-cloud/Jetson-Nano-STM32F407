from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 声明参数，允许通过命令行设置摄像头ID和模型路径
    camera_id_arg = DeclareLaunchArgument(
        'camera_id', default_value='0',
        description='Camera device ID'
    )
    model_arg = DeclareLaunchArgument(
        'model', default_value='yolov8n.pt',
        description='YOLO model file path'
    )

    camera_node = Node(
        package='vision_node',
        executable='camera_node',
        name='camera_node',
        output='screen',
        parameters=[{'camera_id': LaunchConfiguration('camera_id')}]
    )

    yolo_node = Node(
        package='vision_package',
        executable='yolo_node.py',
        name='yolo_node',
        output='screen',
        parameters=[{'model': LaunchConfiguration('model')}]
    )

    yolo_video_subscriber_node = Node(
        package='vision_package',
        executable='yolo_video_subscriber',
        name='yolo_video_subscriber',
        output='screen'
    )

    return LaunchDescription([
        camera_id_arg,
        model_arg,
        camera_node,
        yolo_node,
        yolo_video_subscriber_node,
    ])