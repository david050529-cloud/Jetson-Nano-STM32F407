from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 声明端口参数
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyUSB0',
        description='GPS serial port'
    )

    # GPS发布节点
    gps_publisher_node = Node(
        package='gps_node',
        executable='gps_publisher_node',
        name='gps_publisher_node',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port')
        }]
    )

    # GPS订阅节点
    gps_subscriber_node = Node(
        package='gps_node',
        executable='gps_subscriber_node',
        name='gps_subscriber_node',
        output='screen'
    )

    return LaunchDescription([
        port_arg,
        gps_publisher_node,
        gps_subscriber_node
    ])