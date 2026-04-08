from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

# 生成Launch文件的描述
# 该文件用于启动扩展卡尔曼滤波器(EKF)节点，进行车辆定位

def generate_launch_description():
    return LaunchDescription([
        # 定义一个ROS2节点
        Node(
            package='robot_localization',  # 使用robot_localization包
            executable='ekf_node',  # 启动ekf_node可执行文件
            name='ekf_filter_node',  # 节点名称为ekf_filter_node
            output='screen',  # 输出日志到屏幕
            parameters=[os.path.join(get_package_share_directory('vehicle_localization'), 'config', 'ekf.yaml')],  # 加载EKF配置文件
            remappings=[('/odometry/filtered', '/localization/odometry')]  # 话题重映射
        )
    ])