#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 激光雷达参数
    channel_type = LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='460800')
    frame_id = LaunchConfiguration('frame_id', default='laser')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Standard')
    
    # 静态变换参数（world到laser的坐标变换）
    # 平移参数（单位：米）
    static_transform_x = LaunchConfiguration('static_transform_x', default='0.0')
    static_transform_y = LaunchConfiguration('static_transform_y', default='0.0')
    static_transform_z = LaunchConfiguration('static_transform_z', default='0.0')
    # 旋转参数（单位：弧度）
    static_transform_roll = LaunchConfiguration('static_transform_roll', default='0.0')
    static_transform_pitch = LaunchConfiguration('static_transform_pitch', default='0.0')
    static_transform_yaw = LaunchConfiguration('static_transform_yaw', default='0.0')
    # 坐标系名称
    parent_frame = LaunchConfiguration('parent_frame', default='world')
    child_frame = LaunchConfiguration('child_frame', default='laser')

    return LaunchDescription([
        # 激光雷达参数声明
        DeclareLaunchArgument(
            'channel_type',
            default_value=channel_type,
            description='Specifying channel type of lidar (serial/tcp/udp)'),

        DeclareLaunchArgument(
            'serial_port',
            default_value=serial_port,
            description='Specifying usb port to connected lidar'),

        DeclareLaunchArgument(
            'serial_baudrate',
            default_value=serial_baudrate,
            description='Specifying usb port baudrate to connected lidar'),
        
        DeclareLaunchArgument(
            'frame_id',
            default_value=frame_id,
            description='Specifying frame_id of lidar'),
        
        DeclareLaunchArgument(
            'inverted',
            default_value=inverted,
            description='Specifying whether or not to invert scan data'),

        DeclareLaunchArgument(
            'angle_compensate',
            default_value=angle_compensate,
            description='Specifying whether or not to enable angle_compensate of scan data'),

        DeclareLaunchArgument(
            'scan_mode',
            default_value=scan_mode,
            description='Specifying scan mode of lidar'),
        
        # 静态变换参数声明
        DeclareLaunchArgument(
            'static_transform_x',
            default_value=static_transform_x,
            description='X offset from parent frame to child frame (meters)'),
        
        DeclareLaunchArgument(
            'static_transform_y',
            default_value=static_transform_y,
            description='Y offset from parent frame to child frame (meters)'),
        
        DeclareLaunchArgument(
            'static_transform_z',
            default_value=static_transform_z,
            description='Z offset from parent frame to child frame (meters)'),
        
        DeclareLaunchArgument(
            'static_transform_roll',
            default_value=static_transform_roll,
            description='Roll angle (radians)'),
        
        DeclareLaunchArgument(
            'static_transform_pitch',
            default_value=static_transform_pitch,
            description='Pitch angle (radians)'),
        
        DeclareLaunchArgument(
            'static_transform_yaw',
            default_value=static_transform_yaw,
            description='Yaw angle (radians)'),
        
        DeclareLaunchArgument(
            'parent_frame',
            default_value=parent_frame,
            description='Parent frame name (e.g., world, map, odom)'),
        
        DeclareLaunchArgument(
            'child_frame',
            default_value=child_frame,
            description='Child frame name (should match lidar frame_id)'),

        # 激光雷达节点
        Node(
            package='lidar_node',
            executable='lidar_publisher_node',
            name='lidar_node',
            parameters=[{
                'channel_type': channel_type,
                'serial_port': serial_port, 
                'serial_baudrate': serial_baudrate, 
                'frame_id': frame_id,
                'inverted': inverted, 
                'angle_compensate': angle_compensate, 
                'scan_mode': scan_mode
            }],
            output='screen'),
        
        # 静态变换发布节点（发布world到laser的静态变换）
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=[
                '--x', static_transform_x,
                '--y', static_transform_y,
                '--z', static_transform_z,
                '--roll', static_transform_roll,
                '--pitch', static_transform_pitch,
                '--yaw', static_transform_yaw,
                '--frame-id', parent_frame,
                '--child-frame-id', child_frame
            ],
            output='screen'),
    ])

# 发布world到laser的静态变换
'''
# 查看激光雷达节点的frame_id参数
ros2 param get /lidar_node laser_frame_id

# 或者查看TF树
ros2 run tf2_tools view_frames

# 发布world到laser的静态变换
ros2 run tf2_ros static_transform_publisher --frame-id world --child-frame-id laser

'''