"""
vision.launch.py

  启动摄像头节点 + 视频识别节点，作为视觉子系统的统一入口。

  节点:
    camera_node    — USB 摄像头采集, 发布 raw BGR8 + 可选 JPEG
    road_detector  — 道路信号检测 (斑马线 / 红绿黄交通灯, 使用 best.pt)
    yolo_detector  — 通用 YOLO 目标检测 (COCO 80 类, 使用 yolov8n.pt)

  参数:
    camera_id, width, height, fps, fourcc — 摄像头配置
    conf_threshold, input_size            — YOLO 检测阈值/输入尺寸
    half_precision                        — Jetson FP16 加速
    engine_path                           — TensorRT 引擎路径 (可选)
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([

        # ════ 摄像头参数 ═══════════════════════════════════════════
        DeclareLaunchArgument(
            'camera_id', default_value='0',
            description='Camera device index (/dev/videoN)'),
        DeclareLaunchArgument(
            'width', default_value='640',
            description='Frame width'),
        DeclareLaunchArgument(
            'height', default_value='480',
            description='Frame height'),
        DeclareLaunchArgument(
            'fps', default_value='30',
            description='Target frames per second'),
        DeclareLaunchArgument(
            'fourcc', default_value='MJPG',
            description='Pixel format: MJPG(camera-side encode, best FPS) / YUYV(raw) / empty=auto'),
        DeclareLaunchArgument(
            'publish_compressed', default_value='true',
            description='Publish /camera/image_raw/compressed in addition to raw'),
        DeclareLaunchArgument(
            'jpeg_quality', default_value='80',
            description='JPEG quality (1-100) for compressed topic'),

        # ════ 检测共用参数 ═════════════════════════════════════════
        DeclareLaunchArgument(
            'conf_threshold', default_value='0.35',
            description='YOLO confidence threshold'),
        DeclareLaunchArgument(
            'input_size', default_value='640',
            description='YOLO inference input size (pixels)'),
        DeclareLaunchArgument(
            'half_precision', default_value='true',
            description='Use FP16 inference on CUDA (2× faster on Jetson Nano)'),
        DeclareLaunchArgument(
            'device', default_value='auto',
            description='Inference device: auto / cuda:0 / cpu'),
        DeclareLaunchArgument(
            'engine_path', default_value='',
            description='Path to TensorRT .engine file (optional, fastest on Jetson)'),

        # ════ 节点: 摄像头 ─────────────────────────────────────────
        Node(
            package='vision_node',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[{
                'camera_id':           LaunchConfiguration('camera_id'),
                'width':               LaunchConfiguration('width'),
                'height':              LaunchConfiguration('height'),
                'fps':                 LaunchConfiguration('fps'),
                'fourcc':              LaunchConfiguration('fourcc'),
                'publish_compressed':  LaunchConfiguration('publish_compressed'),
                'jpeg_quality':        LaunchConfiguration('jpeg_quality'),
            }],
        ),

        # ════ 节点: 道路信号检测 (斑马线 + 红/绿/黄交通灯) ──────────
        Node(
            package='vehicle_perception',
            executable='road_detector',
            name='road_detector',
            output='screen',
            parameters=[{
                'conf_threshold': LaunchConfiguration('conf_threshold'),
            }],
        ),

        # ════ 节点: 通用 YOLO 目标检测 (COCO 80 类) ──────────────────
        Node(
            package='vehicle_perception',
            executable='yolo_detector',
            name='yolo_detector',
            output='screen',
            parameters=[{
                'model_path':      'yolov8n.pt',
                'conf_threshold':  LaunchConfiguration('conf_threshold'),
                'input_size':      LaunchConfiguration('input_size'),
                'device':          LaunchConfiguration('device'),
                'half_precision':  LaunchConfiguration('half_precision'),
                'engine_path':     LaunchConfiguration('engine_path'),
            }],
        ),
    ])
