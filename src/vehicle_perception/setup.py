from setuptools import find_packages, setup

package_name = 'vehicle_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 将 config 目录下的模型文件安装到 share/vehicle_perception/config/
        # best.pt: 同时检测斑马线(Zebra)与红/绿/黄交通灯，供 road_detector 使用
        ('share/' + package_name + '/config', [
            'config/best.pt',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='david0529',
    maintainer_email='19804188212@163.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'obstacle_detector = vehicle_perception.obstacle_detector:main',
            # road_detector 基于 best.pt 检测斑马线与红/绿/黄交通灯
            'road_detector = vehicle_perception.road_detector:main',
            # yolo_detector 通用 YOLO 目标检测 (COCO 80 类)，针对 Jetson CUDA/TensorRT 优化
            'yolo_detector = vehicle_perception.yolo_detector:main',
        ],
    },
)
