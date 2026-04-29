from setuptools import setup, find_packages

package_name = 'vision_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/vision.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='david0529',
    maintainer_email='19804188212@163.com',
    description='Vision package with camera driver and YOLO detection',
    license='TODO: License declaration',
    # tests_require=['pytest'],
    extras_require = {
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'camera_node = vision_package.camera_node:main',
            'yolo_node = vision_package.yolo_node:main',
            'yolo_video_subscriber = vision_package.yolo_video_subscriber:main',
        ],
    },
)
