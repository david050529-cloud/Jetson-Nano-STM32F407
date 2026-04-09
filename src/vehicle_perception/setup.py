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
            'traffic_light_detector = vehicle_perception.traffic_light_detector:main',
            'road_detector = vehicle_perception.road_detector:main', 
        ],
    },
)
