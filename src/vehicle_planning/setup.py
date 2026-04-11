from setuptools import find_packages, setup

package_name = 'vehicle_planning'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/path_planning.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='david0529',
    maintainer_email='19804188212@163.com',
    description='无人车路径规划功能包（含路点解析、路网重规划、局部控制）',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            # 比赛路点路径规划节点：加载 waypoint.txt，发布竞赛路径，支持路网重规划
            'path_planner  = vehicle_planning.path_planner:main',
            # 目标点路径规划节点：接收任意目标点，生成直线路径（用于测试/辅助）
            'global_planner = vehicle_planning.global_planner:main',
            # 局部规划控制节点：路径跟踪、属性行为、交通标志响应、障碍物避障
            'local_planner  = vehicle_planning.local_planner:main',
        ],
    },
)
