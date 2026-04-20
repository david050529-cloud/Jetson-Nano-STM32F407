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
        ('share/' + package_name + '/config',
            ['config/nav2_params.yaml']),
        ('share/' + package_name + '/behavior_trees',
            ['behavior_trees/vehicle_mission_bt.xml']),
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
            'path_planner        = vehicle_planning.path_planner:main',
            'global_planner      = vehicle_planning.global_planner:main',
            'local_planner       = vehicle_planning.local_planner:main',
            # Nav2 集成层
            'cmd_vel_to_motor    = vehicle_planning.cmd_vel_to_motor_node:main',
            'nav2_mission_executor = vehicle_planning.nav2_mission_executor:main',
        ],
    },
)
