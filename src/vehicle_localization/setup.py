from setuptools import find_packages, setup

# 定义包的名称
package_name = 'vehicle_localization'

# 配置包的安装信息
setup(
    name=package_name,  # 包名称
    version='0.0.0',  # 包的版本号
    packages=find_packages(exclude=['test']),  # 自动查找子包，排除测试目录
    data_files=[
        # 安装资源文件
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],  # 安装依赖
    zip_safe=True,  # 指定包是否可以安全地作为zip文件安装
    maintainer='david0529',  # 维护者信息
    maintainer_email='19804188212@163.com',  # 维护者邮箱
    description='TODO: Package description',  # 包的描述信息
    license='TODO: License declaration',  # 包的许可证信息
    extras_require={
        'test': [
            'pytest',  # 测试依赖
        ],
    },
    entry_points={
        'console_scripts': [
            # 定义可执行脚本
        ],
    },
)
