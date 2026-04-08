from setuptools import find_packages, setup

# Define the package name
package_name = 'vehicle_bringup'

# Configure the setup for the package
setup(
    name=package_name,  # Name of the package
    version='0.0.0',  # Initial version of the package
    packages=find_packages(exclude=['test']),  # Automatically find sub-packages, excluding tests
    data_files=[
        # Install resource files for the package
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],  # Dependencies required for installation
    zip_safe=True,  # Indicates if the package can be installed as a .egg file
    maintainer='david0529',  # Maintainer of the package
    maintainer_email='19804188212@163.com',  # Maintainer's email address
    description='TODO: Package description',  # Short description of the package
    license='TODO: License declaration',  # License information
    extras_require={
        'test': [
            'pytest',  # Additional dependencies for testing
        ],
    },
    entry_points={
        'console_scripts': [
            # Define executable scripts here
        ],
    },
)
