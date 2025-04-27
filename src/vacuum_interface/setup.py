from setuptools import find_packages, setup

package_name = 'vacuum_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial'], # Added pyserial dependency
    zip_safe=True,
    maintainer='reeve',
    maintainer_email='2544141826@qq.com',
    description='ROS 2 node to interface with STM32 via serial for IMU and Odometry data.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stm32_interface_node = vacuum_interface.stm32_interface_node:main',
        ],
    },
)
