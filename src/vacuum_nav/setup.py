from setuptools import find_packages, setup

package_name = 'vacuum_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/navigation_launch.py',
                                               'launch/vacuum_slam_nav_launch.py',
                                               'launch/goal_receiver_launch.py',
                                               'launch/velocity_converter_launch.py']),
        ('share/' + package_name + '/config', ['config/nav2_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='reeve',
    maintainer_email='2544141826@qq.com',
    description='Vacuum cleaner navigation package with SLAM and Nav2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'goal_receiver_node = vacuum_nav.goal_receiver_node:main',
            'velocity_converter_node = vacuum_nav.velocity_converter_node:main',
        ],
    },
)
