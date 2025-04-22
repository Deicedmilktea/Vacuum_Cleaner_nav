from setuptools import find_packages, setup

package_name = 'vacuum_bluetooth'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Add launch files if you create them later
        # (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools', 'pybluez'], # Add pybluez dependency
    zip_safe=True,
    maintainer='Reeve',
    maintainer_email='user@example.com',
    description='ROS 2 package for Bluetooth communication',
    license='Apache License 2.0', # Or your preferred license
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bluetooth_node = vacuum_bluetooth.bluetooth_node:main',
        ],
    },
)
