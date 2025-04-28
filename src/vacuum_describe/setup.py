import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'vacuum_describe'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Include all URDF files
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.urdf*'))), # Include .urdf and potentially .xacro
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='reeve',
    maintainer_email='2544141826@qq.com',
    description='Package containing the URDF description and TF setup for the vacuum cleaner robot.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
