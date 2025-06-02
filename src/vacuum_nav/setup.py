from setuptools import setup
import os
from glob import glob

package_name = 'vacuum_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='reeve',
    maintainer_email='2544141826@qq.com',
    description='Navigation package for vacuum cleaner robot',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigation_node = vacuum_nav.navigation_node:main',
        ],
    },
)
