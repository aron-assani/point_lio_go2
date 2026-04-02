from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mocap_odometry'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'motioncapture'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@todo.todo',
    description='Broadcasts OptiTrack motion capture data as ROS 2 Odometry',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'log_node = mocap_odometry.log_node:main',
            'publish_node = mocap_odometry.publish_node:main',
            'transform_node = mocap_odometry.transform_node:main',
        ],
    },
)

