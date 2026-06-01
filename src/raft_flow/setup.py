from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'raft_flow'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hd',
    maintainer_email='wanghi6322@gmail.com',
    description='RAFT Optical Flow ROS 2 node with FLIR camera',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'flir_camera_node = raft_flow.flir_camera_node:main',
            'raft_flow_node   = raft_flow.raft_flow_node:main',
        ],
    },
)
