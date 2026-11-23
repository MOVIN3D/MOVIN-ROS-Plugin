from glob import glob
import os

from setuptools import setup

package_name = 'movin_ros'

setup(
    name=package_name,
    version='3.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'test', 'data'), glob('test/data/*.pkl')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='DK Jang',
    maintainer_email='dk.jang@movin3d.com',
    description='Receive, record and replay real-time motion capture from MOVIN Studio in ROS 2 '
                '(movin_msgs/MocapFrame, TF skeleton, SDK-compatible raw-OSC recordings).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mocap_node = movin_ros.mocap_node:main',
            'recording_info = movin_ros.recording_info:main',
        ],
    },
)
