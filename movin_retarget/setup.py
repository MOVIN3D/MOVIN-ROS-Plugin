from glob import glob
import os

from setuptools import setup

package_name = 'movin_retarget'

setup(
    name=package_name,
    version='3.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='DK Jang',
    maintainer_email='dk.jang@movin3d.com',
    description='Optional Unitree G1 retargeting of MOVIN Studio motion capture frames '
                '(movin_msgs/MocapFrame -> sensor_msgs/JointState + TF).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'retarget_node = movin_retarget.retarget_node:main',
        ],
    },
)
