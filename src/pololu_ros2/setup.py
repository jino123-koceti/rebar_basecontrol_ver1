from setuptools import setup
import os
from glob import glob

package_name = 'pololu_ros2'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Matthew Wilson',
    maintainer_email='mattwilsonmbw@gmail.com',
    description='ROS2 driver for Pololu Simple Motor Controller',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pololu_node = pololu_ros2.pololu_node:main',
        ],
    },
)
