import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'robot_control_basics'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Humanoid Robotics Book',
    maintainer_email='humanoid-robotics-book@example.com',
    description='Robot control basics for the Physical AI & Humanoid Robotics book',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vel_publisher = robot_control_basics.vel_publisher_node:main',
            'scan_subscriber = robot_control_basics.scan_subscriber_node:main',
        ],
    },
)