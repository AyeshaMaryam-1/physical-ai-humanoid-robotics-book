import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'ros_basics'

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
    description='Basic ROS 2 examples for the Physical AI & Humanoid Robotics book',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = ros_basics.talker_node:main',
            'listener = ros_basics.listener_node:main',
        ],
    },
)