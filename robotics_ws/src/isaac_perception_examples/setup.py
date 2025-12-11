import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'isaac_perception_examples'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include any launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Include any RViz config files
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Humanoid Robotics Book',
    maintainer_email='humanoid-robotics-book@example.com',
    description='Isaac ROS perception examples for the Physical AI & Humanoid Robotics book',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_detector_node = isaac_perception_examples.object_detector_node:main',
            'pointcloud_processor = isaac_perception_examples.pointcloud_processor:main',
            'sensor_fusion_node = isaac_perception_examples.sensor_fusion_node:main',
        ],
    },
)