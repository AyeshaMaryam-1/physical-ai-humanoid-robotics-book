import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'ai_control_agent'

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
    description='AI control agent for the Physical AI & Humanoid Robotics book',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_ai_agent = ai_control_agent.simple_ai_agent:main',
            'gazebo_commander = ai_control_agent.gazebo_commander:main',
            'digital_twin_test = ai_control_agent.digital_twin_test:main',
        ],
    },
)