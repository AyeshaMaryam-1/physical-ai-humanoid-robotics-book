from setuptools import setup
import os
from glob import glob

package_name = 'vla_llm_planner'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include config files
        (os.path.join('share', package_name, 'config'), glob('config/*.json')),
        # Include message definitions
        (os.path.join('share', package_name, 'msg'), glob('msg/*.msg')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Maintainer',
    maintainer_email='maintainer@todo.todo',
    description='Vision-Language-Action LLM planning package',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'llm_planner_node = vla_llm_planner.llm_planner_node:main',
        ],
    },
    package_data={
        'vla_llm_planner': ['msg/*.msg']
    },
)