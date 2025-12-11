from setuptools import setup

package_name = 'vla_multi_modal'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Maintainer',
    maintainer_email='maintainer@todo.todo',
    description='Vision-Language-Action multi-modal fusion package',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fusion_node = vla_multi_modal.fusion_node:main',
            'simple_task_demo = vla_multi_modal.simple_task_demo:main',
        ],
    },
)