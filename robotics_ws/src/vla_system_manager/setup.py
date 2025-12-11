from setuptools import setup

package_name = 'vla_system_manager'

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
    description='Vision-Language-Action system manager and orchestrator',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vla_orchestrator = vla_system_manager.vla_orchestrator:main',
            'complex_task_demo = vla_system_manager.complex_task_demo:main',
        ],
    },
)