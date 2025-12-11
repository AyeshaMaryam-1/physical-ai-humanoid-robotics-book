from setuptools import setup

package_name = 'vla_voice_input'

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
    description='Vision-Language-Action voice input package using Whisper',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'whisper_node = vla_voice_input.whisper_node:main',
        ],
    },
)