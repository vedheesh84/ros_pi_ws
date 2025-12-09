from setuptools import setup
import os
from glob import glob

package_name = 'ros2_arduino_bridge'

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
        (os.path.join('share', package_name, 'script'), glob('meshes/*.py') ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Varun',
    maintainer_email='varun@example.com',
    description='ROS2 Arduino Bridge for Motor Control',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arduino_bridge = ros2_arduino_bridge.arduino_bridge:main',
            'teleop_keyboard = ros2_arduino_bridge.teleop_keyboard:main',
        ],
    },
)
