from setuptools import setup
from glob import glob
import os

package_name = 'tilemate_main'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sa',
    maintainer_email='ju460648@gmail.com',
    description='',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'gripper_node = tilemate_main.gripper_node:main',
            'motion_node = tilemate_main.motion_node:main',
            'task_controller_node = tilemate_main.task_controller_node:main',
            'command_node = tilemate_main.command_node:main',
        ],
    },
)
