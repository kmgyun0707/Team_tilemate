from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'tilemate_main'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jeonguk',
    maintainer_email='ju460648@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'gripper_node = tilemate_main.gripper_node:main',
            'scraper_motion_node = tilemate_main.scraper_motion_node:main',
            'task_controller_node = tilemate_main.task_controller_node:main',
            'command_node = tilemate_main.command_node:main',
            'tile_motion_node = tilemate_main.tile_motion_node:main',
        ],
    },
)
