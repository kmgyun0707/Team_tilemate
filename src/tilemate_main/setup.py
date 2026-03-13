from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'tilemate_main'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'rule_based_img'), glob('resource/rule_based_img/*.jpg')),
        ('share/' + package_name, glob('resource/*.pt')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'resource'), ['resource/T_gripper2camera.npy']),
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
            'pick_tile_action_server = tilemate_main.pick_tile_action_server:main',
            'place_tile_action_server = tilemate_main.place_tile_action_server:main',
            'cowork_action_server = tilemate_main.cowork_action_server:main',
            'inspect_action_server = tilemate_main.inspect_action_server:main',
            'task_manager = tilemate_main.task_manager:main',
            'depthtest = tilemate_main.depthtest:main',
            'test_wall_plane_visualizer = tilemate_main.test_wall_plane_visualizer:main',
            'test_wall_tile_inspection_3d = tilemate_main.test_wall_tile_inspection_3d:main',
            'yolo_obb_cls_node = tilemate_main.yolo_obb_cls_node:main',
            'press_action_server = tilemate_main.press_action_server:main',
            'pattern_inspect_action_server = tilemate_main.pattern_inspect_action_server:main',
        ],
    },
)
