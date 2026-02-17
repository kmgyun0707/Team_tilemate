from setuptools import find_packages, setup

package_name = 'cobot1'

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
    maintainer='sa',
    maintainer_email='jsa01jsa15@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'move_basic=cobot1.move_basic:main',
            'move_periodic=cobot1.move_periodic:main',
            'sensor=cobot1.sensor_test:main',
            'fire=cobot1.firebase_bridge:main',
            'fire2=cobot1.fire2:main',
            'basic2=cobot1.basic2:main',
            'tile_pic = cobot1.tile_pic_place3:main',
            'tile_no_gripper = cobot1.tile_pic_place3_no_gripper:main',
            'basic2_tile_pic_add = cobot1.basic2_tile_pic_add:main',
        ],
    },
)
