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
<<<<<<< HEAD
    maintainer='jeonguk',
    maintainer_email='ju460648@gmail.com',
=======
    maintainer='sa',
    maintainer_email='jsa01jsa15@gmail.com',
>>>>>>> 753f14b ([Feat] web ui 및 웹으로 로봇팔 제어 기능)
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
<<<<<<< HEAD
<<<<<<< HEAD
            'move_scraper = cobot1.move_scraper:main',
            'move_basic = cobot1.move_basic:main',
            'move_periodic = cobot1.move_periodic:main',
            'grip_test = cobot1.grip_test:main',
            'mini_jog = cobot1.mini_jog:main'
=======
            'move_basic=cobot1.move_basic:main',
            'move_periodic=cobot1.move_periodic:main',
            'sensor=cobot1.sensor_test:main',
            'fire=cobot1.firebase_bridge:main',
            'fire2=cobot1.fire2:main',
            'basic2=cobot1.basic2:main',
            'tile_pic = cobot1.tile_pic_place3:main',
            'tile_no_gripper = cobot1.tile_pic_place3_no_gripper:main',
=======
            'move_scraper =        cobot1.move_scraper:main',
            'move_basic =          cobot1.move_basic:main',
            'move_periodic =       cobot1.move_periodic:main',
            'grip_test =           cobot1.grip_test:main',
            'mini_jog =            cobot1.mini_jog:main',
            'sensor=               cobot1.sensor_test:main',
            'fire=                 cobot1.firebase_bridge:main',
            'fire2=                cobot1.fire2:main',
            'basic2=               cobot1.basic2:main',
            'tile_pic =            cobot1.tile_pic_place3:main',
            'tile_no_gripper =     cobot1.tile_pic_place3_no_gripper:main',
>>>>>>> 0846997 ([bug fix] entry_points  syntax error fix ,  duplicate entry_points fix)
            'basic2_tile_pic_add = cobot1.basic2_tile_pic_add:main',
>>>>>>> 753f14b ([Feat] web ui 및 웹으로 로봇팔 제어 기능)
        ],
    },
)
