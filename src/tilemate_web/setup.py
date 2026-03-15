from setuptools import find_packages, setup
from glob import glob

package_name = 'tilemate_web'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_data={
        package_name: ['index.html', 'report_page.html'],
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.json')),
    ],
    install_requires=[
        'setuptools',
        'fastapi',
        'uvicorn',
        'requests',
    ],
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
            'fire=tilemate_web.firebase_bridge.firebase_bridge:main',
            'firebase=tilemate_web.firebase_bridge.firebase_bridge:main',
            'no=tilemate_web.test:main',
            'web=tilemate_web.websocket_bridge:main',
            'server=tilemate_web.server:main',
        ],
    },
)
