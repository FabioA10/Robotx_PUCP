import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'usv_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robotx',
    maintainer_email='robotx@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'navigation_node = usv_control.navigation_node:main',
            'beacon_control = usv_control.beacon_control:main',
            'teleop_node = usv_control.teleop_node:main',
            'position_control_node = usv_control.position_control_node:main',
            'reel_control = usv_control.reel_control:main',
            'reel_control_node = usv_control.reel_control:main',
        ],
    },
)
