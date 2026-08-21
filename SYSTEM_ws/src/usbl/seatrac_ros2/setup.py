import os
from glob import glob

from setuptools import find_packages, setup


package_name = 'seatrac_ros2'


setup(
    name=package_name,
    version='0.1.0',

    packages=find_packages(
        exclude=['test']
    ),

    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
        (
            os.path.join(
                'share',
                package_name,
                'launch',
            ),
            glob('launch/*.launch.py'),
        ),
        (
            os.path.join(
                'share',
                package_name,
                'config',
            ),
            glob('config/*.yaml'),
        ),
    ],

    install_requires=[
        'setuptools',
    ],

    zip_safe=True,

    maintainer='RobotX PUCP',
    maintainer_email='a20216480@pucp.edu.pe',

    description=(
        'ROS 2 driver and system integration package for '
        'SeaTrac USBL used by RobotX PUCP.'
    ),

    license='Apache-2.0',

    extras_require={
        'test': [
            'pytest',
        ],
    },

    entry_points={
        'console_scripts': [
            (
                'seatrac_status_node = '
                'seatrac_ros2.seatrac_status_node:main'
            ),
            (
                'seatrac_serial_node = '
                'seatrac_ros2.seatrac_serial_node:main'
            ),
            (
                'seatrac_udp_node = '
                'seatrac_ros2.seatrac_udp_node:main'
            ),
        ],
    },
)
