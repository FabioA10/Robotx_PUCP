from glob import glob
import os

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
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='robotx',
    maintainer_email='robotx@todo.todo',
    description='Control and navigation nodes for the RobotX USV.',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'navigation_node = usv_control.navigation_node:main',
            'position_control_node = usv_control.position_control_node:main',
            'uuv_follow_preview_node = usv_control.uuv_follow_preview_node:main',
        ],
    },
)
