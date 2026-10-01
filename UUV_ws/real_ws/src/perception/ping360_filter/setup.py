from glob import glob

from setuptools import find_packages, setup

package_name = 'ping360_filter'


setup(
    name=package_name,
    version='0.6.6',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Luis Muñoz',
    maintainer_email='luis@example.com',
    description='Ping360 pool-contour echo filtering for ROS 2 Jazzy.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'ping360_filter_node = ping360_filter.ping360_filter_node:main',
        ],
    },
)
