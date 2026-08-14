from setuptools import setup
import os
from glob import glob

package_name = 'bluerov2_camera'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tu Nombre',
    maintainer_email='tu_email@dominio.com',
    description='BlueROV2 camera publisher node',
    license='MIT',
    entry_points={
        'console_scripts': [
            'video_publisher = bluerov2_camera.video_publisher:main',
        ],
    },
)

