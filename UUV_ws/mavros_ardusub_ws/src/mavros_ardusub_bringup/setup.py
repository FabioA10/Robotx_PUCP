from setuptools import setup

package_name = 'mavros_ardusub_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/mavros_ardusub_bringup']),
        ('share/mavros_ardusub_bringup', ['package.xml']),
        ('share/mavros_ardusub_bringup/launch', ['launch/ardusub.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
)
