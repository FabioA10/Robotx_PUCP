from setuptools import find_packages, setup

package_name = 'rov_control'

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
    maintainer='luisito',
    maintainer_email='luisito@todo.todo',
    description='Control package for BlueROV2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'velocity_controller_node = rov_control.velocity_controller_node:main',
            'keyboard_teleop_node = rov_control.keyboard_teleop_node:main',
        ],
    },
)
