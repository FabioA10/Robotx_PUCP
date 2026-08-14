from setuptools import find_packages, setup

package_name = 'usv_teleop'

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
    maintainer='robotx',
    maintainer_email='robotx@todo.todo',
    description='Manual teleoperation nodes for the RobotX USV.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'teleop_node = usv_teleop.keyboard_teleop_node:main',
        ],
    },
)
