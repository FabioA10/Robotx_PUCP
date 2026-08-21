from setuptools import find_packages, setup


package_name = 'usv_mavlink'


setup(
    name=package_name,
    version='0.1.0',

    packages=find_packages(),

    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
    ],

    install_requires=[
        'setuptools',
    ],

    zip_safe=True,

    maintainer='RobotX PUCP',
    maintainer_email='a20216480@pucp.edu.pe',

    description=(
        'ROS 2 MAVLink interface for the '
        'RobotX PUCP BlueBoat.'
    ),

    license='Apache-2.0',

    entry_points={
        'console_scripts': [
            (
                'mavlink_bridge_node = '
                'usv_mavlink.mavlink_bridge_node:main'
            ),
        ],
    },
)
