from setuptools import find_packages, setup

package_name = 'uuv_mavlink'

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
    maintainer_email='a20216480@pucp.edu.pe',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mavlink_bridge_node = uuv_mavlink.mavlink_bridge_node:main',
            'manual_control_preview_node = uuv_mavlink.manual_control_preview_node:main',
        ],
    },
)
