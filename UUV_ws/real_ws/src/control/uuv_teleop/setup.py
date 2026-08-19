from setuptools import find_packages, setup

package_name = 'uuv_teleop'

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
            'xbox_teleop_node = uuv_teleop.xbox_teleop_node:main',
            'audio_feedback_node = uuv_teleop.audio_feedback_node:main',
        ],
    },
)
