from setuptools import find_packages, setup

package_name = 'pipeline_perception'

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
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pipeline_detector_node = pipeline_perception.pipeline_detector_node:main',
            'pipeline_follower_node = pipeline_perception.pipeline_follower_node:main',
            'pipeline_mission_manager_node = pipeline_perception.pipeline_mission_manager_node:main',
        ],
    },
)
