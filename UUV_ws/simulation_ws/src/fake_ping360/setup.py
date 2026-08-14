from setuptools import find_packages, setup

package_name = 'fake_ping360'

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
        	'fake_ping360_node = fake_ping360.fake_ping360_node:main',
        	'gz_ping360_text_bridge = fake_ping360.gz_ping360_text_bridge:main',
    	],

    },
)
