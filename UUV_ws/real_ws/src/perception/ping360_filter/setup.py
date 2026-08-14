from setuptools import find_packages, setup


package_name = 'ping360_filter'


setup(
    name=package_name,
    version='0.0.0',

    packages=find_packages(exclude=['test']),

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

    maintainer='luisito',
    maintainer_email='luisito@example.com',

    description=(
        'Filtro de nube de puntos generada por el sonar Ping360.'
    ),

    license='Apache-2.0',

    tests_require=[
        'pytest',
    ],

    entry_points={
        'console_scripts': [
            (
                'ping360_filter_node = '
                'ping360_filter.ping360_filter_node:main'
            ),
        ],
    },
)
