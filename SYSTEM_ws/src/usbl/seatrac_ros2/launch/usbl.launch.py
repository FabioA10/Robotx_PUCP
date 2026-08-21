#!/usr/bin/env python3

import os

from ament_index_python.packages import (
    get_package_share_directory,
)

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)

from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


VALID_ENVIRONMENTS = (
    'freshwater',
    'seawater',
)


def launch_setup(context):

    environment = LaunchConfiguration(
        'environment'
    ).perform(context)

    if environment not in VALID_ENVIRONMENTS:
        raise RuntimeError(
            'Invalid USBL environment: '
            f'{environment}. '
            'Valid options are: '
            'freshwater, seawater.'
        )

    package_share = get_package_share_directory(
        'seatrac_ros2'
    )

    config_file = os.path.join(
        package_share,
        'config',
        f'{environment}.yaml',
    )

    udp_node = Node(
        package='seatrac_ros2',
        executable='seatrac_udp_node',
        name='seatrac_udp_node',
        output='screen',
        parameters=[
            config_file,
        ],
    )

    status_node = Node(
        package='seatrac_ros2',
        executable='seatrac_status_node',
        name='seatrac_status_node',
        output='screen',
    )

    return [
        udp_node,
        status_node,
    ]


def generate_launch_description():

    return LaunchDescription([

        DeclareLaunchArgument(
            'environment',
            default_value='freshwater',
            description=(
                'USBL environment profile: '
                'freshwater or seawater'
            ),
        ),

        OpaqueFunction(
            function=launch_setup,
        ),

    ])
