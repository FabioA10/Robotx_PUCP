"""Start only the real SeaTrac transport needed for a raw capture."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


VALID_ENVIRONMENTS = ('freshwater', 'seawater')


def launch_setup(context):
    """Resolve the environment and create a non-actuating capture graph."""
    environment = LaunchConfiguration('environment').perform(context)
    if environment not in VALID_ENVIRONMENTS:
        raise RuntimeError(
            f'Invalid USBL environment: {environment}. '
            'Valid options are freshwater and seawater.'
        )

    package_share = get_package_share_directory('seatrac_ros2')
    config_file = os.path.join(
        package_share,
        'config',
        f'{environment}.yaml',
    )

    return [
        LogInfo(
            msg=(
                'USBL CAPTURE profile: real X150 transport; '
                'fake source=false; relative-pose decoder=false; '
                'follow=false; actuation=false.'
            )
        ),
        Node(
            package='seatrac_ros2',
            executable='seatrac_udp_node',
            name='seatrac_udp_node',
            output='screen',
            parameters=[config_file],
        ),
        Node(
            package='seatrac_ros2',
            executable='seatrac_status_node',
            name='seatrac_status_node',
            output='screen',
        ),
    ]


def generate_launch_description():
    """Return the raw-capture launch description."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'environment',
                default_value='freshwater',
                description='SeaTrac environment profile: freshwater or seawater',
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
