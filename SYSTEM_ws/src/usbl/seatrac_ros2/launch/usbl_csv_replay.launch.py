"""Replay normalized USBL fixes without transport, simulator or actuation."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Return the normalized-position replay profile."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'csv_path',
                description='Absolute path to a reviewed normalized USBL CSV file',
            ),
            DeclareLaunchArgument(
                'expected_beacon_id',
                default_value='2',
                description='Only this remote beacon is accepted',
            ),
            DeclareLaunchArgument(
                'speed',
                default_value='1.0',
                description='Replay speed multiplier',
            ),
            DeclareLaunchArgument(
                'loop',
                default_value='false',
                description='Loop only for deliberate hardware-free testing',
            ),
            LogInfo(
                msg=(
                    'USBL REPLAY profile: normalized CSV source; '
                    'real transport=false; fake source=false; actuation=false.'
                )
            ),
            Node(
                package='seatrac_ros2',
                executable='seatrac_csv_replay',
                name='seatrac_csv_replay',
                output='screen',
                parameters=[
                    {
                        'csv_path': LaunchConfiguration('csv_path'),
                        'expected_beacon_id': ParameterValue(
                            LaunchConfiguration('expected_beacon_id'),
                            value_type=int,
                        ),
                        'speed': ParameterValue(
                            LaunchConfiguration('speed'), value_type=float
                        ),
                        'loop': ParameterValue(
                            LaunchConfiguration('loop'), value_type=bool
                        ),
                    }
                ],
            ),
        ]
    )
