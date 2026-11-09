"""Run a simulated USBL target and the USV preview follower."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Start the complete hardware-free preview pipeline."""
    follow_enabled = LaunchConfiguration('follow_enabled')
    dropout_every_s = LaunchConfiguration('dropout_every_s')
    dropout_duration_s = LaunchConfiguration('dropout_duration_s')

    return LaunchDescription([
        DeclareLaunchArgument('follow_enabled', default_value='false'),
        DeclareLaunchArgument('dropout_every_s', default_value='0.0'),
        DeclareLaunchArgument('dropout_duration_s', default_value='3.0'),
        Node(
            package='seatrac_ros2',
            executable='fake_usbl_node',
            name='fake_usbl_node',
            output='screen',
            parameters=[{
                'dropout_every_s': ParameterValue(
                    dropout_every_s, value_type=float
                ),
                'dropout_duration_s': ParameterValue(
                    dropout_duration_s, value_type=float
                ),
            }],
        ),
        Node(
            package='usv_control',
            executable='uuv_follow_preview_node',
            name='uuv_follow_preview_node',
            output='screen',
            parameters=[{
                'follow_enabled': ParameterValue(
                    follow_enabled, value_type=bool
                ),
            }],
        ),
    ])
