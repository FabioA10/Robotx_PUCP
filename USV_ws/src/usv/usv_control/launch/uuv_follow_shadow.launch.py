"""Run the complete hardware-free UUV-follow shadow pipeline."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Start fake USBL, follower and a non-actuating command arbiter."""
    follow_enabled = LaunchConfiguration('follow_enabled')
    shadow_enabled = LaunchConfiguration('shadow_enabled')
    usbl_source = LaunchConfiguration('usbl_source')
    dropout_every_s = LaunchConfiguration('dropout_every_s')
    dropout_duration_s = LaunchConfiguration('dropout_duration_s')

    return LaunchDescription([
        DeclareLaunchArgument(
            'follow_enabled',
            default_value='false',
            description='Enable follower candidate calculation only',
        ),
        DeclareLaunchArgument(
            'shadow_enabled',
            default_value='false',
            description=(
                'Separate operator gate for non-actuating shadow arbitration'
            ),
        ),
        DeclareLaunchArgument(
            'usbl_source',
            default_value='fake',
            choices=['fake', 'external'],
            description=(
                'Exclusive USBL source: fake starts fake_usbl_node; external '
                'expects exactly one already-running publisher (real or replay)'
            ),
        ),
        DeclareLaunchArgument(
            'dropout_every_s',
            default_value='0.0',
            description='Period between simulated USBL dropouts; zero disables them',
        ),
        DeclareLaunchArgument(
            'dropout_duration_s',
            default_value='3.0',
            description='Duration of each simulated USBL dropout',
        ),
        Node(
            package='seatrac_ros2',
            executable='fake_usbl_node',
            name='fake_usbl_node',
            output='screen',
            condition=IfCondition(PythonExpression([
                "'", usbl_source, "' == 'fake'",
            ])),
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
        Node(
            package='usv_control',
            executable='usv_command_shadow_node',
            name='usv_command_shadow_node',
            output='screen',
            parameters=[{
                'shadow_enabled': ParameterValue(
                    shadow_enabled, value_type=bool
                ),
            }],
        ),
    ])
