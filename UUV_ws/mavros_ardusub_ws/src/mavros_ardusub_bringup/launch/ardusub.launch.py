"""MAVROS bridge for ArduSub through a dedicated BlueOS UDP endpoint."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'fcu_url', default_value='udp://:14540@',
            description='BlueOS must send MAVLink UDP to this local port.',
        ),
        DeclareLaunchArgument(
            'gcs_url', default_value='',
            description='Optional MAVLink GCS forwarding URL.',
        ),
        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros',
            output='screen',
            parameters=[{
                'fcu_url': LaunchConfiguration('fcu_url'),
                'gcs_url': LaunchConfiguration('gcs_url'),
                'target_system_id': 1,
                'target_component_id': 1,
            }],
        ),
    ])
