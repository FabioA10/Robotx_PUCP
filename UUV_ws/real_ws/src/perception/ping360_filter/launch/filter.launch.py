import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory('ping360_filter')
    default_config = os.path.join(package_share, 'config', 'filter.yaml')
    default_rviz = os.path.join(package_share, 'rviz', 'ping360_filter.rviz')

    return LaunchDescription(
        [
            DeclareLaunchArgument('config', default_value=default_config),
            DeclareLaunchArgument('rviz', default_value='true'),
            DeclareLaunchArgument('use_sim_time', default_value='true'),
            DeclareLaunchArgument(
                'contour_max_range_m',
                default_value='2.0',
                description='Radio maximo del contorno filtrado, en metros.',
            ),
            Node(
                package='ping360_filter',
                executable='ping360_filter_node',
                name='ping360_filter',
                output='screen',
                parameters=[
                    LaunchConfiguration('config'),
                    {
                        'use_sim_time': LaunchConfiguration('use_sim_time'),
                        'contour_max_range_m': ParameterValue(
                            LaunchConfiguration('contour_max_range_m'),
                            value_type=float,
                        ),
                    },
                ],
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                name='ping360_filter_rviz',
                arguments=['-d', default_rviz],
                output='screen',
                condition=IfCondition(LaunchConfiguration('rviz')),
            ),
        ]
    )
