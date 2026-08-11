import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    mavros_share = get_package_share_directory('mavros')
    
    pluginlists_yaml = os.path.join(mavros_share, 'launch', 'apm_pluginlists.yaml')
    config_yaml = os.path.join(mavros_share, 'launch', 'apm_config.yaml')

    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='udp://0.0.0.0:14550@',
        description='FCU connection URL'
    )

    gcs_url_arg = DeclareLaunchArgument(
        'gcs_url',
        default_value='',
        description='GCS connection URL'
    )

    tgt_system_arg = DeclareLaunchArgument(
        'tgt_system',
        default_value='1',
        description='Target system ID'
    )

    tgt_component_arg = DeclareLaunchArgument(
        'tgt_component',
        default_value='1',
        description='Target component ID'
    )

    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        namespace='mavros',
        output='screen',
        parameters=[
            pluginlists_yaml,
            config_yaml,
            {
                'fcu_url': LaunchConfiguration('fcu_url'),
                'gcs_url': LaunchConfiguration('gcs_url'),
                'target_system_id': LaunchConfiguration('tgt_system'),
                'target_component_id': LaunchConfiguration('tgt_component'),
                'fcu_protocol': 'v2.0',
            }
        ]
    )

    return LaunchDescription([
        fcu_url_arg,
        gcs_url_arg,
        tgt_system_arg,
        tgt_component_arg,
        mavros_node
    ])
