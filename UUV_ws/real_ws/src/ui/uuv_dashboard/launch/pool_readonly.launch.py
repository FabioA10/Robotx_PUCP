"""Passive pool inspection: no arm, mode, accessory or motion output."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    dashboard = Node(package='uuv_dashboard', executable='pool_dashboard_node',
                     output='screen')
    bridge = Node(
        package='uuv_mavlink', executable='mavlink_bridge_node', output='screen',
        parameters=[{
            'udp_port': 14552,
            'enable_command_output': False,
            'enable_arm_disarm': False,
            'enable_accessory_output': False,
            'enable_mode_control': False,
            'sys_status_rate_hz': 0.0,
        }],
    )
    return LaunchDescription([
        DeclareLaunchArgument('camera', default_value='true',
                              description='Start the existing video receiver'),
        RegisterEventHandler(OnProcessExit(
            target_action=dashboard,
            on_exit=[EmitEvent(event=Shutdown(reason='Dashboard cerrado'))],
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=bridge,
            on_exit=[EmitEvent(event=Shutdown(reason='Puente MAVLink cerrado'))],
        )),
        bridge,
        Node(package='bluerov2_camera', executable='video_publisher',
             output='screen', condition=IfCondition(LaunchConfiguration('camera'))),
        dashboard,
    ])
