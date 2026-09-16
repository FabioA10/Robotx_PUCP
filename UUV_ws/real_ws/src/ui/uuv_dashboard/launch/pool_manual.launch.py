"""Supervised Xbox-pilot profile for later, manual-only pool trials.

All physical outputs remain false by default.  There is no sequence transport
and no GUI axis-control path in this launch: the established Xbox teleop node
is the only requested manual-command source.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    manual_output = LaunchConfiguration('manual_output')
    arm_control = LaunchConfiguration('arm_control')
    accessory_control = LaunchConfiguration('accessory_control')
    mode_control = LaunchConfiguration('mode_control')
    camera = LaunchConfiguration('camera')
    command_scale = LaunchConfiguration('command_scale')
    dashboard = Node(
        package='uuv_dashboard', executable='pool_dashboard_node', output='screen',
        parameters=[{
            'operator_profile': True,
            'enable_mode_requests': mode_control,
        }],
    )
    bridge = Node(
        package='uuv_mavlink', executable='mavlink_bridge_node', output='screen',
        parameters=[{
            'udp_port': 14552,
            'motor_power_on_threshold': 11.0,
            'motor_power_off_threshold': 8.0,
            'sys_status_rate_hz': 20.0,
            'enable_command_output': manual_output,
            'enable_arm_disarm': arm_control,
            'enable_accessory_output': accessory_control,
            'enable_mode_control': mode_control,
            'command_scale': command_scale,
            'require_manual_mode': True,
        }],
    )
    return LaunchDescription([
        DeclareLaunchArgument('camera', default_value='false'),
        DeclareLaunchArgument('manual_output', default_value='false'),
        DeclareLaunchArgument('arm_control', default_value='false'),
        DeclareLaunchArgument('accessory_control', default_value='false'),
        DeclareLaunchArgument('mode_control', default_value='false'),
        DeclareLaunchArgument('command_scale', default_value='0.20'),
        RegisterEventHandler(OnProcessExit(
            target_action=dashboard,
            on_exit=[EmitEvent(event=Shutdown(reason='Dashboard cerrado'))],
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=bridge,
            on_exit=[EmitEvent(event=Shutdown(reason='Puente MAVLink cerrado'))],
        )),
        bridge,
        Node(package='uuv_dashboard', executable='sequence_executor_node',
             output='screen', parameters=[{'enable_sequence_execution': False}]),
        Node(package='joy', executable='game_controller_node', output='screen',
             parameters=[{'autorepeat_rate': 20.0}]),
        Node(package='uuv_teleop', executable='xbox_teleop_node', output='screen'),
        Node(package='uuv_teleop', executable='audio_feedback_node', output='screen',
             parameters=[{'enabled': True, 'volume': 0.35}]),
        Node(package='bluerov2_camera', executable='video_publisher',
             output='screen', condition=IfCondition(camera)),
        dashboard,
    ])
