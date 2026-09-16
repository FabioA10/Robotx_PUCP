"""One bridge, one executor and one GUI; physical output requires explicit opt-in."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    output = ParameterValue(LaunchConfiguration('enable_sequence_output'), value_type=bool)
    bridge = Node(
        package='uuv_mavlink', executable='mavlink_bridge_node', output='screen',
        parameters=[{
            'udp_port': 14552, 'sequence_monitor_enabled': True,
            'enable_sequence_output': output, 'enable_command_output': output,
            'enable_arm_disarm': output, 'enable_mode_control': False,
            'require_manual_mode': True, 'enable_accessory_output': False,
            'command_scale': 0.20, 'sys_status_rate_hz': 0.0,
        }],
    )
    executor = Node(
        package='uuv_mavlink', executable='sequence_executor_node', output='screen',
        parameters=[{'water_density': ParameterValue(LaunchConfiguration('water_density'), value_type=float)}],
    )
    dashboard = Node(package='uuv_dashboard', executable='sequence_dashboard_node', output='screen')
    actions = [
        DeclareLaunchArgument('enable_sequence_output', default_value='false',
                              description='Explicitly enable physical GUIDED, Xbox arm and manual fallback'),
        DeclareLaunchArgument('camera', default_value='true'),
        DeclareLaunchArgument('water_density', default_value='1000.0'),
    ]
    for node in (bridge, executor, dashboard):
        actions.append(RegisterEventHandler(OnProcessExit(
            target_action=node,
            on_exit=[EmitEvent(event=Shutdown(reason='Proceso principal cerrado'))])))
    actions += [bridge, executor,
                Node(package='joy', executable='game_controller_node', output='screen',
                     parameters=[{'autorepeat_rate': 20.0}]),
                Node(package='uuv_teleop', executable='xbox_teleop_node', output='screen'),
                Node(package='bluerov2_camera', executable='video_publisher', output='screen',
                     condition=IfCondition(LaunchConfiguration('camera'))), dashboard]
    return LaunchDescription(actions)
