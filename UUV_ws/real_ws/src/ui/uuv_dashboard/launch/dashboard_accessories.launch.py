from launch import LaunchDescription
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    dashboard = Node(
        package='uuv_dashboard',
        executable='dashboard_node',
        output='screen',
    )

    return LaunchDescription([
        RegisterEventHandler(
            OnProcessExit(
                target_action=dashboard,
                on_exit=[
                    EmitEvent(
                        event=Shutdown(reason='Dashboard cerrado')
                    ),
                ],
            )
        ),

        Node(
            package='uuv_mavlink',
            executable='mavlink_bridge_node',
            output='screen',
            parameters=[{
                'udp_port': 14552,
                'enable_command_output': False,
                'enable_arm_disarm': False,
                'enable_accessory_output': True,
                'enable_mode_control': True,
                'sys_status_rate_hz': 0.0,
            }],
        ),

        Node(
            package='bluerov2_camera',
            executable='video_publisher',
            output='screen',
        ),

        Node(
            package='joy',
            executable='game_controller_node',
            output='screen',
            parameters=[{
                'autorepeat_rate': 20.0,
            }],
        ),

        Node(
            package='uuv_teleop',
            executable='xbox_teleop_node',
            output='screen',
            remappings=[
                (
                    '/uuv/control/camera_tilt',
                    '/uuv/joystick/camera_tilt',
                ),
                (
                    '/uuv/control/lights_step',
                    '/uuv/joystick/lights_step',
                ),
            ],
        ),

        dashboard,
    ])
