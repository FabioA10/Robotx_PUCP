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
    command_scale = ParameterValue(LaunchConfiguration('command_scale'), value_type=float)
    bridge = Node(
        package='uuv_mavlink',
        executable='mavlink_bridge_node',
        output='both',
        parameters=[
            {
                'udp_port': 14552,
                'sequence_monitor_enabled': True,
                'enable_sequence_output': output,
                'enable_command_output': output,
                'enable_arm_disarm': output,
                'enable_mode_control': False,
                'require_manual_mode': True,
                'enable_accessory_output': False,
                'command_scale': command_scale,
                'sys_status_rate_hz': 0.0,
            }
        ],
    )
    executor = Node(
        package='uuv_mavlink',
        executable='sequence_executor_node',
        output='both',
        parameters=[
            {
                'water_density': ParameterValue(
                    LaunchConfiguration('water_density'), value_type=float
                ),
                'event_log_enabled': True,
                'event_log_directory': ParameterValue(
                    LaunchConfiguration('event_log_directory'), value_type=str
                ),
            }
        ],
    )
    dashboard = Node(package='uuv_dashboard', executable='sequence_dashboard_node', output='both')
    sonar = Node(
        package='ping360_ros2',
        executable='ping360_pointcloud_node',
        output='both',
        condition=IfCondition(LaunchConfiguration('sonar')),
        parameters=[
            {
                'host': ParameterValue(LaunchConfiguration('sonar_host'), value_type=str),
                'port': ParameterValue(LaunchConfiguration('sonar_port'), value_type=int),
                'range_m': ParameterValue(LaunchConfiguration('sonar_range'), value_type=float),
                'threshold': ParameterValue(
                    LaunchConfiguration('sonar_threshold'), value_type=int
                ),
                'angle_step': ParameterValue(
                    LaunchConfiguration('sonar_angle_step'), value_type=int
                ),
            }
        ],
    )
    sonar_filter = Node(
        package='ping360_filter',
        executable='ping360_filter_node',
        output='both',
        condition=IfCondition(LaunchConfiguration('sonar')),
        parameters=[
            {
                'input_topic': '/ping360/points',
                'filtered_topic': '/ping360/points_filtered',
                'legacy_filtered_topic': '/ping360/filtered_points',
                'filter_mode': 'pool_contour',
                'contour_max_range_m': ParameterValue(
                    LaunchConfiguration('sonar_filter_radius'), value_type=float
                ),
                'contour_min_intensity': ParameterValue(
                    LaunchConfiguration('sonar_filter_min_intensity'), value_type=float
                ),
            }
        ],
    )
    actions = [
        DeclareLaunchArgument(
            'enable_sequence_output',
            default_value='false',
            description='Explicitly enable physical GUIDED, Xbox arm and manual fallback',
        ),
        DeclareLaunchArgument('camera', default_value='true'),
        DeclareLaunchArgument(
            'sonar',
            default_value='false',
            description='Inicia adquisición y filtro Ping360 dentro de este launch',
        ),
        DeclareLaunchArgument('sonar_host', default_value='192.168.2.2'),
        DeclareLaunchArgument('sonar_port', default_value='9092'),
        DeclareLaunchArgument('sonar_range', default_value='5.0'),
        DeclareLaunchArgument(
            'sonar_threshold',
            default_value='1',
            description='Umbral de adquisición; el filtro aplica el umbral útil',
        ),
        DeclareLaunchArgument('sonar_angle_step', default_value='4'),
        DeclareLaunchArgument(
            'sonar_filter_radius',
            default_value='2.0',
            description='Radio local máximo para aceptar candidatos del contorno',
        ),
        DeclareLaunchArgument(
            'sonar_filter_min_intensity',
            default_value='90.0',
            description='Intensidad mínima del filtro de contorno',
        ),
        DeclareLaunchArgument(
            'command_scale',
            default_value='0.20',
            description='Escala del movimiento manual Xbox entre 0.0 y 1.0',
        ),
        DeclareLaunchArgument('water_density', default_value='1000.0'),
        DeclareLaunchArgument(
            'event_log_directory',
            default_value='',
            description='JSONL event directory; empty uses ~/.ros/uuv_sequence_logs',
        ),
    ]
    for node in (bridge, executor, dashboard):
        actions.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=node,
                    on_exit=[EmitEvent(event=Shutdown(reason='Proceso principal cerrado'))],
                )
            )
        )
    actions += [
        bridge,
        executor,
        sonar,
        sonar_filter,
        Node(
            package='joy',
            executable='game_controller_node',
            output='both',
            parameters=[{'autorepeat_rate': 20.0}],
        ),
        Node(package='uuv_teleop', executable='xbox_teleop_node', output='both'),
        Node(
            package='bluerov2_camera',
            executable='video_publisher',
            output='both',
            condition=IfCondition(LaunchConfiguration('camera')),
        ),
        dashboard,
    ]
    return LaunchDescription(actions)
