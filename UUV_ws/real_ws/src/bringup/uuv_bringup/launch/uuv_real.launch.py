from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    # ------------------------------------------------------------------
    # Launch arguments
    # ------------------------------------------------------------------

    sonar = LaunchConfiguration('sonar')
    usbl = LaunchConfiguration('usbl')
    camera = LaunchConfiguration('camera')
    control = LaunchConfiguration('control')
    camera_tilt = LaunchConfiguration('camera_tilt')

    usbl_port = LaunchConfiguration('usbl_port')

    ping360_host = LaunchConfiguration('ping360_host')
    ping360_port = LaunchConfiguration('ping360_port')
    ping360_range = LaunchConfiguration('ping360_range')
    ping360_threshold = LaunchConfiguration('ping360_threshold')
    ping360_angle_step = LaunchConfiguration('ping360_angle_step')

    mavlink_url = LaunchConfiguration('mavlink_url')

    command_output = LaunchConfiguration('command_output')
    command_scale = LaunchConfiguration('command_scale')

    telemetry = LaunchConfiguration('telemetry')
    xbox = LaunchConfiguration('xbox')
    arm_control = LaunchConfiguration('arm_control')

    return LaunchDescription([

        # ==============================================================
        # Enable / disable subsystems
        # ==============================================================

        DeclareLaunchArgument(
            'sonar',
            default_value='true',
            description='Start Ping360 acquisition and filtering'
        ),

        DeclareLaunchArgument(
            'usbl',
            default_value='true',
            description='Start SeaTrac USBL nodes'
        ),

        DeclareLaunchArgument(
            'camera',
            default_value='true',
            description='Start BlueROV2 video publisher'
        ),

        DeclareLaunchArgument(
            'control',
            default_value='false',
            description='Start ROV velocity controller'
        ),

        DeclareLaunchArgument(
            'camera_tilt',
            default_value='false',
            description='Start BlueROV2 camera tilt controller'
        ),

        # ==============================================================
        # Hardware configuration
        # ==============================================================

        DeclareLaunchArgument(
            'usbl_port',
            default_value='/dev/ttyUSB0',
            description='Serial port used by SeaTrac USBL'
        ),

        DeclareLaunchArgument(
            'ping360_host',
            default_value='192.168.2.2',
            description='Ping360 IP address'
        ),

        DeclareLaunchArgument(
            'ping360_port',
            default_value='9092',
            description='Ping360 UDP port'
        ),

        DeclareLaunchArgument(
            'ping360_range',
            default_value='5.0',
            description='Ping360 range in meters'
        ),

        DeclareLaunchArgument(
            'ping360_threshold',
            default_value='25',
            description='Ping360 detection threshold'
        ),

        DeclareLaunchArgument(
            'ping360_angle_step',
            default_value='4',
            description='Ping360 angular step'
        ),

        DeclareLaunchArgument(
            'mavlink_url',
            default_value='udpin:0.0.0.0:14550',
            description='MAVLink endpoint used by vehicle control'
        ),

        DeclareLaunchArgument(
            'telemetry',
            default_value='true',
            description='Start UUV MAVLink bridge'
        ),

        DeclareLaunchArgument(
            'xbox',
            default_value='false',
            description='Start Xbox controller and UUV teleoperation node'
        ),
        DeclareLaunchArgument(
            'command_output',
            default_value='false',
            description='Enable MAVLink MANUAL_CONTROL transmission'
        ),

        DeclareLaunchArgument(
            'command_scale',
            default_value='0.20',
            description='Maximum MANUAL_CONTROL command scale'
        ),

        DeclareLaunchArgument(
            'arm_control',
            default_value='false',
            description='Enable Xbox ARM/DISARM commands through MAVLink'
        ),


        # ==============================================================
        # MAVLINK TELEMETRY
        #
        # Read-only vehicle telemetry:
        #   /uuv/connected
        #   /uuv/armed
        #   /uuv/mode
        #   /uuv/power/voltage
        #   /uuv/power/current
        #   /uuv/power/motors_enabled
        # ==============================================================

        Node(
            package='uuv_mavlink',
            executable='mavlink_bridge_node',
            name='uuv_mavlink_bridge',
            output='screen',
            condition=IfCondition(telemetry),
            parameters=[{
                'udp_port': 14552,
                'source_system_id': 255,
                'target_system_id': 1,
                'target_component_id': 1,
                'heartbeat_timeout': 3.0,
                'motor_power_on_threshold': 11.0,
                'motor_power_off_threshold': 8.0,
                'sys_status_rate_hz': 20.0,


                'enable_command_output': command_output,
                'command_scale': command_scale,
                'enable_arm_disarm': arm_control,
            }],
        ),

        # ==============================================================
        # XBOX TELEOPERATION
        #
        # Xbox -> /joy -> xbox_teleop_node
        #      -> /uuv/cmd_vel_manual
        #
        # IMPORTANT:
        # /uuv/cmd_vel_manual is NOT connected to ArduSub yet.
        # ==============================================================

        Node(
            package='joy',
            executable='game_controller_node',
            name='xbox_controller',
            output='screen',
            condition=IfCondition(xbox),
        ),

        Node(
            package='uuv_teleop',
            executable='xbox_teleop_node',
            name='xbox_teleop',
            output='screen',
            condition=IfCondition(xbox),
        ),

        Node(
            package='uuv_teleop',
            executable='audio_feedback_node',
            name='uuv_audio_feedback',
            output='screen',
            condition=IfCondition(xbox),
            parameters=[{
                'enabled': True,
                'volume': 0.35,
            }],
        ),


        # ==============================================================
        # SONAR - Ping360
        # ==============================================================

        Node(
            package='ping360_ros2',
            executable='ping360_pointcloud_node',
            name='ping360_pointcloud',
            output='screen',
            condition=IfCondition(sonar),
            parameters=[{
                'host': ping360_host,
                'port': ping360_port,
                'range_m': ping360_range,
                'threshold': ping360_threshold,
                'angle_step': ping360_angle_step,
            }],
        ),

        Node(
            package='ping360_filter',
            executable='ping360_filter_node',
            name='ping360_filter',
            output='screen',
            condition=IfCondition(sonar),
            parameters=[{
                'input_topic': '/ping360/points',
                'output_topic': '/ping360/filtered_points',
                'first_return_only': False,
            }],
        ),

        # ==============================================================
        # USBL - SeaTrac
        # ==============================================================

        Node(
            package='seatrac_ros2',
            executable='seatrac_serial_node',
            name='seatrac_serial',
            output='screen',
            condition=IfCondition(usbl),
            parameters=[{
                'port': usbl_port,
            }],
        ),

        Node(
            package='seatrac_ros2',
            executable='seatrac_status_node',
            name='seatrac_status',
            output='screen',
            condition=IfCondition(usbl),
        ),

        # ==============================================================
        # CAMERA
        #
        # BlueROV2 H264 stream
        #       ↓
        # bluerov2_camera
        #       ↓
        # /camera/image/compressed
        #       ↓
        # rqt_image_view
        # ==============================================================

        Node(
            package='bluerov2_camera',
            executable='video_publisher',
            name='bluerov2_video',
            output='screen',
            condition=IfCondition(camera),
        ),

        # --------------------------------------------------------------
        # Camera viewer
        #
        # Wait a few seconds so the camera publisher has time to start.
        # The image topic is passed directly to rqt_image_view so the
        # operator does not have to select it manually.
        # --------------------------------------------------------------

        TimerAction(
            period=2.0,

            actions=[

                Node(
                    package='rqt_image_view',
                    executable='rqt_image_view',
                    name='uuv_camera_view',
                    output='screen',
                    condition=IfCondition(camera),
                ),

            ],
        ),

        # ==============================================================
        # CAMERA TILT
        # ==============================================================

        Node(
            package='camera_control',
            executable='camera_tilt_controller_node',
            name='camera_tilt_controller',
            output='screen',
            condition=IfCondition(camera_tilt),
            parameters=[{
                'mavlink_url': mavlink_url,
                'channel': 8,
                'timeout_s': 1.0,
            }],
        ),

        # ==============================================================
        # VEHICLE CONTROL
        #
        # Disabled by default because this node can command/arm
        # the physical vehicle.
        # ==============================================================

        Node(
            package='rov_control',
            executable='velocity_controller_node',
            name='rov_velocity_controller',
            output='screen',
            condition=IfCondition(control),
        ),
    ])
