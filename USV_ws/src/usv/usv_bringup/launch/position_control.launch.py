from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('acceptance_radius', default_value='0.5', description='Waypoint arrival tolerance in meters'),
        DeclareLaunchArgument('max_linear_speed', default_value='1.5', description='Maximum linear speed in m/s'),
        DeclareLaunchArgument('max_angular_speed', default_value='1.0', description='Maximum angular speed in rad/s'),
        DeclareLaunchArgument('pose_topic', default_value='/mavros/local_position/pose', description='Subscribed pose topic'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/mavros/setpoint_velocity/cmd_vel', description='Published cmd_vel topic'),
        
        Node(
            package='usv_control',
            executable='position_control_node',
            name='position_control_node',
            output='screen',
            parameters=[{
                'acceptance_radius': LaunchConfiguration('acceptance_radius'),
                'max_linear_speed': LaunchConfiguration('max_linear_speed'),
                'max_angular_speed': LaunchConfiguration('max_angular_speed'),
                'pose_topic': LaunchConfiguration('pose_topic'),
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            }]
        )
    ])
