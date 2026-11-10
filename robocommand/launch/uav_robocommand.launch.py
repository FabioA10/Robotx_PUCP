"""Launch file for MAVROS2 (receiving forwarded MAVLink from QGC) and RoboCommand UAV Bridge."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("robocommand")
    default_params_file = os.path.join(pkg_share, "config", "uav_params.yaml")

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_file,
        description="Path to the ROS 2 YAML parameters file for robocommand_uav_bridge.",
    )
    broker_host_arg = DeclareLaunchArgument(
        "broker_host",
        default_value="192.168.65.2",
        description="RoboCommand MQTT broker IP/hostname (e.g. 192.168.65.2 or rc-test.local).",
    )
    broker_port_arg = DeclareLaunchArgument(
        "broker_port",
        default_value="1883",
        description="RoboCommand MQTT broker TCP port (e.g. 1883 or 1901).",
    )
    team_id_arg = DeclareLaunchArgument(
        "team_id",
        default_value="PUCP",
        description="Assigned RoboNation Team ID.",
    )
    vehicle_id_arg = DeclareLaunchArgument(
        "vehicle_id",
        default_value="UAV1",
        description="Unique vehicle identifier for the UAV.",
    )
    launch_mavros_arg = DeclareLaunchArgument(
        "launch_mavros",
        default_value="true",
        description="Whether to launch mavros_node listening to QGC's forwarded MAVLink UDP port.",
    )
    fcu_url_arg = DeclareLaunchArgument(
        "fcu_url",
        default_value="udp://127.0.0.1:14555@",
        description="MAVLink UDP endpoint matching QGroundControl MAVLink Forwarding target.",
    )
    tgt_system_arg = DeclareLaunchArgument(
        "tgt_system",
        default_value="1",
        description="MAVLink target system ID of the PX4 autopilot.",
    )

    mavros_node = Node(
        package="mavros",
        executable="mavros_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("launch_mavros")),
        parameters=[
            {
                "fcu_url": LaunchConfiguration("fcu_url"),
                "gcs_url": "",
                "tgt_system": ParameterValue(
                    LaunchConfiguration("tgt_system"), value_type=int
                ),
                "tgt_component": 1,
                "fcu_protocol": "v2.0",
            }
        ],
    )

    uav_bridge_node = Node(
        package="robocommand",
        executable="uav_bridge_node",
        name="robocommand_uav_bridge",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {
                "broker_host": LaunchConfiguration("broker_host"),
                "broker_port": ParameterValue(
                    LaunchConfiguration("broker_port"), value_type=int
                ),
                "team_id": LaunchConfiguration("team_id"),
                "vehicle_id": LaunchConfiguration("vehicle_id"),
            },
        ],
    )

    return LaunchDescription(
        [
            params_file_arg,
            broker_host_arg,
            broker_port_arg,
            team_id_arg,
            vehicle_id_arg,
            launch_mavros_arg,
            fcu_url_arg,
            tgt_system_arg,
            mavros_node,
            uav_bridge_node,
        ]
    )
