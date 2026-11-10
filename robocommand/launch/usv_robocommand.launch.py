"""Launch file for MAVROS2 (ArduPilot/ArduRover) and RoboCommand USV Bridge."""

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
    default_params_file = os.path.join(pkg_share, "config", "usv_params.yaml")

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_file,
        description="Path to the ROS 2 YAML parameters file for robocommand_usv_bridge.",
    )
    broker_host_arg = DeclareLaunchArgument(
        "broker_host",
        default_value="192.168.65.2",
        description="RoboCommand MQTT broker IP/hostname.",
    )
    broker_port_arg = DeclareLaunchArgument(
        "broker_port",
        default_value="1883",
        description="RoboCommand MQTT broker TCP port.",
    )
    team_id_arg = DeclareLaunchArgument(
        "team_id",
        default_value="PUCP",
        description="Assigned RoboNation Team ID.",
    )
    vehicle_id_arg = DeclareLaunchArgument(
        "vehicle_id",
        default_value="USV1",
        description="Unique vehicle identifier for the USV.",
    )
    launch_mavros_arg = DeclareLaunchArgument(
        "launch_mavros",
        default_value="true",
        description="Whether to launch mavros_node for the USV.",
    )
    fcu_url_arg = DeclareLaunchArgument(
        "fcu_url",
        default_value="udp://127.0.0.1:14565@",
        description="MAVLink endpoint for the ArduPilot USV.",
    )
    tgt_system_arg = DeclareLaunchArgument(
        "tgt_system",
        default_value="1",
        description="MAVLink target system ID of the USV autopilot.",
    )

    mavros_node = Node(
        package="mavros",
        executable="mavros_node",
        namespace="usv",
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

    usv_bridge_node = Node(
        package="robocommand",
        executable="usv_bridge_node",
        name="robocommand_usv_bridge",
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
            usv_bridge_node,
        ]
    )
