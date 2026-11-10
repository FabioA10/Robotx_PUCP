"""Combined Launch file for all three RoboCommand Vehicle Bridges: UAV (PX4), USV (ArduPilot), and UUV (ArduSub)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(val: str) -> bool:
    return val.strip().lower() in ("true", "1", "yes", "on")


def _launch_setup(context, *args, **kwargs):
    params_file = LaunchConfiguration("params_file").perform(context)
    broker_host = LaunchConfiguration("broker_host").perform(context)
    broker_port = int(LaunchConfiguration("broker_port").perform(context))
    team_id = LaunchConfiguration("team_id").perform(context)

    enable_uav = _as_bool(LaunchConfiguration("enable_uav").perform(context))
    enable_usv = _as_bool(LaunchConfiguration("enable_usv").perform(context))
    enable_uuv = _as_bool(LaunchConfiguration("enable_uuv").perform(context))
    launch_mavros = _as_bool(
        LaunchConfiguration("launch_mavros").perform(context)
    )

    uav_fcu_url = LaunchConfiguration("uav_fcu_url").perform(context)
    usv_fcu_url = LaunchConfiguration("usv_fcu_url").perform(context)
    uuv_fcu_url = LaunchConfiguration("uuv_fcu_url").perform(context)

    participating = []
    if enable_usv:
        participating.append("USV1")
    if enable_uuv:
        participating.append("UUV1")
    if enable_uav:
        participating.append("UAV1")

    uav_decl_role = enable_uav
    usv_decl_role = (not enable_uav) and enable_usv
    uuv_decl_role = (not enable_uav) and (not enable_usv) and enable_uuv

    nodes = []

    if enable_uav:
        if launch_mavros:
            nodes.append(
                Node(
                    package="mavros",
                    executable="mavros_node",
                    output="screen",
                    parameters=[
                        {
                            "fcu_url": uav_fcu_url,
                            "gcs_url": "",
                            "tgt_system": 1,
                            "tgt_component": 1,
                            "fcu_protocol": "v2.0",
                        }
                    ],
                )
            )
        nodes.append(
            Node(
                package="robocommand",
                executable="uav_bridge_node",
                name="robocommand_uav_bridge",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "broker_host": broker_host,
                        "broker_port": broker_port,
                        "team_id": team_id,
                        "All_participating_vehicle_ids": participating,
                        "send_declaration_role": uav_decl_role,
                        "auto_send_declaration_on_course": uav_decl_role,
                    },
                ],
            )
        )

    if enable_usv:
        if launch_mavros:
            nodes.append(
                Node(
                    package="mavros",
                    executable="mavros_node",
                    namespace="usv",
                    output="screen",
                    parameters=[
                        {
                            "fcu_url": usv_fcu_url,
                            "gcs_url": "",
                            "tgt_system": 1,
                            "tgt_component": 1,
                            "fcu_protocol": "v2.0",
                        }
                    ],
                )
            )
        nodes.append(
            Node(
                package="robocommand",
                executable="usv_bridge_node",
                name="robocommand_usv_bridge",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "broker_host": broker_host,
                        "broker_port": broker_port,
                        "team_id": team_id,
                        "All_participating_vehicle_ids": participating,
                        "send_declaration_role": usv_decl_role,
                        "auto_send_declaration_on_course": usv_decl_role,
                    },
                ],
            )
        )

    if enable_uuv:
        if launch_mavros:
            nodes.append(
                Node(
                    package="mavros",
                    executable="mavros_node",
                    namespace="uuv",
                    output="screen",
                    parameters=[
                        {
                            "fcu_url": uuv_fcu_url,
                            "gcs_url": "",
                            "tgt_system": 1,
                            "tgt_component": 1,
                            "fcu_protocol": "v2.0",
                        }
                    ],
                )
            )
        nodes.append(
            Node(
                package="robocommand",
                executable="uuv_bridge_node",
                name="robocommand_uuv_bridge",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "broker_host": broker_host,
                        "broker_port": broker_port,
                        "team_id": team_id,
                        "All_participating_vehicle_ids": participating,
                        "send_declaration_role": uuv_decl_role,
                        "auto_send_declaration_on_course": uuv_decl_role,
                    },
                ],
            )
        )

    return nodes


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("robocommand")
    default_params_file = os.path.join(
        pkg_share, "config", "robocommand_params.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params_file,
                description="Path to the combined ROS 2 YAML parameters file.",
            ),
            DeclareLaunchArgument(
                "broker_host",
                default_value="192.168.65.2",
                description="RoboCommand MQTT broker IP/hostname.",
            ),
            DeclareLaunchArgument(
                "broker_port",
                default_value="1883",
                description="RoboCommand MQTT broker TCP port.",
            ),
            DeclareLaunchArgument(
                "team_id",
                default_value="PUCP",
                description="Assigned RoboNation Team ID.",
            ),
            DeclareLaunchArgument(
                "enable_uav",
                default_value="true",
                description="Launch robocommand_uav_bridge.",
            ),
            DeclareLaunchArgument(
                "enable_usv",
                default_value="true",
                description="Launch robocommand_usv_bridge.",
            ),
            DeclareLaunchArgument(
                "enable_uuv",
                default_value="true",
                description="Launch robocommand_uuv_bridge.",
            ),
            DeclareLaunchArgument(
                "launch_mavros",
                default_value="false",
                description="Whether to also launch mavros_node instances for enabled vehicles.",
            ),
            DeclareLaunchArgument(
                "uav_fcu_url",
                default_value="udp://127.0.0.1:14555@",
                description="MAVLink UDP endpoint for PX4 UAV.",
            ),
            DeclareLaunchArgument(
                "usv_fcu_url",
                default_value="udp://127.0.0.1:14565@",
                description="MAVLink UDP endpoint for ArduPilot USV.",
            ),
            DeclareLaunchArgument(
                "uuv_fcu_url",
                default_value="udp://127.0.0.1:14575@",
                description="MAVLink UDP endpoint for ArduSub UUV.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
