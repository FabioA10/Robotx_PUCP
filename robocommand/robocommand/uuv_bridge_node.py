#!/usr/bin/env python3
"""ROS 2 RoboCommand Bridge Node for an ArduSub UUV via MAVROS2."""

from __future__ import annotations

import rclpy

from robocommand.base_bridge_node import BaseRoboCommandBridgeNode
from robocommand.mavros_adapter import (
    VehicleTelemetrySnapshot,
    map_ardupilot_uuv_to_robot_state,
)
from robocommand.proto_gen import rx_common_pb2


class RoboCommandUuvBridgeNode(BaseRoboCommandBridgeNode):
    """Bridges MAVROS2 ArduSub UUV telemetry (including depth_m), task reports, and mission control with RoboCommand."""

    def __init__(self) -> None:
        super().__init__(
            node_name="robocommand_uuv_bridge",
            default_vehicle_id="UUV1",
            vehicle_type_enum=rx_common_pb2.TYPE_UUV,
            default_mavros_ns="/uuv/mavros",
            default_run_start_mode="AUTO",
        )

    def _map_telemetry_to_robot_state(
        self, snap: VehicleTelemetrySnapshot
    ) -> int:
        return map_ardupilot_uuv_to_robot_state(
            snap, require_armed_for_auto=self._require_armed_for_auto
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RoboCommandUuvBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
