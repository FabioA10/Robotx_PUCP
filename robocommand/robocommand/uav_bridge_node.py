#!/usr/bin/env python3
"""ROS 2 RoboCommand Bridge Node for a PX4 UAV via QGroundControl + MAVROS2."""

from __future__ import annotations

import rclpy

from robocommand.base_bridge_node import BaseRoboCommandBridgeNode
from robocommand.mavros_adapter import (
    VehicleTelemetrySnapshot,
    map_px4_to_robot_state,
)
from robocommand.proto_gen import rx_common_pb2


class RoboCommandUavBridgeNode(BaseRoboCommandBridgeNode):
    """Bridges MAVROS2 PX4 UAV telemetry, geofence declaration, and mission control with RoboCommand."""

    def __init__(self) -> None:
        super().__init__(
            node_name="robocommand_uav_bridge",
            default_vehicle_id="UAV1",
            vehicle_type_enum=rx_common_pb2.TYPE_UAV,
            default_mavros_ns="/mavros",
            default_run_start_mode="AUTO.MISSION",
        )

    def _map_telemetry_to_robot_state(
        self, snap: VehicleTelemetrySnapshot
    ) -> int:
        return map_px4_to_robot_state(
            snap, require_armed_for_auto=self._require_armed_for_auto
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RoboCommandUavBridgeNode()
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
