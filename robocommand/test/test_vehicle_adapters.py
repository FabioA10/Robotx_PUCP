"""Unit tests for UAV (PX4), USV (ArduPilot/ArduRover), and UUV (ArduSub) MAVROS2 adapters and parsers."""

import unittest

from robocommand.mavros_adapter import (
    LANDED_STATE_IN_AIR,
    LANDED_STATE_ON_GROUND,
    MAV_STATE_EMERGENCY,
    VehicleTelemetrySnapshot,
    map_ardupilot_usv_to_robot_state,
    map_ardupilot_uuv_to_robot_state,
    map_px4_to_flight_phase,
    map_px4_to_robot_state,
    parse_beacon_state,
    parse_color,
    parse_pipeline_segment,
    parse_task_param,
    parse_tier_param,
)
from robocommand.proto_gen import common_pb2, rx_common_pb2


class TestVehicleAdaptersAndParsers(unittest.TestCase):
    def test_px4_uav_state_and_flight_phase(self):
        snap = VehicleTelemetrySnapshot(
            connected=True,
            armed=True,
            mode="AUTO.LOITER",
            landed_state=LANDED_STATE_ON_GROUND,
        )
        self.assertEqual(map_px4_to_robot_state(snap), common_pb2.STATE_AUTO)
        self.assertEqual(
            map_px4_to_flight_phase(snap), rx_common_pb2.FLIGHT_PHASE_GROUNDED
        )

        snap.mode = "AUTO.MISSION"
        snap.landed_state = LANDED_STATE_IN_AIR
        self.assertEqual(map_px4_to_robot_state(snap), common_pb2.STATE_AUTO)
        self.assertEqual(
            map_px4_to_flight_phase(snap), rx_common_pb2.FLIGHT_PHASE_AIRBORNE
        )

        snap.mode = "POSCTL"
        self.assertEqual(map_px4_to_robot_state(snap), common_pb2.STATE_MANUAL)

        snap.system_status = MAV_STATE_EMERGENCY
        self.assertEqual(map_px4_to_robot_state(snap), common_pb2.STATE_KILLED)

    def test_ardupilot_usv_state_mapping(self):
        snap = VehicleTelemetrySnapshot(connected=True, armed=True, mode="HOLD")
        self.assertEqual(
            map_ardupilot_usv_to_robot_state(snap), common_pb2.STATE_AUTO
        )

        for auto_mode in ("LOITER", "AUTO", "GUIDED", "RTL", "SMART_RTL"):
            snap.mode = auto_mode
            self.assertEqual(
                map_ardupilot_usv_to_robot_state(snap), common_pb2.STATE_AUTO
            )

        for manual_mode in ("MANUAL", "ACRO", "STEERING"):
            snap.mode = manual_mode
            self.assertEqual(
                map_ardupilot_usv_to_robot_state(snap), common_pb2.STATE_MANUAL
            )

        snap.kill_switch_engaged = True
        self.assertEqual(
            map_ardupilot_usv_to_robot_state(snap), common_pb2.STATE_KILLED
        )

    def test_ardupilot_uuv_state_mapping(self):
        snap = VehicleTelemetrySnapshot(connected=True, armed=True, mode="POSHOLD")
        self.assertEqual(
            map_ardupilot_uuv_to_robot_state(snap), common_pb2.STATE_AUTO
        )

        for auto_mode in (
            "ALT_HOLD",
            "DEPTH_HOLD",
            "HOLD",
            "LOITER",
            "AUTO",
            "GUIDED",
            "SURFACE",
        ):
            snap.mode = auto_mode
            self.assertEqual(
                map_ardupilot_uuv_to_robot_state(snap), common_pb2.STATE_AUTO
            )

        for manual_mode in ("MANUAL", "STABILIZE", "ACRO"):
            snap.mode = manual_mode
            self.assertEqual(
                map_ardupilot_uuv_to_robot_state(snap), common_pb2.STATE_MANUAL
            )

        snap.kill_switch_engaged = True
        self.assertEqual(
            map_ardupilot_uuv_to_robot_state(snap), common_pb2.STATE_KILLED
        )

    def test_parsers_enforce_protocol_constraints(self):
        self.assertEqual(parse_tier_param("none"), common_pb2.TIER_NONE)
        self.assertEqual(parse_tier_param("advanced"), common_pb2.TIER_ADVANCED)
        with self.assertRaises(ValueError):
            parse_tier_param("unknown")

        self.assertEqual(
            parse_task_param("safe-passage"), rx_common_pb2.TASK_SAFE_PASSAGE
        )
        with self.assertRaises(ValueError):
            parse_task_param("unknown")

        self.assertEqual(
            parse_beacon_state("flashing_blue"),
            rx_common_pb2.BEACON_STATE_FLASHING_BLUE,
        )
        self.assertEqual(
            parse_pipeline_segment("damaged"),
            rx_common_pb2.PIPELINE_SEGMENT_DAMAGED,
        )
        self.assertEqual(parse_color("any"), rx_common_pb2.COLOR_ANY)


if __name__ == "__main__":
    unittest.main()
