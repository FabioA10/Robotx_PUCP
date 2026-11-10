"""MAVROS2 telemetry extraction, PX4 / ArduPilot (ArduRover & ArduSub) state mapping, and protocol parsers for RoboCommand."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Set, Tuple

from robocommand.proto_gen import common_pb2, rx_common_pb2

# PX4 custom_mode strings reported by MAVROS2 /mavros/state (for UAV)
PX4_AUTO_MODES: Set[str] = {
    "AUTO.MISSION",
    "AUTO.LOITER",
    "AUTO.TAKEOFF",
    "AUTO.LAND",
    "AUTO.RTL",
    "AUTO.PRECLAND",
    "AUTO.FOLLOW_TARGET",
    "OFFBOARD",
}

PX4_MANUAL_MODES: Set[str] = {
    "MANUAL",
    "ALTCTL",
    "POSCTL",
    "POSCTL.POSCTL",
    "POSCTL.ORBIT",
    "ACRO",
    "STABILIZED",
    "RATTITUDE",
}

# ArduRover / ArduBoat custom_mode strings reported by MAVROS2 (for USV / UAS / ASV)
ARDUPILOT_USV_AUTO_MODES: Set[str] = {
    "AUTO",
    "GUIDED",
    "LOITER",
    "HOLD",
    "RTL",
    "SMART_RTL",
    "FOLLOW",
    "SIMPLE",
}

ARDUPILOT_USV_MANUAL_MODES: Set[str] = {
    "MANUAL",
    "ACRO",
    "STEERING",
    "INITIALISING",
}

# ArduSub custom_mode strings reported by MAVROS2 (for UUV)
ARDUPILOT_UUV_AUTO_MODES: Set[str] = {
    "AUTO",
    "GUIDED",
    "POSHOLD",
    "ALT_HOLD",
    "DEPTH_HOLD",
    "LOITER",
    "HOLD",
    "SURFACE",
    "CIRCLE",
}

ARDUPILOT_UUV_MANUAL_MODES: Set[str] = {
    "MANUAL",
    "STABILIZE",
    "ACRO",
    "TRANSECT",
    "INITIALISING",
}

# MAV_STATE enum values from MAVLink / mavros_msgs/State.system_status
MAV_STATE_EMERGENCY = 6
MAV_STATE_FLIGHT_TERMINATION = 7

# mavros_msgs/ExtendedState.landed_state constants
LANDED_STATE_UNDEFINED = 0
LANDED_STATE_ON_GROUND = 1
LANDED_STATE_IN_AIR = 2
LANDED_STATE_TAKEOFF = 3
LANDED_STATE_LANDING = 4

TIER_MAP = {
    "none": common_pb2.TIER_NONE,
    "core": common_pb2.TIER_CORE,
    "advanced": common_pb2.TIER_ADVANCED,
    "disruptive": common_pb2.TIER_DISRUPTIVE,
}

TASK_MAP = {
    "none": rx_common_pb2.TASK_NONE,
    "task_none": rx_common_pb2.TASK_NONE,
    "safe-passage": rx_common_pb2.TASK_SAFE_PASSAGE,
    "safe_passage": rx_common_pb2.TASK_SAFE_PASSAGE,
    "task_safe_passage": rx_common_pb2.TASK_SAFE_PASSAGE,
    "task1": rx_common_pb2.TASK_SAFE_PASSAGE,
    "infra-survey-repair": rx_common_pb2.TASK_INFRA_SURVEY_REPAIR,
    "infra_survey_repair": rx_common_pb2.TASK_INFRA_SURVEY_REPAIR,
    "task_infra_survey_repair": rx_common_pb2.TASK_INFRA_SURVEY_REPAIR,
    "task2": rx_common_pb2.TASK_INFRA_SURVEY_REPAIR,
    "coordinated-logistics": rx_common_pb2.TASK_COORDINATED_LOGISTICS,
    "coordinated_logistics": rx_common_pb2.TASK_COORDINATED_LOGISTICS,
    "task_coordinated_logistics": rx_common_pb2.TASK_COORDINATED_LOGISTICS,
    "task3": rx_common_pb2.TASK_COORDINATED_LOGISTICS,
    "dynamic-incident": rx_common_pb2.TASK_DYNAMIC_INCIDENT,
    "dynamic_incident": rx_common_pb2.TASK_DYNAMIC_INCIDENT,
    "task_dynamic_incident": rx_common_pb2.TASK_DYNAMIC_INCIDENT,
    "task4": rx_common_pb2.TASK_DYNAMIC_INCIDENT,
}

BEACON_STATE_MAP = {
    "off": rx_common_pb2.BEACON_STATE_OFF,
    "beacon_state_off": rx_common_pb2.BEACON_STATE_OFF,
    "flashing_red": rx_common_pb2.BEACON_STATE_FLASHING_RED,
    "flashing-red": rx_common_pb2.BEACON_STATE_FLASHING_RED,
    "beacon_state_flashing_red": rx_common_pb2.BEACON_STATE_FLASHING_RED,
    "red": rx_common_pb2.BEACON_STATE_FLASHING_RED,
    "flashing_green": rx_common_pb2.BEACON_STATE_FLASHING_GREEN,
    "flashing-green": rx_common_pb2.BEACON_STATE_FLASHING_GREEN,
    "beacon_state_flashing_green": rx_common_pb2.BEACON_STATE_FLASHING_GREEN,
    "green": rx_common_pb2.BEACON_STATE_FLASHING_GREEN,
    "flashing_blue": rx_common_pb2.BEACON_STATE_FLASHING_BLUE,
    "flashing-blue": rx_common_pb2.BEACON_STATE_FLASHING_BLUE,
    "beacon_state_flashing_blue": rx_common_pb2.BEACON_STATE_FLASHING_BLUE,
    "steady_blue": rx_common_pb2.BEACON_STATE_STEADY_BLUE,
    "steady-blue": rx_common_pb2.BEACON_STATE_STEADY_BLUE,
    "beacon_state_steady_blue": rx_common_pb2.BEACON_STATE_STEADY_BLUE,
}

PIPELINE_STATUS_MAP = {
    "intact": rx_common_pb2.PIPELINE_SEGMENT_INTACT,
    "green": rx_common_pb2.PIPELINE_SEGMENT_INTACT,
    "pipeline_segment_intact": rx_common_pb2.PIPELINE_SEGMENT_INTACT,
    "damaged": rx_common_pb2.PIPELINE_SEGMENT_DAMAGED,
    "red": rx_common_pb2.PIPELINE_SEGMENT_DAMAGED,
    "pipeline_segment_damaged": rx_common_pb2.PIPELINE_SEGMENT_DAMAGED,
}

COLOR_MAP = {
    "red": rx_common_pb2.COLOR_RED,
    "color_red": rx_common_pb2.COLOR_RED,
    "green": rx_common_pb2.COLOR_GREEN,
    "color_green": rx_common_pb2.COLOR_GREEN,
    "blue": rx_common_pb2.COLOR_BLUE,
    "color_blue": rx_common_pb2.COLOR_BLUE,
    "any": rx_common_pb2.COLOR_ANY,
    "color_any": rx_common_pb2.COLOR_ANY,
}


def parse_tier_param(tier_str: str) -> int:
    key = tier_str.strip().lower()
    if key not in TIER_MAP:
        raise ValueError(
            f"Invalid task tier '{tier_str}'. Allowed values: {list(TIER_MAP.keys())} "
            "(do not use 'unknown' as a substitute for 'none')"
        )
    return TIER_MAP[key]


def parse_task_param(task_str: str) -> int:
    key = task_str.strip().lower()
    if key not in TASK_MAP:
        raise ValueError(
            f"Invalid task '{task_str}'. Allowed values: {list(TASK_MAP.keys())} "
            "(never use TASK_UNKNOWN)"
        )
    return TASK_MAP[key]


def parse_beacon_state(val: Any) -> int:
    if isinstance(val, int):
        if val in (
            rx_common_pb2.BEACON_STATE_OFF,
            rx_common_pb2.BEACON_STATE_FLASHING_RED,
            rx_common_pb2.BEACON_STATE_FLASHING_GREEN,
            rx_common_pb2.BEACON_STATE_FLASHING_BLUE,
            rx_common_pb2.BEACON_STATE_STEADY_BLUE,
        ):
            return val
    key = str(val).strip().lower()
    if key not in BEACON_STATE_MAP:
        raise ValueError(
            f"Invalid beacon state '{val}'. Allowed: {list(BEACON_STATE_MAP.keys())}"
        )
    return BEACON_STATE_MAP[key]


def parse_pipeline_segment(val: Any) -> int:
    if isinstance(val, int):
        if val in (
            rx_common_pb2.PIPELINE_SEGMENT_INTACT,
            rx_common_pb2.PIPELINE_SEGMENT_DAMAGED,
        ):
            return val
    key = str(val).strip().lower()
    if key not in PIPELINE_STATUS_MAP:
        raise ValueError(
            f"Invalid pipeline segment status '{val}'. Allowed: {list(PIPELINE_STATUS_MAP.keys())}"
        )
    return PIPELINE_STATUS_MAP[key]


def parse_color(val: Any) -> int:
    if isinstance(val, int):
        if val in (
            rx_common_pb2.COLOR_RED,
            rx_common_pb2.COLOR_GREEN,
            rx_common_pb2.COLOR_BLUE,
            rx_common_pb2.COLOR_ANY,
        ):
            return val
    key = str(val).strip().lower()
    if key not in COLOR_MAP:
        raise ValueError(f"Invalid color '{val}'. Allowed: {list(COLOR_MAP.keys())}")
    return COLOR_MAP[key]


@dataclass
class VehicleTelemetrySnapshot:
    """Latest vehicle telemetry and state derived from MAVROS2 and ROS 2 topics."""

    connected: bool = False
    armed: bool = False
    mode: str = ""
    system_status: int = 0
    landed_state: int = LANDED_STATE_UNDEFINED
    kill_switch_engaged: bool = False

    latitude: float = 0.0
    longitude: float = 0.0
    altitude_hae_m: float = 0.0
    depth_m: float = 0.0
    spd_mps: float = 0.0
    heading_deg: float = 0.0
    roll_deg: float = 0.0
    pitch_deg: float = 0.0


# Backward-compatible alias for UAV code and tests
UavTelemetrySnapshot = VehicleTelemetrySnapshot


def quaternion_to_roll_pitch_deg(
    x: float, y: float, z: float, w: float
) -> Tuple[float, float]:
    """Convert a unit quaternion (x, y, z, w) to (roll_deg, pitch_deg)."""
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll_rad = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch_rad = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch_rad = math.asin(sinp)

    return (math.degrees(roll_rad), math.degrees(pitch_rad))


def map_px4_to_robot_state(
    snapshot: VehicleTelemetrySnapshot,
    require_armed_for_auto: bool = False,
) -> int:
    """Map PX4 state from MAVROS2 to RoboCommand `RobotState` enum (for UAV).

    - Hold (`AUTO.LOITER`), `AUTO.MISSION`, `OFFBOARD`, etc. map to `STATE_AUTO` so
      the UAV can hold position in `AUTO.LOITER` prior to `RunStart` while satisfying
      RoboCommand's requirement that all declared vehicles report `STATE_AUTO`.
    - Emergency / Flight Termination or explicit kill switch maps to `STATE_KILLED`.
    """
    if (
        snapshot.kill_switch_engaged
        or snapshot.system_status in (MAV_STATE_EMERGENCY, MAV_STATE_FLIGHT_TERMINATION)
    ):
        return common_pb2.STATE_KILLED

    mode_upper = snapshot.mode.strip().upper()
    if not snapshot.connected and not mode_upper:
        return common_pb2.STATE_UNKNOWN

    if mode_upper in PX4_AUTO_MODES or mode_upper.startswith("AUTO."):
        if require_armed_for_auto and not snapshot.armed:
            return common_pb2.STATE_MANUAL
        return common_pb2.STATE_AUTO

    if mode_upper in PX4_MANUAL_MODES:
        return common_pb2.STATE_MANUAL

    return common_pb2.STATE_MANUAL if snapshot.connected else common_pb2.STATE_UNKNOWN


def map_ardupilot_usv_to_robot_state(
    snapshot: VehicleTelemetrySnapshot,
    require_armed_for_auto: bool = False,
) -> int:
    """Map ArduRover / ArduBoat state from MAVROS2 to RoboCommand `RobotState` enum (for USV).

    - `HOLD` / `LOITER` / `AUTO` / `GUIDED` / `RTL` / `SMART_RTL` map to `STATE_AUTO`
      so the USV can station-keep in `HOLD` or `LOITER` before `RunStart` while
      reporting `STATE_AUTO` to RoboCommand.
    - `MANUAL` / `ACRO` / `STEERING` map to `STATE_MANUAL`.
    - Emergency / Flight Termination or explicit kill switch maps to `STATE_KILLED`.
    """
    if (
        snapshot.kill_switch_engaged
        or snapshot.system_status in (MAV_STATE_EMERGENCY, MAV_STATE_FLIGHT_TERMINATION)
    ):
        return common_pb2.STATE_KILLED

    mode_upper = snapshot.mode.strip().upper()
    if not snapshot.connected and not mode_upper:
        return common_pb2.STATE_UNKNOWN

    if mode_upper in ARDUPILOT_USV_AUTO_MODES:
        if require_armed_for_auto and not snapshot.armed:
            return common_pb2.STATE_MANUAL
        return common_pb2.STATE_AUTO

    if mode_upper in ARDUPILOT_USV_MANUAL_MODES:
        return common_pb2.STATE_MANUAL

    return common_pb2.STATE_MANUAL if snapshot.connected else common_pb2.STATE_UNKNOWN


def map_ardupilot_uuv_to_robot_state(
    snapshot: VehicleTelemetrySnapshot,
    require_armed_for_auto: bool = False,
) -> int:
    """Map ArduSub state from MAVROS2 to RoboCommand `RobotState` enum (for UUV).

    - `POSHOLD` / `ALT_HOLD` / `DEPTH_HOLD` / `HOLD` / `LOITER` / `AUTO` / `GUIDED` / `SURFACE`
      map to `STATE_AUTO` so the UUV can hold station before `RunStart` while
      reporting `STATE_AUTO` to RoboCommand.
    - `MANUAL` / `STABILIZE` / `ACRO` map to `STATE_MANUAL`.
    - Emergency / Flight Termination or explicit kill switch maps to `STATE_KILLED`.
    """
    if (
        snapshot.kill_switch_engaged
        or snapshot.system_status in (MAV_STATE_EMERGENCY, MAV_STATE_FLIGHT_TERMINATION)
    ):
        return common_pb2.STATE_KILLED

    mode_upper = snapshot.mode.strip().upper()
    if not snapshot.connected and not mode_upper:
        return common_pb2.STATE_UNKNOWN

    if mode_upper in ARDUPILOT_UUV_AUTO_MODES:
        if require_armed_for_auto and not snapshot.armed:
            return common_pb2.STATE_MANUAL
        return common_pb2.STATE_AUTO

    if mode_upper in ARDUPILOT_UUV_MANUAL_MODES:
        return common_pb2.STATE_MANUAL

    return common_pb2.STATE_MANUAL if snapshot.connected else common_pb2.STATE_UNKNOWN


def map_px4_to_flight_phase(snapshot: VehicleTelemetrySnapshot) -> int:
    """Map MAVROS2 `/mavros/extended_state` (`landed_state`) to RoboCommand `FlightPhase`.

    RoboCommand `FlightPhase` has:
      - `FLIGHT_PHASE_UNKNOWN = 0`
      - `FLIGHT_PHASE_GROUNDED = 1`
      - `FLIGHT_PHASE_AIRBORNE = 2`
    """
    if snapshot.landed_state == LANDED_STATE_ON_GROUND:
        return rx_common_pb2.FLIGHT_PHASE_GROUNDED

    if snapshot.landed_state in (
        LANDED_STATE_IN_AIR,
        LANDED_STATE_TAKEOFF,
        LANDED_STATE_LANDING,
    ):
        return rx_common_pb2.FLIGHT_PHASE_AIRBORNE

    # Fallback if `/mavros/extended_state` has not yet been received
    if not snapshot.connected:
        return rx_common_pb2.FLIGHT_PHASE_UNKNOWN
    return (
        rx_common_pb2.FLIGHT_PHASE_AIRBORNE
        if snapshot.armed
        else rx_common_pb2.FLIGHT_PHASE_GROUNDED
    )
