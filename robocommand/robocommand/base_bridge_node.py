"""Base ROS 2 RoboCommand Bridge Node shared by UAV (PX4), USV (ArduPilot), and UUV (ArduSub)."""

from __future__ import annotations

from collections import deque
import json
import math
import os
import threading
import time
from typing import Any, Deque, Dict, List, Optional, Tuple

from robocommand.proto_gen import (
    common_pb2,
    rx_commands_pb2,
    rx_common_pb2,
    rx_course_pb2,
    rx_reports_pb2,
    rx_requests_pb2,
)
from robocommand.geofence import (
    LatLngTuple,
    derive_geofence_from_course,
    ensure_closed_ring,
    load_geofence_from_qgc_plan,
    validate_uav_geofence,
)
from robocommand.mavros_adapter import (
    VehicleTelemetrySnapshot,
    map_px4_to_flight_phase,
    quaternion_to_roll_pitch_deg,
)

from geometry_msgs.msg import PoseStamped, TwistStamped
from google.protobuf.timestamp_pb2 import Timestamp
from mavros_msgs.msg import ExtendedState, State, VFR_HUD
from mavros_msgs.srv import CommandBool, SetMode
import paho.mqtt.client as mqtt
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Bool, Float64, String, UInt32
from std_srvs.srv import Trigger

TOPIC_ROOT = "robocommand/robotx"
MAX_PUBLISH_PER_SEC = 5

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


def now_proto_timestamp() -> Timestamp:
    ts = Timestamp()
    ts.GetCurrentTime()
    return ts


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


class BaseRoboCommandBridgeNode(Node):
    """Shared base bridge node for UAV (PX4), USV (ArduPilot), and UUV (ArduSub)."""

    def __init__(
        self,
        node_name: str,
        default_vehicle_id: str,
        vehicle_type_enum: int,
        default_mavros_ns: str,
        default_run_start_mode: str,
    ) -> None:
        super().__init__(node_name)
        self._vehicle_type_enum = vehicle_type_enum

        # ── Declare ROS 2 Parameters ─────────────────────────────────────────
        self.declare_parameter("broker_host", "192.168.65.2")
        self.declare_parameter("broker_port", 1883)
        self.declare_parameter("team_id", "PUCP")
        self.declare_parameter("vehicle_id", default_vehicle_id)
        self.declare_parameter("All_participating_vehicle_ids", [default_vehicle_id])

        # Multi-vehicle declaration coordination
        self.declare_parameter("send_declaration_role", True)
        self.declare_parameter("auto_send_declaration_on_course", True)

        # UAV regulatory & declaration parameters
        self.declare_parameter("uav_pilot_callsign", "PILOT_UAPL12345")
        self.declare_parameter("uav_registration", "")
        self.declare_parameter("uav_geofence_max_altitude_hae_m", 40.0)

        # Task tier declarations (use "none", "core", "advanced", or "disruptive")
        self.declare_parameter("task1_tier", "none")
        self.declare_parameter("task2_tier", "none")
        self.declare_parameter("task3_tier", "none")
        self.declare_parameter("task4_tier", "none")

        # UAV Geofence configuration
        self.declare_parameter("auto_geofence_from_course", True)
        self.declare_parameter("include_uav_transit_area", True)
        self.declare_parameter("geofence_inset_m", 0.5)
        self.declare_parameter("geofence_plan_file", "")
        self.declare_parameter("geofence_lats", [0.0])
        self.declare_parameter("geofence_lngs", [0.0])

        # Heartbeat & task state parameters
        self.declare_parameter("heartbeat_rate_hz", 2.0)
        self.declare_parameter("require_armed_for_auto", False)
        self.declare_parameter("state_override", "")
        self.declare_parameter("current_task", "none")
        self.declare_parameter("initial_task_on_run_start", "none")

        # RunStart -> MAVROS2 mission trigger parameters
        # Support both generic and legacy PX4 parameter names
        self.declare_parameter("auto_trigger_mission_on_run_start", True)
        self.declare_parameter("auto_trigger_px4_mission_on_run_start", True)
        self.declare_parameter("auto_arm_on_run_start", False)
        self.declare_parameter("run_start_mode", default_run_start_mode)
        self.declare_parameter("px4_run_start_mode", default_run_start_mode)
        self.declare_parameter("enable_run_start_mode", False)
        self.declare_parameter("mission_start_requires_telemetry", True)
        self.declare_parameter(
            "mission_start_requires_local_position",
            vehicle_type_enum == rx_common_pb2.TYPE_UUV,
        )
        self.declare_parameter(
            "mission_start_requires_depth",
            vehicle_type_enum == rx_common_pb2.TYPE_UUV,
        )
        self.declare_parameter("mission_start_telemetry_timeout_s", 1.5)

        # Task 4 Dynamic Incident Response auto-acknowledgment
        self.declare_parameter("auto_ack_incidents", True)

        # UUV depth source configuration ("auto", "topic", "rel_alt", "vfr_hud")
        self.declare_parameter("depth_source", "auto")

        # MAVROS2 namespace
        self.declare_parameter("mavros_ns", default_mavros_ns)

        # ── Read Parameters ──────────────────────────────────────────────────
        self._broker_host: str = str(self.get_parameter("broker_host").value).strip()
        self._broker_port: int = int(self.get_parameter("broker_port").value)
        self._team_id: str = str(self.get_parameter("team_id").value).strip()
        self._vehicle_id: str = str(self.get_parameter("vehicle_id").value).strip()
        self._participating_vehicles: List[str] = [
            str(v).strip()
            for v in self.get_parameter("All_participating_vehicle_ids").value
            if str(v).strip()
        ]
        if self._vehicle_id not in self._participating_vehicles:
            self._participating_vehicles.append(self._vehicle_id)

        self._uav_pilot_callsign: str = str(
            self.get_parameter("uav_pilot_callsign").value
        ).strip()
        self._uav_registration: str = str(
            self.get_parameter("uav_registration").value
        ).strip()
        self._max_alt_hae_m: float = float(
            self.get_parameter("uav_geofence_max_altitude_hae_m").value
        )

        self._current_task_enum: int = parse_task_param(
            str(self.get_parameter("current_task").value)
        )
        self._require_armed_for_auto: bool = bool(
            self.get_parameter("require_armed_for_auto").value
        )
        self._state_override: str = (
            str(self.get_parameter("state_override").value).strip().lower()
        )
        self._depth_source: str = (
            str(self.get_parameter("depth_source").value).strip().lower()
        )
        self._last_topic_depth_time: float = 0.0
        self._last_state_time: float = 0.0
        self._last_local_position_time: float = 0.0
        self._last_depth_time: float = 0.0

        # ── Internal State & Sequence Counters (§3.4.8) ──────────────────────
        self._lock = threading.Lock()
        self._telemetry = VehicleTelemetrySnapshot()

        self._mqtt_connected: bool = False
        self._request_seq: int = 0
        self._report_seq: int = 0
        self._publish_timestamps: Deque[float] = deque()

        self._course_id: str = ""
        self._pinger_freq_hz: int = 0
        self._course_corners: List[LatLngTuple] = []
        self._transit_corners: List[LatLngTuple] = []

        self._active_declaration_seq: Optional[int] = None
        self._declaration_sent_for_course: bool = False
        self._run_started: bool = False
        self._run_id: Optional[int] = None

        self._last_command_seq: Optional[int] = None
        self._last_command_type: str = ""
        self._last_readiness_confirmed_seq: Optional[int] = None

        # ── Dynamic Parameter Callback ───────────────────────────────────────
        self.add_on_set_parameters_callback(self._on_parameters_set)

        # ── MAVROS2 Subscriptions & Service Clients ──────────────────────────
        mavros_ns = str(self.get_parameter("mavros_ns").value).rstrip("/")
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Subscribe to both <mavros_ns>/* and /mavros/mavros/* if default
        ns_candidates = [mavros_ns]
        if mavros_ns == "/mavros":
            ns_candidates.append("/mavros/mavros")

        for ns in ns_candidates:
            self.create_subscription(
                State, f"{ns}/state", self._on_mavros_state, state_qos
            )
            self.create_subscription(
                ExtendedState,
                f"{ns}/extended_state",
                self._on_mavros_extended_state,
                sensor_qos,
            )
            self.create_subscription(
                NavSatFix,
                f"{ns}/global_position/global",
                self._on_mavros_global_pos,
                sensor_qos,
            )
            self.create_subscription(
                Float64,
                f"{ns}/global_position/compass_hdg",
                self._on_mavros_compass_hdg,
                sensor_qos,
            )
            self.create_subscription(
                Float64,
                f"{ns}/global_position/rel_alt",
                self._on_mavros_rel_alt,
                sensor_qos,
            )
            self.create_subscription(
                VFR_HUD,
                f"{ns}/vfr_hud",
                self._on_mavros_vfr_hud,
                sensor_qos,
            )
            self.create_subscription(
                TwistStamped,
                f"{ns}/local_position/velocity_local",
                self._on_mavros_velocity,
                sensor_qos,
            )
            self.create_subscription(
                PoseStamped,
                f"{ns}/local_position/pose",
                self._on_mavros_local_position,
                sensor_qos,
            )
            self.create_subscription(
                Imu, f"{ns}/imu/data", self._on_mavros_imu, sensor_qos
            )

        self._set_mode_client = self.create_client(
            SetMode, f"{mavros_ns}/set_mode"
        )
        self._arming_client = self.create_client(
            CommandBool, f"{mavros_ns}/cmd/arming"
        )

        # ── Shared Multi-Node Coordination Topic ─────────────────────────────
        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._shared_decl_seq_pub = self.create_publisher(
            UInt32, "/robocommand/active_declaration_seq", latched_qos
        )
        self.create_subscription(
            UInt32,
            "/robocommand/active_declaration_seq",
            self._on_shared_declaration_seq,
            latched_qos,
        )

        # ── Operator & Autonomy ROS 2 Publishers ─────────────────────────────
        self._run_started_pub = self.create_publisher(
            Bool, "~/run_started", latched_qos
        )
        self._run_id_pub = self.create_publisher(UInt32, "~/run_id", latched_qos)
        self._course_pub = self.create_publisher(String, "~/course", latched_qos)
        self._pinger_pub = self.create_publisher(
            UInt32, "~/pinger_freq_hz", latched_qos
        )
        self._status_pub = self.create_publisher(String, "~/status", 10)

        # Task 4 Incoming Command Publishers
        self._cmd_assistance_pub = self.create_publisher(
            String, "~/command/assistance_request", 10
        )
        self._cmd_keep_out_pub = self.create_publisher(
            String, "~/command/keep_out_zone", 10
        )
        self._cmd_all_clear_pub = self.create_publisher(
            String, "~/command/all_clear", 10
        )
        self._cmd_moving_obj_pub = self.create_publisher(
            String, "~/command/moving_object_alert", 10
        )
        self._cmd_readiness_confirm_pub = self.create_publisher(
            String, "~/command/readiness_confirm", 10
        )

        # ── Operator & Autonomy ROS 2 Subscriptions & Services ───────────────
        self.create_subscription(String, "~/set_task", self._on_set_task_msg, 10)
        self.create_subscription(
            String, "~/state_override", self._on_state_override_msg, 10
        )
        self.create_subscription(Bool, "~/kill_override", self._on_kill_override, 10)
        self.create_subscription(Float64, "~/depth_m", self._on_depth_topic, 10)

        # Task 1-4 Outgoing Report Subscriptions
        self.create_subscription(
            String, "~/report/safe_passage", self._on_report_safe_passage, 10
        )
        self.create_subscription(
            String, "~/report/pipeline_survey", self._on_report_pipeline_survey, 10
        )
        self.create_subscription(
            String,
            "~/report/resource_delivery",
            self._on_report_resource_delivery,
            10,
        )
        self.create_subscription(
            UInt32, "~/report/docking", self._on_report_docking, 10
        )
        self.create_subscription(
            UInt32, "~/report/firefighting", self._on_report_firefighting, 10
        )
        self.create_subscription(
            UInt32, "~/report/incident_ack", self._on_report_incident_ack, 10
        )
        self.create_subscription(
            UInt32, "~/report/readiness", self._on_report_readiness, 10
        )

        self.create_service(
            Trigger, "~/send_declaration", self._on_send_declaration_srv
        )

        # Publish initial run_started = False
        self._run_started_pub.publish(Bool(data=False))

        # ── MQTT Client Setup (§3.4.5, §3.4.6) ───────────────────────────────
        self._topic_course = f"{TOPIC_ROOT}/course"
        self._topic_request = f"{TOPIC_ROOT}/{self._team_id}/request"
        self._topic_command = f"{TOPIC_ROOT}/{self._team_id}/command"
        self._topic_report = (
            f"{TOPIC_ROOT}/{self._team_id}/{self._vehicle_id}/report"
        )

        self._mqtt = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2,
            client_id=f"ocs_{self._team_id}_{self._vehicle_id}_{os.getpid()}",
        )
        self._mqtt.on_connect = self._on_mqtt_connect
        self._mqtt.on_disconnect = self._on_mqtt_disconnect
        self._mqtt.on_message = self._on_mqtt_message
        self._mqtt.reconnect_delay_set(min_delay=1, max_delay=5)

        self._start_mqtt_nonblocking()

        # ── Periodic Timers (2 Hz Heartbeat + 1 Hz Operator Status) ──────────
        hb_rate = float(self.get_parameter("heartbeat_rate_hz").value)
        hb_period = 1.0 / max(0.5, min(5.0, hb_rate))
        self._hb_timer = self.create_timer(hb_period, self._publish_heartbeat)
        self._status_timer = self.create_timer(1.0, self._publish_operator_status)

        vtype_str = rx_common_pb2.VehicleType.Name(self._vehicle_type_enum)
        self.get_logger().info(
            f"RoboCommand Bridge ({vtype_str}) started | team_id={self._team_id} "
            f"vehicle_id={self._vehicle_id} broker={self._broker_host}:{self._broker_port} "
            f"mavros_ns={mavros_ns}"
        )

    # ── Abstract / Overridable Vehicle State Mapping ─────────────────────────

    def _map_telemetry_to_robot_state(
        self, snap: VehicleTelemetrySnapshot
    ) -> int:
        raise NotImplementedError

    # ── Parameter & Topic Handlers ───────────────────────────────────────────

    def _on_parameters_set(self, params) -> SetParametersResult:
        for param in params:
            if param.name == "current_task":
                try:
                    new_task = parse_task_param(str(param.value))
                except ValueError as e:
                    return SetParametersResult(successful=False, reason=str(e))
                with self._lock:
                    self._current_task_enum = new_task
                self.get_logger().info(
                    f"Updated Heartbeat.current_task -> {rx_common_pb2.RxTask.Name(new_task)}"
                )
            elif param.name == "state_override":
                val = str(param.value).strip().lower()
                if val not in ("", "auto", "manual", "killed"):
                    return SetParametersResult(
                        successful=False,
                        reason="state_override must be '', 'auto', 'manual', or 'killed'",
                    )
                with self._lock:
                    self._state_override = val
                self.get_logger().info(f"Updated state_override -> '{val}'")
        return SetParametersResult(successful=True)

    def _on_shared_declaration_seq(self, msg: UInt32) -> None:
        seq = int(msg.data)
        if seq <= 0:
            return
        with self._lock:
            if self._active_declaration_seq != seq:
                self._active_declaration_seq = seq
                self._run_started = False
                self._run_id = None
                self.get_logger().info(
                    f"Synchronized active_declaration_seq={seq} from shared OCS topic"
                )

    def _on_state_override_msg(self, msg: String) -> None:
        val = msg.data.strip().lower()
        if val not in ("", "auto", "manual", "killed"):
            self.get_logger().error(
                f"Rejected ~/state_override '{msg.data}': use '', 'auto', 'manual', or 'killed'"
            )
            return
        with self._lock:
            self._state_override = val
        self.get_logger().info(f"Updated state_override via topic -> '{val}'")

    def _on_set_task_msg(self, msg: String) -> None:
        try:
            new_task = parse_task_param(msg.data)
        except ValueError as e:
            self.get_logger().error(f"Rejected ~/set_task '{msg.data}': {e}")
            return
        with self._lock:
            self._current_task_enum = new_task
        self.get_logger().info(
            f"Updated Heartbeat.current_task via topic -> {rx_common_pb2.RxTask.Name(new_task)}"
        )

    def _on_kill_override(self, msg: Bool) -> None:
        with self._lock:
            self._telemetry.kill_switch_engaged = bool(msg.data)
        self.get_logger().warn(
            f"Kill switch override set to {self._telemetry.kill_switch_engaged}"
        )

    def _on_depth_topic(self, msg: Float64) -> None:
        if math.isnan(msg.data):
            return
        with self._lock:
            if self._depth_source in ("auto", "topic"):
                self._telemetry.depth_m = max(0.0, float(msg.data))
                self._last_topic_depth_time = time.monotonic()
                self._last_depth_time = self._last_topic_depth_time

    # ── MAVROS2 Callbacks ────────────────────────────────────────────────────

    def _on_mavros_state(self, msg: State) -> None:
        with self._lock:
            self._telemetry.connected = bool(msg.connected)
            self._telemetry.armed = bool(msg.armed)
            self._telemetry.mode = str(msg.mode)
            self._telemetry.system_status = int(msg.system_status)
            self._last_state_time = time.monotonic()

    def _on_mavros_extended_state(self, msg: ExtendedState) -> None:
        with self._lock:
            self._telemetry.landed_state = int(msg.landed_state)

    def _on_mavros_global_pos(self, msg: NavSatFix) -> None:
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            return
        with self._lock:
            self._telemetry.latitude = float(msg.latitude)
            self._telemetry.longitude = float(msg.longitude)
            if not math.isnan(msg.altitude):
                # NavSatFix.altitude is height above the WGS-84 ellipsoid (HAE) in meters
                self._telemetry.altitude_hae_m = float(msg.altitude)

    def _on_mavros_compass_hdg(self, msg: Float64) -> None:
        if math.isnan(msg.data):
            return
        with self._lock:
            self._telemetry.heading_deg = float(msg.data) % 360.0

    def _on_mavros_rel_alt(self, msg: Float64) -> None:
        if math.isnan(msg.data):
            return
        with self._lock:
            if self._depth_source == "rel_alt" or (
                self._depth_source == "auto"
                and (time.monotonic() - self._last_topic_depth_time > 2.0)
            ):
                self._telemetry.depth_m = max(0.0, -float(msg.data))
                self._last_depth_time = time.monotonic()

    def _on_mavros_vfr_hud(self, msg: VFR_HUD) -> None:
        if math.isnan(msg.altitude):
            return
        with self._lock:
            if self._depth_source == "vfr_hud":
                self._telemetry.depth_m = max(0.0, -float(msg.altitude))
                self._last_depth_time = time.monotonic()

    def _on_mavros_local_position(self, msg: PoseStamped) -> None:
        point = msg.pose.position
        if not all(math.isfinite(value) for value in (point.x, point.y, point.z)):
            return
        with self._lock:
            self._last_local_position_time = time.monotonic()

    def _on_mavros_velocity(self, msg: TwistStamped) -> None:
        vx = msg.twist.linear.x
        vy = msg.twist.linear.y
        vz = msg.twist.linear.z
        spd = math.sqrt(vx * vx + vy * vy + vz * vz)
        if math.isnan(spd):
            return
        with self._lock:
            self._telemetry.spd_mps = float(spd)

    def _on_mavros_imu(self, msg: Imu) -> None:
        q = msg.orientation
        if q.x == 0.0 and q.y == 0.0 and q.z == 0.0 and q.w == 0.0:
            return
        roll_deg, pitch_deg = quaternion_to_roll_pitch_deg(q.x, q.y, q.z, q.w)
        with self._lock:
            self._telemetry.roll_deg = float(roll_deg)
            self._telemetry.pitch_deg = float(pitch_deg)

    # ── MQTT Connection & Rate-Limited Publishing ────────────────────────────

    def _start_mqtt_nonblocking(self) -> None:
        try:
            self._mqtt.connect_async(
                self._broker_host, self._broker_port, keepalive=15
            )
            self._mqtt.loop_start()
        except Exception as e:
            self.get_logger().error(f"Failed to start async MQTT client: {e}")

    def _on_mqtt_connect(
        self, client: mqtt.Client, userdata, flags, reason_code, properties
    ) -> None:
        if reason_code == 0:
            with self._lock:
                self._mqtt_connected = True
            self.get_logger().info(
                f"Connected to RoboCommand MQTT broker at {self._broker_host}:{self._broker_port}. "
                f"Subscribing to '{self._topic_course}' and '{self._topic_command}'."
            )
            client.subscribe(self._topic_course)
            client.subscribe(self._topic_command)
        else:
            self.get_logger().error(
                f"MQTT connect failed with reason_code={reason_code}"
            )

    def _on_mqtt_disconnect(
        self, client: mqtt.Client, userdata, disconnect_flags, reason_code, properties
    ) -> None:
        with self._lock:
            self._mqtt_connected = False
        self.get_logger().warn(
            f"Disconnected from RoboCommand MQTT broker (reason_code={reason_code}). "
            "Automatic reconnection active."
        )

    def _mqtt_publish_rate_limited(self, topic: str, payload: bytes) -> bool:
        """Publish to MQTT while enforcing the <= 5 msg/sec rate limit (§3.4.5)."""
        now = time.monotonic()
        with self._lock:
            if not self._mqtt_connected:
                return False
            while (
                self._publish_timestamps
                and now - self._publish_timestamps[0] >= 1.0
            ):
                self._publish_timestamps.popleft()
            if len(self._publish_timestamps) >= MAX_PUBLISH_PER_SEC:
                self.get_logger().warn(
                    f"Rate limit ({MAX_PUBLISH_PER_SEC} msg/s) reached; dropping publish to {topic}"
                )
                return False
            self._publish_timestamps.append(now)

        info = self._mqtt.publish(topic, payload)
        return info.rc == mqtt.MQTT_ERR_SUCCESS

    def _publish_vehicle_report(self, **body_kwargs) -> Tuple[bool, int]:
        """Serialize and publish an `RxReport` with an incremented per-vehicle sequence number."""
        with self._lock:
            self._report_seq += 1
            seq = self._report_seq

        report = rx_reports_pb2.RxReport(
            team_id=self._team_id,
            vehicle_id=self._vehicle_id,
            seq=seq,
            sent_at=now_proto_timestamp(),
            **body_kwargs,
        )
        ok = self._mqtt_publish_rate_limited(
            self._topic_report, report.SerializeToString()
        )
        return ok, seq

    # ── Incoming RoboCommand Message Handling ────────────────────────────────

    def _on_mqtt_message(
        self, client: mqtt.Client, userdata, msg: mqtt.MQTTMessage
    ) -> None:
        try:
            if msg.topic == self._topic_course:
                self._handle_rx_course(msg.payload)
            elif msg.topic == self._topic_command:
                self._handle_rx_command(msg.payload)
            else:
                self.get_logger().warn(
                    f"Received message on unexpected topic: {msg.topic}"
                )
        except Exception as e:
            # §3.4.13: Reject malformed or unexpected protobuf payloads without stopping
            self.get_logger().error(
                f"Failed to parse or process MQTT payload on '{msg.topic}': {e}"
            )

    def _handle_rx_course(self, payload: bytes) -> None:
        course = rx_course_pb2.RxCourse()
        course.ParseFromString(payload)
        if not course.course_id or len(course.corners) < 4:
            raise ValueError(
                f"Invalid RxCourse received: course_id='{course.course_id}', "
                f"corners={len(course.corners)}"
            )

        corners = [(p.latitude, p.longitude) for p in course.corners]
        transit = [(p.latitude, p.longitude) for p in course.uav_transit_area]
        if corners[0] != corners[-1]:
            self.get_logger().warn(
                "RxCourse.corners first and last points did not match; closing ring locally."
            )
            corners = ensure_closed_ring(corners)

        with self._lock:
            course_changed = self._course_id != course.course_id
            self._course_id = course.course_id
            self._pinger_freq_hz = int(course.pinger_freq_hz)
            self._course_corners = corners
            self._transit_corners = transit
            if course_changed:
                self._declaration_sent_for_course = False

        self.get_logger().info(
            f"[RxCourse] Validated course_id='{course.course_id}', "
            f"pinger={course.pinger_freq_hz} Hz, "
            f"corners={len(corners)} pts, transit_area={len(transit)} pts"
        )

        # Publish course info and acoustic pinger frequency to ROS 2
        course_json = {
            "course_id": course.course_id,
            "pinger_freq_hz": int(course.pinger_freq_hz),
            "corners": [{"latitude": lat, "longitude": lng} for lat, lng in corners],
            "uav_transit_area": [
                {"latitude": lat, "longitude": lng} for lat, lng in transit
            ],
        }
        self._course_pub.publish(String(data=json.dumps(course_json)))
        self._pinger_pub.publish(UInt32(data=int(course.pinger_freq_hz)))

        if bool(self.get_parameter("send_declaration_role").value) and bool(
            self.get_parameter("auto_send_declaration_on_course").value
        ):
            with self._lock:
                already_sent = self._declaration_sent_for_course
            if not already_sent:
                ok, detail = self.send_run_declaration()
                if ok:
                    with self._lock:
                        self._declaration_sent_for_course = True
                else:
                    self.get_logger().error(
                        f"Auto RunDeclaration failed: {detail}"
                    )

    def _is_command_for_this_vehicle(
        self, cmd_vehicle_id: str, target_vehicle_types: List[int]
    ) -> bool:
        if cmd_vehicle_id and cmd_vehicle_id != self._vehicle_id:
            return False
        if not target_vehicle_types:
            return True
        if rx_common_pb2.TYPE_UNKNOWN in target_vehicle_types:
            return True
        return self._vehicle_type_enum in target_vehicle_types

    def _handle_rx_command(self, payload: bytes) -> None:
        cmd = rx_commands_pb2.RxCommand()
        cmd.ParseFromString(payload)

        # §3.4.13: Confirm that topic team identifier matches decoded message
        if cmd.team_id != self._team_id:
            self.get_logger().error(
                f"Rejected RxCommand: message team_id '{cmd.team_id}' != '{self._team_id}'"
            )
            return

        body_type = cmd.WhichOneof("body")
        self.get_logger().info(
            f"[RxCommand] seq={cmd.seq} body={body_type} vehicle_id='{cmd.vehicle_id}'"
        )

        if body_type == "run_start":
            self._handle_run_start(cmd.seq, cmd.run_start)

        elif body_type == "assistance_request":
            ar = cmd.assistance_request
            if self._is_command_for_this_vehicle(cmd.vehicle_id, [ar.vehicle_type]):
                with self._lock:
                    self._last_command_seq = int(cmd.seq)
                    self._last_command_type = "assistance_request"
                msg_dict = {
                    "command_seq": int(cmd.seq),
                    "vehicle_id": cmd.vehicle_id,
                    "vehicle_type": rx_common_pb2.VehicleType.Name(ar.vehicle_type),
                    "position": {
                        "latitude": ar.position.latitude,
                        "longitude": ar.position.longitude,
                    },
                }
                self._cmd_assistance_pub.publish(String(data=json.dumps(msg_dict)))
                if bool(self.get_parameter("auto_ack_incidents").value):
                    self.send_incident_ack(int(cmd.seq))

        elif body_type == "keep_out_zone":
            koz = cmd.keep_out_zone
            if self._is_command_for_this_vehicle(cmd.vehicle_id, [koz.vehicle_type]):
                with self._lock:
                    self._last_command_seq = int(cmd.seq)
                    self._last_command_type = "keep_out_zone"
                msg_dict = {
                    "command_seq": int(cmd.seq),
                    "vehicle_id": cmd.vehicle_id,
                    "vehicle_type": rx_common_pb2.VehicleType.Name(koz.vehicle_type),
                    "center": {
                        "latitude": koz.center.latitude,
                        "longitude": koz.center.longitude,
                    },
                    "radius_m": float(koz.radius_m),
                }
                self._cmd_keep_out_pub.publish(String(data=json.dumps(msg_dict)))
                if bool(self.get_parameter("auto_ack_incidents").value):
                    self.send_incident_ack(int(cmd.seq))

        elif body_type == "all_clear":
            ac = cmd.all_clear
            if self._is_command_for_this_vehicle(cmd.vehicle_id, [ac.vehicle_type]):
                with self._lock:
                    self._last_command_seq = int(cmd.seq)
                    self._last_command_type = "all_clear"
                msg_dict = {
                    "command_seq": int(cmd.seq),
                    "vehicle_id": cmd.vehicle_id,
                    "vehicle_type": rx_common_pb2.VehicleType.Name(ac.vehicle_type),
                }
                self._cmd_all_clear_pub.publish(String(data=json.dumps(msg_dict)))
                if bool(self.get_parameter("auto_ack_incidents").value):
                    self.send_incident_ack(int(cmd.seq))

        elif body_type == "moving_object_alert":
            moa = cmd.moving_object_alert
            types = list(moa.affected_vehicle_types)
            if self._is_command_for_this_vehicle(cmd.vehicle_id, types):
                with self._lock:
                    self._last_command_seq = int(cmd.seq)
                    self._last_command_type = "moving_object_alert"
                msg_dict = {
                    "command_seq": int(cmd.seq),
                    "vehicle_id": cmd.vehicle_id,
                    "position": {
                        "latitude": moa.position.latitude,
                        "longitude": moa.position.longitude,
                    },
                    "heading_deg": float(moa.heading_deg),
                    "speed_mps": float(moa.speed_mps),
                    "affected_vehicle_types": [
                        rx_common_pb2.VehicleType.Name(t) for t in types
                    ],
                    "keep_out_radius_m": float(moa.keep_out_radius_m),
                }
                self._cmd_moving_obj_pub.publish(String(data=json.dumps(msg_dict)))

        elif body_type == "readiness_confirm":
            rc = cmd.readiness_confirm
            if not rc.vehicle_id or rc.vehicle_id == self._vehicle_id:
                with self._lock:
                    self._last_command_seq = int(cmd.seq)
                    self._last_command_type = "readiness_confirm"
                    self._last_readiness_confirmed_seq = int(rc.report_seq)
                msg_dict = {
                    "command_seq": int(cmd.seq),
                    "report_seq": int(rc.report_seq),
                    "vehicle_id": rc.vehicle_id,
                }
                self._cmd_readiness_confirm_pub.publish(
                    String(data=json.dumps(msg_dict))
                )
                self.get_logger().info(
                    f"[Task 4] ReadinessConfirm received for report_seq={rc.report_seq} "
                    f"vehicle_id='{rc.vehicle_id}'"
                )

    def _handle_run_start(
        self, command_seq: int, run_start: rx_commands_pb2.RunStart
    ) -> None:
        with self._lock:
            expected_decl_seq = self._active_declaration_seq
            send_role = bool(self.get_parameter("send_declaration_role").value)

        if expected_decl_seq is None:
            if send_role:
                self.get_logger().warn(
                    f"Ignored RunStart (run_id={run_start.run_id}, "
                    f"declaration_seq={run_start.declaration_seq}): no RunDeclaration sent yet."
                )
                return
            # Secondary vehicle node that hasn't yet received the shared seq accepts the positive declaration_seq
            if run_start.declaration_seq <= 0:
                return
            expected_decl_seq = int(run_start.declaration_seq)

        if run_start.declaration_seq != expected_decl_seq:
            self.get_logger().warn(
                f"Rejected RunStart: declaration_seq={run_start.declaration_seq} "
                f"does not match our active RunDeclaration seq={expected_decl_seq}."
            )
            return

        with self._lock:
            self._active_declaration_seq = expected_decl_seq
            self._run_started = True
            self._run_id = int(run_start.run_id)

        self.get_logger().info(
            f"[RunStart VERIFIED] declaration_seq={run_start.declaration_seq} "
            f"run_id={run_start.run_id} (command_seq={command_seq}). Official run started!"
        )

        self._run_started_pub.publish(Bool(data=True))
        self._run_id_pub.publish(UInt32(data=int(run_start.run_id)))

        # Optionally update current_task when the run starts
        initial_task_str = str(
            self.get_parameter("initial_task_on_run_start").value
        ).strip()
        if initial_task_str and initial_task_str.lower() != "none":
            try:
                task_val = parse_task_param(initial_task_str)
                with self._lock:
                    self._current_task_enum = task_val
                self.get_logger().info(
                    f"RunStart auto-set Heartbeat.current_task -> "
                    f"{rx_common_pb2.RxTask.Name(task_val)}"
                )
            except ValueError as e:
                self.get_logger().error(f"Invalid initial_task_on_run_start: {e}")

        trigger_mission = bool(
            self.get_parameter("auto_trigger_mission_on_run_start").value
        ) and bool(
            self.get_parameter("auto_trigger_px4_mission_on_run_start").value
        )
        if trigger_mission:
            self._trigger_mission_start()

    # ── Outgoing Task 1-4 Report Publishers (§3.4.12) ────────────────────────

    def send_safe_passage_report(
        self,
        entry_lat: float,
        entry_lng: float,
        exit_lat: float,
        exit_lng: float,
        buoys: List[Tuple[float, float, int]],
    ) -> Tuple[bool, int]:
        pb_buoys = [
            rx_reports_pb2.BuoyDetection(
                position=common_pb2.LatLng(latitude=float(lat), longitude=float(lng)),
                state=int(state),
            )
            for lat, lng, state in buoys
        ]
        ok, seq = self._publish_vehicle_report(
            safe_passage=rx_reports_pb2.SafePassageReport(
                entry_position=common_pb2.LatLng(
                    latitude=float(entry_lat), longitude=float(entry_lng)
                ),
                exit_position=common_pb2.LatLng(
                    latitude=float(exit_lat), longitude=float(exit_lng)
                ),
                buoys=pb_buoys,
            )
        )
        if ok:
            self.get_logger().info(
                f"[Task 1] Published SafePassageReport (seq={seq}, buoys={len(pb_buoys)})"
            )
        return ok, seq

    def _on_report_safe_passage(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            entry = data["entry_position"]
            exit_p = data["exit_position"]
            buoys = []
            for b in data.get("buoys", []):
                pos = b.get("position", b)
                buoys.append(
                    (
                        float(pos["latitude"]),
                        float(pos["longitude"]),
                        parse_beacon_state(b["state"]),
                    )
                )
            self.send_safe_passage_report(
                float(entry["latitude"]),
                float(entry["longitude"]),
                float(exit_p["latitude"]),
                float(exit_p["longitude"]),
                buoys,
            )
        except Exception as e:
            self.get_logger().error(f"Failed to send SafePassageReport: {e}")

    def send_pipeline_survey_report(
        self,
        active_buoy_lat: float,
        active_buoy_lng: float,
        segments: List[int],
    ) -> Tuple[bool, int]:
        ok, seq = self._publish_vehicle_report(
            pipeline_survey=rx_reports_pb2.PipelineSurveyReport(
                active_buoy_position=common_pb2.LatLng(
                    latitude=float(active_buoy_lat), longitude=float(active_buoy_lng)
                ),
                segments=[int(s) for s in segments],
            )
        )
        if ok:
            self.get_logger().info(
                f"[Task 2] Published PipelineSurveyReport (seq={seq}, segments={len(segments)})"
            )
        return ok, seq

    def _on_report_pipeline_survey(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            ab = data["active_buoy_position"]
            segs = [parse_pipeline_segment(s) for s in data["segments"]]
            self.send_pipeline_survey_report(
                float(ab["latitude"]), float(ab["longitude"]), segs
            )
        except Exception as e:
            self.get_logger().error(f"Failed to send PipelineSurveyReport: {e}")

    def send_resource_delivery_request(
        self, task_enum: int, resource_color_enum: int, circle_color_enum: int
    ) -> Tuple[bool, int]:
        ok, seq = self._publish_vehicle_report(
            resource_delivery=rx_reports_pb2.ResourceDeliveryRequest(
                task=int(task_enum),
                resource_color=int(resource_color_enum),
                delivery_circle_color=int(circle_color_enum),
            )
        )
        if ok:
            self.get_logger().info(
                f"[Task 2/3] Published ResourceDeliveryRequest (seq={seq})"
            )
        return ok, seq

    def _on_report_resource_delivery(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            task_enum = parse_task_param(str(data["task"]))
            res_color = parse_color(data["resource_color"])
            cir_color = parse_color(data["delivery_circle_color"])
            self.send_resource_delivery_request(task_enum, res_color, cir_color)
        except Exception as e:
            self.get_logger().error(f"Failed to send ResourceDeliveryRequest: {e}")

    def send_docking_report(self, bay_id: int) -> Tuple[bool, int]:
        ok, seq = self._publish_vehicle_report(
            docking=rx_reports_pb2.DockingReport(bay_id=int(bay_id))
        )
        if ok:
            self.get_logger().info(
                f"[Task 3] Published DockingReport (seq={seq}, bay_id={bay_id})"
            )
        return ok, seq

    def _on_report_docking(self, msg: UInt32) -> None:
        self.send_docking_report(int(msg.data))

    def send_firefighting_report(self, window_id: int) -> Tuple[bool, int]:
        ok, seq = self._publish_vehicle_report(
            firefighting=rx_reports_pb2.FirefightingReport(window_id=int(window_id))
        )
        if ok:
            self.get_logger().info(
                f"[Task 3] Published FirefightingReport (seq={seq}, window_id={window_id})"
            )
        return ok, seq

    def _on_report_firefighting(self, msg: UInt32) -> None:
        self.send_firefighting_report(int(msg.data))

    def send_incident_ack(self, command_seq: int) -> Tuple[bool, int]:
        ok, seq = self._publish_vehicle_report(
            incident_ack=rx_reports_pb2.IncidentAck(command_seq=int(command_seq))
        )
        if ok:
            self.get_logger().info(
                f"[Task 4] Published IncidentAck (command_seq={command_seq}, report_seq={seq})"
            )
        return ok, seq

    def _on_report_incident_ack(self, msg: UInt32) -> None:
        self.send_incident_ack(int(msg.data))

    def send_readiness_report(self, command_seq: int) -> Tuple[bool, int]:
        ok, seq = self._publish_vehicle_report(
            readiness=rx_reports_pb2.ReadinessReport(command_seq=int(command_seq))
        )
        if ok:
            self.get_logger().info(
                f"[Task 4] Published ReadinessReport (command_seq={command_seq}, report_seq={seq})"
            )
        return ok, seq

    def _on_report_readiness(self, msg: UInt32) -> None:
        self.send_readiness_report(int(msg.data))

    # ── Pre-Run Declaration & Geofence Resolution (§3.4.10) ──────────────────

    def _has_participating_uav(self, vehicles: List[str]) -> bool:
        if self._vehicle_type_enum == rx_common_pb2.TYPE_UAV:
            return True
        return any(v.upper().startswith("UAV") for v in vehicles)

    def _resolve_uav_geofence(self) -> List[LatLngTuple]:
        plan_file = str(self.get_parameter("geofence_plan_file").value).strip()
        if plan_file:
            fence = load_geofence_from_qgc_plan(plan_file)
            self.get_logger().info(
                f"Loaded UAV geofence ({len(fence)} pts) from QGC plan file: {plan_file}"
            )
            return fence

        lats = list(self.get_parameter("geofence_lats").value)
        lngs = list(self.get_parameter("geofence_lngs").value)
        if (
            len(lats) >= 3
            and len(lats) == len(lngs)
            and any(lat != 0.0 for lat in lats)
        ):
            fence = ensure_closed_ring(list(zip(lats, lngs)))
            self.get_logger().info(
                f"Loaded UAV geofence ({len(fence)} pts) from YAML geofence_lats/lngs"
            )
            return fence

        if bool(self.get_parameter("auto_geofence_from_course").value):
            with self._lock:
                course_corners = list(self._course_corners)
                transit_corners = list(self._transit_corners)
            if len(course_corners) < 4:
                raise ValueError(
                    "Cannot derive UAV geofence from course: RxCourse has not been received yet."
                )
            include_transit = bool(
                self.get_parameter("include_uav_transit_area").value
            )
            inset_m = float(self.get_parameter("geofence_inset_m").value)
            fence = derive_geofence_from_course(
                course_corners=course_corners,
                transit_corners=transit_corners,
                include_transit=include_transit,
                inset_m=inset_m,
            )
            self.get_logger().info(
                f"Auto-derived closed UAV geofence ({len(fence)} pts, "
                f"include_transit={include_transit}, inset={inset_m} m) from RxCourse"
            )
            return fence

        raise ValueError(
            "No UAV geofence source configured. Enable auto_geofence_from_course, "
            "set geofence_plan_file, or provide geofence_lats/geofence_lngs."
        )

    def send_run_declaration(self) -> Tuple[bool, str]:
        """Build, validate, and publish `RxRequest(run_declaration=...)`."""
        vehicles = [
            str(v).strip()
            for v in self.get_parameter("All_participating_vehicle_ids").value
            if str(v).strip()
        ]
        if self._vehicle_id not in vehicles:
            vehicles.append(self._vehicle_id)
        self._participating_vehicles = vehicles

        has_uav = self._has_participating_uav(vehicles)
        fence: List[LatLngTuple] = []
        max_alt_hae = 0.0
        callsign = ""

        if has_uav:
            try:
                fence = self._resolve_uav_geofence()
            except Exception as e:
                return False, str(e)

            with self._lock:
                course_corners = list(self._course_corners)
                transit_corners = list(self._transit_corners)

            max_alt_hae = float(
                self.get_parameter("uav_geofence_max_altitude_hae_m").value
            )
            problems = validate_uav_geofence(
                geofence=fence,
                max_altitude_hae_m=max_alt_hae,
                course_corners=course_corners,
                transit_corners=transit_corners,
            )
            if problems:
                err_msg = (
                    "Local RoboCommand geofence validation failed (RoboCommand would refuse "
                    "this declaration): " + "; ".join(problems)
                )
                self.get_logger().error(err_msg)
                return False, err_msg

            callsign = str(self.get_parameter("uav_pilot_callsign").value).strip()

        try:
            t1 = parse_tier_param(str(self.get_parameter("task1_tier").value))
            t2 = parse_tier_param(str(self.get_parameter("task2_tier").value))
            t3 = parse_tier_param(str(self.get_parameter("task3_tier").value))
            t4 = parse_tier_param(str(self.get_parameter("task4_tier").value))
        except ValueError as e:
            return False, str(e)

        with self._lock:
            self._request_seq += 1
            seq = self._request_seq
            # Reset run_started state for the new declaration (§3.4.13)
            self._active_declaration_seq = seq
            self._run_started = False
            self._run_id = None

        self._run_started_pub.publish(Bool(data=False))
        self._shared_decl_seq_pub.publish(UInt32(data=seq))

        req = rx_requests_pb2.RxRequest(
            team_id=self._team_id,
            seq=seq,
            sent_at=now_proto_timestamp(),
            run_declaration=rx_requests_pb2.RunDeclaration(
                vehicle_ids=vehicles,
                task1_tier=t1,
                task2_tier=t2,
                task3_tier=t3,
                task4_tier=t4,
                uav_geofence=[
                    common_pb2.LatLng(latitude=lat, longitude=lng)
                    for lat, lng in fence
                ],
                uav_geofence_max_altitude_hae_m=max_alt_hae,
                uav_pilot_callsign=callsign,
            ),
        )

        published = self._mqtt_publish_rate_limited(
            self._topic_request, req.SerializeToString()
        )
        if not published:
            return False, "MQTT client not connected or rate-limited"

        msg = (
            f"Published RunDeclaration (seq={seq}, vehicles={vehicles}, "
            f"geofence_pts={len(fence)}, max_alt_hae={max_alt_hae}m, callsign='{callsign}') "
            f"-> {self._topic_request}"
        )
        self.get_logger().info(msg)
        return True, msg

    def _on_send_declaration_srv(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        ok, message = self.send_run_declaration()
        response.success = ok
        response.message = message
        return response

    # ── 2 Hz Heartbeat Publication (§3.4.11) ─────────────────────────────────

    def _resolve_robot_state(
        self, snap: VehicleTelemetrySnapshot, state_override: str
    ) -> int:
        if state_override == "auto":
            return common_pb2.STATE_AUTO
        if state_override == "manual":
            return common_pb2.STATE_MANUAL
        if state_override == "killed":
            return common_pb2.STATE_KILLED
        return self._map_telemetry_to_robot_state(snap)

    def _build_heartbeat_message(
        self,
        snap: VehicleTelemetrySnapshot,
        robot_state: int,
        current_task: int,
    ) -> rx_reports_pb2.Heartbeat:
        hb_kwargs: Dict[str, Any] = {
            "state": robot_state,
            "position": common_pb2.LatLng(
                latitude=snap.latitude, longitude=snap.longitude
            ),
            "spd_mps": snap.spd_mps,
            "heading_deg": snap.heading_deg,
            "roll_deg": snap.roll_deg,
            "pitch_deg": snap.pitch_deg,
            "current_task": current_task,
            "vehicle_type": self._vehicle_type_enum,
        }

        if self._vehicle_type_enum == rx_common_pb2.TYPE_UAV:
            hb_kwargs["altitude_hae_m"] = snap.altitude_hae_m
            hb_kwargs["flight_phase"] = map_px4_to_flight_phase(snap)
            hb_kwargs["uav_registration"] = self._uav_registration
        elif self._vehicle_type_enum == rx_common_pb2.TYPE_UUV:
            hb_kwargs["depth_m"] = snap.depth_m
            hb_kwargs["flight_phase"] = rx_common_pb2.FLIGHT_PHASE_UNKNOWN
        else:
            # TYPE_USV
            hb_kwargs["flight_phase"] = rx_common_pb2.FLIGHT_PHASE_UNKNOWN

        return rx_reports_pb2.Heartbeat(**hb_kwargs)

    def _publish_heartbeat(self) -> None:
        with self._lock:
            if not self._mqtt_connected:
                return
            snap = VehicleTelemetrySnapshot(**self._telemetry.__dict__)
            current_task = self._current_task_enum
            state_override = self._state_override

        robot_state = self._resolve_robot_state(snap, state_override)
        hb = self._build_heartbeat_message(snap, robot_state, current_task)
        self._publish_vehicle_report(heartbeat=hb)

    # ── MAVROS2 Mission Start Trigger on RunStart ────────────────────────────

    def _resolve_run_start_target_mode(self) -> str:
        mode = str(self.get_parameter("run_start_mode").value).strip()
        px4_mode = str(self.get_parameter("px4_run_start_mode").value).strip()
        if self._vehicle_type_enum == rx_common_pb2.TYPE_UAV and px4_mode:
            return px4_mode
        return mode or px4_mode

    def _trigger_mission_start(self) -> None:
        if not bool(self.get_parameter("enable_run_start_mode").value):
            self.get_logger().warn(
                "RunStart received; MAVROS mode change remains disabled until "
                "enable_run_start_mode:=true is set locally after preflight."
            )
            return

        blockers = self._mission_start_blockers()
        if blockers:
            self.get_logger().error(
                "Rejected RunStart MAVROS mode change: " + "; ".join(blockers)
            )
            return

        target_mode = self._resolve_run_start_target_mode()
        auto_arm = bool(self.get_parameter("auto_arm_on_run_start").value)

        if auto_arm:
            if self._arming_client.service_is_ready():
                arm_req = CommandBool.Request()
                arm_req.value = True
                future = self._arming_client.call_async(arm_req)
                future.add_done_callback(self._on_arm_done)
            else:
                self.get_logger().warn(
                    "MAVROS arming service not ready when RunStart arrived."
                )

        if target_mode:
            if self._set_mode_client.service_is_ready():
                mode_req = SetMode.Request()
                mode_req.custom_mode = target_mode
                future = self._set_mode_client.call_async(mode_req)
                future.add_done_callback(
                    lambda fut: self._on_set_mode_done(fut, target_mode)
                )
            else:
                self.get_logger().warn(
                    f"MAVROS set_mode service not ready; could not switch to '{target_mode}'."
                )

    def _mission_start_blockers(self) -> List[str]:
        """Return safety blockers before a RunStart can change a vehicle mode."""
        now = time.monotonic()
        timeout = max(
            0.1, float(self.get_parameter("mission_start_telemetry_timeout_s").value)
        )
        with self._lock:
            connected = self._telemetry.connected
            state_age = now - self._last_state_time
            local_position_age = now - self._last_local_position_time
            depth_age = now - self._last_depth_time

        blockers: List[str] = []
        if bool(self.get_parameter("mission_start_requires_telemetry").value):
            if not connected or state_age > timeout:
                blockers.append("MAVROS/FCU telemetry is disconnected or stale")
        if bool(self.get_parameter("mission_start_requires_local_position").value):
            if local_position_age > timeout:
                blockers.append("local position is unavailable or stale")
        if bool(self.get_parameter("mission_start_requires_depth").value):
            if depth_age > timeout:
                blockers.append("depth sample is unavailable or stale")
        return blockers

    def _on_arm_done(self, future) -> None:
        try:
            res = future.result()
            self.get_logger().info(
                f"MAVROS arming call completed: success={res.success}"
            )
        except Exception as e:
            self.get_logger().error(f"MAVROS arming service call failed: {e}")

    def _on_set_mode_done(self, future, target_mode: str) -> None:
        try:
            res = future.result()
            self.get_logger().info(
                f"MAVROS set_mode('{target_mode}') completed: mode_sent={res.mode_sent}"
            )
        except Exception as e:
            self.get_logger().error(
                f"MAVROS set_mode('{target_mode}') call failed: {e}"
            )

    # ── Operator Status Publication (§3.4.13) ────────────────────────────────

    def _publish_operator_status(self) -> None:
        with self._lock:
            snap = VehicleTelemetrySnapshot(**self._telemetry.__dict__)
            state_override = self._state_override
            flight_phase_name = (
                rx_common_pb2.FlightPhase.Name(map_px4_to_flight_phase(snap))
                if self._vehicle_type_enum == rx_common_pb2.TYPE_UAV
                else rx_common_pb2.FlightPhase.Name(
                    rx_common_pb2.FLIGHT_PHASE_UNKNOWN
                )
            )
            status_dict = {
                "mqtt_connected": self._mqtt_connected,
                "broker": f"{self._broker_host}:{self._broker_port}",
                "team_id": self._team_id,
                "vehicle_id": self._vehicle_id,
                "vehicle_type": rx_common_pb2.VehicleType.Name(
                    self._vehicle_type_enum
                ),
                "course_id": self._course_id,
                "pinger_freq_hz": self._pinger_freq_hz,
                "declaration_seq": self._active_declaration_seq,
                "run_started": self._run_started,
                "run_id": self._run_id,
                "report_seq": self._report_seq,
                "last_command_seq": self._last_command_seq,
                "last_command_type": self._last_command_type,
                "last_readiness_confirmed_seq": self._last_readiness_confirmed_seq,
                "mavros_connected": snap.connected,
                "autopilot_armed": snap.armed,
                "px4_armed": snap.armed,
                "autopilot_mode": snap.mode,
                "px4_mode": snap.mode,
                "state_override": state_override,
                "robot_state": common_pb2.RobotState.Name(
                    self._resolve_robot_state(snap, state_override)
                ),
                "flight_phase": flight_phase_name,
                "current_task": rx_common_pb2.RxTask.Name(self._current_task_enum),
                "lat": snap.latitude,
                "lng": snap.longitude,
                "altitude_hae_m": round(snap.altitude_hae_m, 2),
                "depth_m": round(snap.depth_m, 2),
                "spd_mps": round(snap.spd_mps, 2),
                "heading_deg": round(snap.heading_deg, 1),
            }

        self._status_pub.publish(String(data=json.dumps(status_dict)))

    def destroy_node(self) -> None:
        try:
            self._mqtt.loop_stop()
            self._mqtt.disconnect()
        except Exception:
            pass
        super().destroy_node()
