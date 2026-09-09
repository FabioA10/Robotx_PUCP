import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Vector3Stamped

from std_msgs.msg import Bool
from std_msgs.msg import Empty
from std_msgs.msg import Float32
from std_msgs.msg import String
from std_msgs.msg import UInt8MultiArray
from std_msgs.msg import Int8
from std_msgs.msg import UInt32

from pymavlink import mavutil


class MavlinkBridgeNode(Node):

    def __init__(self):
        super().__init__('uuv_mavlink_bridge')

        self.declare_parameter('enable_mode_control', False)
        self.enable_mode_control = bool(
            self.get_parameter('enable_mode_control').value
        )
        self.create_subscription(
            String,
            '/uuv/control/mode_request',
            self.mode_request_callback,
            10
        )

        # =========================================================
        # Parameters
        # =========================================================

        self.mode_name_by_id = {
            0: 'STABILIZE',
            1: 'ACRO',
            2: 'ALT_HOLD',
            3: 'AUTO',
            4: 'GUIDED',
            7: 'CIRCLE',
            9: 'SURFACE',
            16: 'POSHOLD',
            19: 'MANUAL',
            20: 'MOTOR_DETECT',
            21: 'SURFTRAK',
        }

        # =========================================================
        # Modes available from Xbox selector
        #
        # LB + D-pad UP   -> next mode
        # LB + D-pad DOWN -> previous mode
        #
        # MOTOR_DETECT is intentionally excluded.
        # =========================================================

        self.mode_sequence = [
            ('MANUAL', 19),
            ('STABILIZE', 0),
            ('ACRO', 1),
            ('ALT_HOLD', 2),
            ('POSHOLD', 16),
            ('SURFTRAK', 21),
            ('GUIDED', 4),
            ('AUTO', 3),
            ('CIRCLE', 7),
            ('SURFACE', 9),
        ]

        self.declare_parameter(
            'enable_arm_disarm',
            False
        )

        self.declare_parameter(
            'sys_status_rate_hz',
            20.0
        )

        self.sys_status_rate_hz = (
            self.get_parameter('sys_status_rate_hz')
            .get_parameter_value()
            .double_value
        )

        self.enable_arm_disarm = (
            self.get_parameter('enable_arm_disarm')
            .get_parameter_value()
            .bool_value
        )


        self.declare_parameter('udp_port', 14552)

        self.declare_parameter('target_system_id', 1)
        self.declare_parameter('target_component_id', 1)
        self.declare_parameter('source_system_id', 255)

        self.declare_parameter('heartbeat_timeout', 3.0)

        self.declare_parameter(
            'motor_power_on_threshold',
            10.0
        )

        self.declare_parameter(
            'motor_power_off_threshold',
            5.0
        )

        # ---------------------------------------------------------
        # Manual control safety parameters
        # ---------------------------------------------------------

        self.declare_parameter(
            'enable_command_output',
            False
        )

        self.declare_parameter(
            'command_timeout',
            0.30
        )

        # Software limit for the first physical tests.
        # 0.20 = maximum 20 % of MANUAL_CONTROL range.
        self.declare_parameter(
            'command_scale',
            0.20
        )

        self.declare_parameter(
            'require_manual_mode',
            True
        )

        self.declare_parameter('enable_accessory_output', False)
        self.declare_parameter('camera_tilt_channel', 8)
        self.declare_parameter('accessory_min_pwm', 1100)
        self.declare_parameter('accessory_neutral_pwm', 1500)
        self.declare_parameter('accessory_max_pwm', 1900)
        self.declare_parameter('lights_steps', 9)
        self.declare_parameter('lights_dimmer_button', 13)
        self.declare_parameter('lights_brighter_button', 14)

        # ---------------------------------------------------------
        # Read parameters
        # ---------------------------------------------------------

        self.source_system_id = (
            self.get_parameter('source_system_id')
            .get_parameter_value()
            .integer_value
        )

        self.udp_port = (
            self.get_parameter('udp_port')
            .get_parameter_value()
            .integer_value
        )

        self.target_system_id = (
            self.get_parameter('target_system_id')
            .get_parameter_value()
            .integer_value
        )

        self.target_component_id = (
            self.get_parameter('target_component_id')
            .get_parameter_value()
            .integer_value
        )

        self.heartbeat_timeout = (
            self.get_parameter('heartbeat_timeout')
            .get_parameter_value()
            .double_value
        )

        self.motor_power_on_threshold = (
            self.get_parameter('motor_power_on_threshold')
            .get_parameter_value()
            .double_value
        )

        self.motor_power_off_threshold = (
            self.get_parameter('motor_power_off_threshold')
            .get_parameter_value()
            .double_value
        )

        self.enable_command_output = (
            self.get_parameter('enable_command_output')
            .get_parameter_value()
            .bool_value
        )

        self.command_timeout = (
            self.get_parameter('command_timeout')
            .get_parameter_value()
            .double_value
        )

        self.command_scale = (
            self.get_parameter('command_scale')
            .get_parameter_value()
            .double_value
        )

        self.require_manual_mode = (
            self.get_parameter('require_manual_mode')
            .get_parameter_value()
            .bool_value
        )

        self.enable_accessory_output = bool(
            self.get_parameter('enable_accessory_output').value
        )
        self.camera_tilt_channel = int(
            self.get_parameter('camera_tilt_channel').value
        )
        self.accessory_min_pwm = int(
            self.get_parameter('accessory_min_pwm').value
        )
        self.accessory_neutral_pwm = int(
            self.get_parameter('accessory_neutral_pwm').value
        )
        self.accessory_max_pwm = int(
            self.get_parameter('accessory_max_pwm').value
        )
        self.lights_steps = max(
            1,
            int(self.get_parameter('lights_steps').value)
        )
        self.lights_dimmer_button = int(
            self.get_parameter('lights_dimmer_button').value
        )
        self.lights_brighter_button = int(
            self.get_parameter('lights_brighter_button').value
        )

        self.command_scale = max(
            0.0,
            min(1.0, self.command_scale)
        )

        # =========================================================
        # State
        # =========================================================

        self.last_armed_state = None

        self.last_heartbeat_time = None
        self.last_cmd_time = None

        self.connected = False
        self.armed = False
        self.mode = 'UNKNOWN'

        self.motors_enabled = False
        self.deadman = False

        self.last_cmd = Twist()

        self.last_connected_state = None
        self.last_motor_power_state = None
        self.last_motion_allowed = None

        self.lights_level = 0
        self.pending_manual_buttons = 0

        self.last_camera_command_time = None
        self.camera_timeout_timer = self.create_timer(
            0.1, self.check_camera_timeout
        )

        # =========================================================
        # Publishers
        # =========================================================

        self.connected_pub = self.create_publisher(
            Bool,
            '/uuv/connected',
            10
        )

        self.armed_pub = self.create_publisher(
            Bool,
            '/uuv/armed',
            10
        )

        self.mode_pub = self.create_publisher(
            String,
            '/uuv/mode',
            10
        )

        self.voltage_pub = self.create_publisher(
            Float32,
            '/uuv/power/voltage',
            10
        )

        self.current_pub = self.create_publisher(
            Float32,
            '/uuv/power/current',
            10
        )

        self.motors_enabled_pub = self.create_publisher(
            Bool,
            '/uuv/power/motors_enabled',
            10
        )

        self.command_output_pub = self.create_publisher(
            Bool,
            '/uuv/control/command_output_enabled',
            10
        )

        self.motion_allowed_pub = self.create_publisher(
            Bool,
            '/uuv/control/mavlink_motion_allowed',
            10
        )

        self.lights_percent_pub = self.create_publisher(
            Float32,
            '/uuv/accessories/lights_percent',
            10
        )

        # Telemetría MAVLink conservando explícitamente la convención NED.
        self.local_position_ned_pub = self.create_publisher(
            PointStamped, '/uuv/telemetry/local_position_ned', 10)
        self.local_velocity_ned_pub = self.create_publisher(
            Vector3Stamped, '/uuv/telemetry/local_velocity_ned', 10)
        self.attitude_rpy_pub = self.create_publisher(
            Vector3Stamped, '/uuv/telemetry/attitude_rpy', 10)
        self.heading_pub = self.create_publisher(
            Float32, '/uuv/telemetry/heading_deg', 10)
        self.relative_altitude_pub = self.create_publisher(
            Float32, '/uuv/telemetry/relative_altitude_m', 10)
        self.depth_estimate_pub = self.create_publisher(
            Float32, '/uuv/telemetry/depth_estimate_m', 10)
        self.pressure_abs_pub = self.create_publisher(
            Float32, '/uuv/telemetry/pressure_abs_hpa', 10)
        self.ekf_flags_pub = self.create_publisher(
            UInt32, '/uuv/telemetry/ekf_status_flags', 10)

        # =========================================================
        # Subscribers
        # =========================================================

        self.create_subscription(
            Int8,
            '/uuv/control/mode_step',
            self.mode_step_callback,
            10
        )

        self.create_subscription(
            UInt8MultiArray,
            '/uuv/indicator/rgb',
            self.rgb_indicator_callback,
            10
        )

        self.create_subscription(
            Empty,
            '/uuv/control/arm_request',
            self.arm_request_callback,
            10
        )

        self.create_subscription(
            Empty,
            '/uuv/control/disarm_request',
            self.disarm_request_callback,
            10
        )

        self.create_subscription(
            Twist,
            '/uuv/cmd_vel_manual',
            self.cmd_callback,
            10
        )

        self.create_subscription(
            Bool,
            '/uuv/deadman',
            self.deadman_callback,
            10
        )

        self.create_subscription(
            Float32,
            '/uuv/control/camera_tilt',
            self.camera_tilt_callback,
            10
        )

        self.create_subscription(
            Int8,
            '/uuv/control/lights_step',
            self.lights_step_callback,
            10
        )

        # =========================================================
        # MAVLink
        # =========================================================

        connection_string = (
            f'udpin:0.0.0.0:{self.udp_port}'
        )

        self.get_logger().info(
            f'Opening MAVLink connection on '
            f'{connection_string}'
        )

        self.master = mavutil.mavlink_connection(
            connection_string,
            source_system=self.source_system_id
        )

        # =========================================================
        # Timers
        # =========================================================

        # MAVLink receiver
        self.read_timer = self.create_timer(
            0.01,
            self.read_mavlink
        )

        # Connection watchdog
        self.status_timer = self.create_timer(
            0.2,
            self.update_connection_status
        )

        # MANUAL_CONTROL output at 20 Hz
        self.command_timer = self.create_timer(
            0.05,
            self.update_manual_control
        )

        # =========================================================
        # Startup information
        # =========================================================

        self.get_logger().info(
            f'Target autopilot: '
            f'SYSID={self.target_system_id}, '
            f'COMPID={self.target_component_id}'
        )

        self.get_logger().info(
            f'MAVLink source SYSID: {self.source_system_id}'
        )

        self.get_logger().info(
            f'Motor power thresholds: '
            f'OFF <= {self.motor_power_off_threshold:.1f} V | '
            f'ON >= {self.motor_power_on_threshold:.1f} V'
        )

        self.get_logger().info(
            f'Command timeout: '
            f'{self.command_timeout:.2f} s'
        )

        self.get_logger().info(
            f'Command scale: '
            f'{self.command_scale:.2f}'
        )

        if self.enable_command_output:

            self.get_logger().warning(
                'MAVLink command output ENABLED'
            )

        else:

            self.get_logger().info(
                'MAVLink command output DISABLED'
            )

            if not self.enable_accessory_output:
                self.get_logger().info(
                    'Telemetry remains READ-ONLY.'
                )

        if self.enable_accessory_output:
            self.get_logger().warning(
                'Camera tilt and light output ENABLED'
            )
        else:
            self.get_logger().info(
                'Camera tilt and light output DISABLED'
            )

    def mode_request_callback(self, msg):
        modes = {
            'MANUAL': 19,
            'ALT_HOLD': 2,
            'POSHOLD': 16,
        }
        requested = msg.data.strip().upper()

        if not self.enable_mode_control:
            self.get_logger().warning('Selector de modos deshabilitado')
            return

        if requested not in modes:
            self.get_logger().warning('Modo no permitido por el selector')
            return

        heartbeat_fresh = (
            self.last_heartbeat_time is not None
            and time.monotonic() - self.last_heartbeat_time
            < self.heartbeat_timeout
        )

        if not self.connected or not heartbeat_fresh:
            self.get_logger().warning('Cambio rechazado: sin conexión reciente')
            return

        if self.armed:
            self.get_logger().warning(
                'Cambio rechazado: esta prueba requiere ROV desarmado'
            )
            return

        if self.deadman:
            self.get_logger().warning('Cambio rechazado: suelta RB')
            return

        try:
            self.master.mav.set_mode_send(
                self.target_system_id,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                modes[requested]
            )
            self.get_logger().info(
                f'Modo solicitado: {requested}; esperando heartbeat'
            )
        except Exception as exc:
            self.get_logger().error(f'Error solicitando modo: {exc}')

    def mode_step_callback(self, msg):

        # ---------------------------------------------------------
        # Safety checks
        # ---------------------------------------------------------

        if not self.enable_command_output:
            self.get_logger().warning(
                'MODE change rejected: command output disabled'
            )
            return

        if not self.connected:
            self.get_logger().warning(
                'MODE change rejected: MAVLink disconnected'
            )
            return

        if not self.motors_enabled:
            self.get_logger().warning(
                'MODE change rejected: propulsion power disabled'
            )
            return

        if self.deadman:
            self.get_logger().warning(
                'MODE change rejected: release RB first'
            )
            return


        # ---------------------------------------------------------
        # Direction
        #
        # +1 = next mode
        # -1 = previous mode
        # ---------------------------------------------------------

        step = int(msg.data)

        if step not in (-1, 1):
            return


        # ---------------------------------------------------------
        # Current mode
        # ---------------------------------------------------------

        mode_names = [
            mode_name
            for mode_name, _ in self.mode_sequence
        ]

        if self.mode not in mode_names:

            self.get_logger().warning(
                f'Current mode {self.mode} '
                f'is not in mode selector'
            )
            return


        current_index = mode_names.index(
            self.mode
        )

        new_index = current_index + step


        # ---------------------------------------------------------
        # Do not wrap around
        # ---------------------------------------------------------

        if new_index < 0:

            self.get_logger().info(
                f'Already at first mode: {self.mode}'
            )
            return

        if new_index >= len(self.mode_sequence):

            self.get_logger().info(
                f'Already at last mode: {self.mode}'
            )
            return


        requested_mode, custom_mode = (
            self.mode_sequence[new_index]
        )


        # ---------------------------------------------------------
        # Send mode change to ArduSub
        # ---------------------------------------------------------

        self.get_logger().warning(
            f'MODE request: '
            f'{self.mode} -> {requested_mode}'
        )

        try:

            self.master.mav.set_mode_send(
                self.target_system_id,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                custom_mode
            )

        except Exception as exc:

            self.get_logger().error(
                f'Failed to send mode change: {exc}'
            )



    def arm_request_callback(self, msg):

        if not self.enable_arm_disarm:
            self.get_logger().warning(
                'ARM rejected: arm/disarm control disabled'
            )
            return

        if not self.connected:
            self.get_logger().warning(
                'ARM rejected: MAVLink disconnected'
            )
            return

        if not self.motors_enabled:
            self.get_logger().warning(
                'ARM rejected: propulsion power disabled'
            )
            return

        if self.armed:
            self.get_logger().info(
                'ARM ignored: vehicle already armed'
            )
            return

        if not self.deadman:
            self.get_logger().warning(
                'ARM rejected: RB deadman is not pressed'
            )
            return

        if (
            self.require_manual_mode
            and self.mode != 'MANUAL'
        ):
            self.get_logger().warning(
                f'ARM rejected: vehicle mode is {self.mode}'
            )
            return

        if not self.enable_command_output:
            self.get_logger().warning(
                'ARM rejected: command output disabled'
            )
            return

        self.get_logger().warning(
            'Sending MAVLink ARM command'
        )

        self.send_arm_disarm(True)


    def disarm_request_callback(self, msg):

        if not self.enable_arm_disarm:
            self.get_logger().warning(
                'DISARM rejected: arm/disarm control disabled'
            )
            return

        if not self.connected:
            self.get_logger().warning(
                'DISARM rejected: MAVLink disconnected'
            )
            return

        self.get_logger().warning(
            'Sending MAVLink DISARM command'
        )

        self.send_arm_disarm(False)


    def send_arm_disarm(self, arm):

        self.master.mav.command_long_send(
            self.target_system_id,
            self.target_component_id,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1.0 if arm else 0.0,
            0.0,  # NO force arm/disarm
            0.0,
            0.0,
            0.0,
            0.0,
            0.0
        )

    def request_sys_status_rate(self):

        if self.sys_status_rate_hz <= 0.0:
            return

        interval_us = int(
            1_000_000
            / self.sys_status_rate_hz
        )

        self.master.mav.command_long_send(
            self.target_system_id,
            self.target_component_id,

            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,

            0,

            mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS,
            interval_us,

            0,
            0,
            0,
            0,
            0
        )

        self.get_logger().info(
            f'Requested SYS_STATUS at '
            f'{self.sys_status_rate_hz:.1f} Hz'
        )

    # =============================================================
    # ROS callbacks
    # =============================================================

    def cmd_callback(self, msg):

        self.last_cmd = msg
        self.last_cmd_time = time.monotonic()

    def deadman_callback(self, msg):

        self.deadman = msg.data

    def camera_tilt_callback(self, msg):

        if not self.accessory_command_is_allowed():
            return

        self.last_camera_command_time = time.monotonic()
        command = self.clamp(float(msg.data), -1.0, 1.0)

        if command >= 0.0:
            pwm = int(
                self.accessory_neutral_pwm
                + command * (
                    self.accessory_max_pwm
                    - self.accessory_neutral_pwm
                )
            )
        else:
            pwm = int(
                self.accessory_neutral_pwm
                + command * (
                    self.accessory_neutral_pwm
                    - self.accessory_min_pwm
                )
            )

        self.send_rc_override(self.camera_tilt_channel, pwm)

    def check_camera_timeout(self):
        if self.last_camera_command_time is None:
            return

        elapsed = time.monotonic() - self.last_camera_command_time

        if elapsed <= 0.3:
            return

        if self.accessory_command_is_allowed():
            self.send_rc_override(
                self.camera_tilt_channel,
                self.accessory_neutral_pwm
            )

        self.last_camera_command_time = None


    def lights_step_callback(self, msg):

        if not self.accessory_command_is_allowed():
            return

        step = int(msg.data)
        if step not in (-1, 1):
            return

        new_level = max(
            0,
            min(
                self.lights_steps,
                self.lights_level + step
            )
        )

        if new_level == self.lights_level:
            return

        self.lights_level = new_level

        if step > 0:
            button = self.lights_brighter_button
        else:
            button = self.lights_dimmer_button

        self.pending_manual_buttons |= (1 << button)

        lights_percent = (
            100.0 * self.lights_level / self.lights_steps
        )

        state_msg = Float32()
        state_msg.data = float(lights_percent)
        self.lights_percent_pub.publish(state_msg)

        self.get_logger().info(
            f'Lights request: {lights_percent:.1f}% '
            f'(level {self.lights_level}/{self.lights_steps})'
        )

    def accessory_command_is_allowed(self):

        if not self.enable_accessory_output:
            return False

        if not self.connected:
            return False

        return True

    def send_rc_override(self, channel, pwm):

        if channel < 1 or channel > 18:
            self.get_logger().error(
                f'Invalid RC override channel: {channel}'
            )
            return

        channels = [65535] * 18
        channels[channel - 1] = int(pwm)

        try:
            self.master.mav.rc_channels_override_send(
                self.target_system_id,
                self.target_component_id,
                *channels
            )
        except Exception as exc:
            self.get_logger().error(
                f'Failed to send RC override: {exc}'
            )

    def rgb_indicator_callback(self, msg):

        if len(msg.data) != 3:

            self.get_logger().warning(
                'RGB command rejected: expected [R, G, B]'
            )
            return

        if not self.connected:

            self.get_logger().warning(
                'RGB command rejected: MAVLink disconnected'
            )
            return

        r = int(msg.data[0])
        g = int(msg.data[1])
        b = int(msg.data[2])

        self.send_rgb_indicator(
            r,
            g,
            b
        )

    # =============================================================
    # MAVLink receiver
    # =============================================================

    def read_mavlink(self):

        while True:

            msg = self.master.recv_match(
                blocking=False
            )

            if msg is None:
                break

            if (
                msg.get_srcSystem()
                != self.target_system_id
                or
                msg.get_srcComponent()
                != self.target_component_id
            ):
                continue

            msg_type = msg.get_type()

            if msg_type == 'HEARTBEAT':

                self.process_heartbeat(msg)

            elif msg_type == 'SYS_STATUS':

                self.process_sys_status(msg)

            elif msg_type == 'LOCAL_POSITION_NED':

                self.process_local_position_ned(msg)

            elif msg_type == 'ATTITUDE':

                self.process_attitude(msg)

            elif msg_type == 'GLOBAL_POSITION_INT':

                self.process_global_position(msg)

            elif msg_type == 'VFR_HUD':

                self.process_vfr_hud(msg)

            elif msg_type == 'SCALED_PRESSURE2':

                self.process_scaled_pressure2(msg)

            elif msg_type == 'EKF_STATUS_REPORT':

                self.process_ekf_status(msg)

            elif msg_type == 'STATUSTEXT':
                text = msg.text
                if isinstance(text, bytes):
                    text = text.decode('utf-8', errors='replace')

                self.get_logger().warning(
                    f'ArduSub [{msg.severity}]: {text}'
                )

            elif msg_type == 'COMMAND_ACK':
                self.get_logger().info(
                    f'ArduSub ACK: command={msg.command}, '
                    f'result={msg.result}'
                )

    def _telemetry_stamp(self):
        return self.get_clock().now().to_msg()

    def process_local_position_ned(self, msg):
        stamp = self._telemetry_stamp()
        position = PointStamped()
        position.header.stamp = stamp
        position.header.frame_id = 'mavlink_local_ned'
        position.point.x = float(msg.x)
        position.point.y = float(msg.y)
        position.point.z = float(msg.z)
        self.local_position_ned_pub.publish(position)

        velocity = Vector3Stamped()
        velocity.header.stamp = stamp
        velocity.header.frame_id = 'mavlink_local_ned'
        velocity.vector.x = float(msg.vx)
        velocity.vector.y = float(msg.vy)
        velocity.vector.z = float(msg.vz)
        self.local_velocity_ned_pub.publish(velocity)

    def process_attitude(self, msg):
        attitude = Vector3Stamped()
        attitude.header.stamp = self._telemetry_stamp()
        attitude.header.frame_id = 'mavlink_body_ned'
        attitude.vector.x = float(msg.roll)
        attitude.vector.y = float(msg.pitch)
        attitude.vector.z = float(msg.yaw)
        self.attitude_rpy_pub.publish(attitude)

    def process_global_position(self, msg):
        relative_altitude = Float32()
        relative_altitude.data = float(msg.relative_alt) / 1000.0
        self.relative_altitude_pub.publish(relative_altitude)
        depth = Float32()
        depth.data = max(0.0, -relative_altitude.data)
        self.depth_estimate_pub.publish(depth)
        if int(msg.hdg) != 65535:
            heading = Float32()
            heading.data = float(msg.hdg) / 100.0
            self.heading_pub.publish(heading)

    def process_vfr_hud(self, msg):
        heading = Float32()
        heading.data = float(msg.heading)
        self.heading_pub.publish(heading)

    def process_scaled_pressure2(self, msg):
        pressure = Float32()
        pressure.data = float(msg.press_abs)
        self.pressure_abs_pub.publish(pressure)

    def process_ekf_status(self, msg):
        flags = UInt32()
        flags.data = int(msg.flags)
        self.ekf_flags_pub.publish(flags)

    def send_rgb_indicator(self, r, g, b):

        custom_bytes = [
            r,
            g,
            b,
        ] + [0] * 21

        try:

            self.master.mav.led_control_send(
                self.target_system_id,
                self.target_component_id,

                255,   # all LED instances
                255,   # LED_CONTROL_PATTERN_CUSTOM

                3,     # custom length = RGB

                custom_bytes
            )

            self.get_logger().info(
                f'RGB indicator: '
                f'R={r} G={g} B={b}'
            )

        except Exception as exc:

            self.get_logger().error(
                f'Failed to send RGB indicator command: {exc}'
            )
    # =============================================================
    # HEARTBEAT
    # =============================================================

    def process_heartbeat(self, msg):

        self.last_heartbeat_time = time.monotonic()

        self.armed = bool(
            msg.base_mode
            & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        )

        if self.armed != self.last_armed_state:

            if self.armed:

                self.get_logger().warning(
                    '================================'
                )

                self.get_logger().warning(
                    '      BLUE ROV2 ARMED'
                )

                self.get_logger().warning(
                    '================================'
                )

            else:

                self.get_logger().info(
                    '================================'
                )

                self.get_logger().info(
                    '      BLUE ROV2 DISARMED'
                )

                self.get_logger().info(
                    '================================'
                )

            self.last_armed_state = self.armed


        # =========================================================
        # Resolve ArduSub mode from MAVLink custom_mode
        # =========================================================

        custom_mode = int(msg.custom_mode)

        self.mode = self.mode_name_by_id.get(
            custom_mode,
            mavutil.mode_string_v10(msg)
        )


        armed_msg = Bool()
        armed_msg.data = self.armed

        self.armed_pub.publish(
            armed_msg
        )

        mode_msg = String()
        mode_msg.data = self.mode

        self.mode_pub.publish(
        mode_msg
        )

    # =============================================================
    # SYS_STATUS
    # =============================================================

    def process_sys_status(self, msg):

        # ---------------------------------------------------------
        # Voltage
        # ---------------------------------------------------------

        if msg.voltage_battery != 65535:

            voltage_value = (
                msg.voltage_battery
                / 1000.0
            )

            voltage_msg = Float32()
            voltage_msg.data = voltage_value

            self.voltage_pub.publish(
                voltage_msg
            )

            # Motor power hysteresis



            if (
                voltage_value
                >= self.motor_power_on_threshold
            ):

                self.motors_enabled = True

            elif (
                voltage_value
                <= self.motor_power_off_threshold
            ):

                self.motors_enabled = False




            motors_msg = Bool()
            motors_msg.data = (
                self.motors_enabled
            )

            self.motors_enabled_pub.publish(
                motors_msg
            )

            if (
                self.motors_enabled
                != self.last_motor_power_state
            ):

                if self.motors_enabled:

                    self.get_logger().info(
                        'Propulsion power ENABLED '
                        f'({voltage_value:.2f} V)'
                    )

                else:

                    self.get_logger().warning(
                        'Propulsion power DISABLED '
                        f'({voltage_value:.2f} V)'
                    )

                self.last_motor_power_state = (
                    self.motors_enabled
                )

        # ---------------------------------------------------------
        # Current
        # ---------------------------------------------------------

        if msg.current_battery >= 0:

            current_value = (
                msg.current_battery
                / 100.0
            )

            current_msg = Float32()
            current_msg.data = current_value

            self.current_pub.publish(
                current_msg
            )

    # =============================================================
    # Connection watchdog
    # =============================================================

    def update_connection_status(self):

        connected = False

        if self.last_heartbeat_time is not None:

            elapsed = (
                time.monotonic()
                - self.last_heartbeat_time
            )

            connected = (
                elapsed
                < self.heartbeat_timeout
            )

        self.connected = connected

        msg = Bool()
        msg.data = connected

        self.connected_pub.publish(msg)

        if connected != self.last_connected_state:

            if connected:

                self.get_logger().info(
                    'BlueROV2 autopilot CONNECTED'

                )

                self.request_sys_status_rate()

            else:

                self.get_logger().warning(
                    'BlueROV2 autopilot DISCONNECTED'
                )

            self.last_connected_state = (
                connected
            )

    # =============================================================
    # Manual control safety
    # =============================================================

    def command_is_fresh(self):

        if self.last_cmd_time is None:

            return False

        age = (
            time.monotonic()
            - self.last_cmd_time
        )

        return (
            age <= self.command_timeout
        )

    def motion_is_allowed(self):

        if not self.connected:
            return False

        if not self.motors_enabled:
            return False

        if not self.deadman:
            return False

        if not self.command_is_fresh():
            return False

        if not self.armed:
            return False

        if (
            self.require_manual_mode
            and self.mode != 'MANUAL'
        ):
            return False

        return True

    # =============================================================
    # ROS Twist -> MAVLink MANUAL_CONTROL
    # =============================================================

    @staticmethod
    def clamp(value, minimum, maximum):

        return max(
            minimum,
            min(maximum, value)
        )

    def convert_to_manual_control(self, cmd):

        linear_x = self.clamp(
            cmd.linear.x,
            -1.0,
            1.0
        )

        linear_y = self.clamp(
            cmd.linear.y,
            -1.0,
            1.0
        )

        linear_z = self.clamp(
            cmd.linear.z,
            -1.0,
            1.0
        )

        yaw = self.clamp(
            cmd.angular.z,
            -1.0,
            1.0
        )

        # Apply software gain limit

        linear_x *= self.command_scale
        linear_y *= self.command_scale
        linear_z *= self.command_scale
        yaw *= self.command_scale

        # MAVLink MANUAL_CONTROL
        #
        # X:
        #   + = forward
        #
        # Y:
        #   + = right
        # ROS +Y is left -> invert
        #
        # Z (ArduSub legacy):
        #   0    = one extreme
        #   500  = neutral
        #   1000 = opposite extreme
        #
        # R:
        #   + = clockwise
        # ROS +angular.z is CCW -> invert

        x = int(
            linear_x * 1000.0
        )

        y = int(
            -linear_y * 1000.0
        )

        z = int(
            500.0
            + linear_z * 500.0
        )

        r = int(
            -yaw * 1000.0
        )

        return x, y, z, r

    # =============================================================
    # MANUAL_CONTROL transmitter
    # =============================================================

    def update_manual_control(self):

        allowed = self.motion_is_allowed()

        allowed_msg = Bool()
        allowed_msg.data = allowed

        self.motion_allowed_pub.publish(
            allowed_msg
        )

        output_msg = Bool()
        output_msg.data = (
            self.enable_command_output
        )

        self.command_output_pub.publish(
            output_msg
        )

        if allowed != self.last_motion_allowed:

            self.get_logger().info(
                f'MAVLink motion allowed: '
                f'{allowed}'
            )

            self.last_motion_allowed = (
                allowed
            )

        # ---------------------------------------------------------
        # Critical safety gate
        # ---------------------------------------------------------

        if (
            not self.enable_command_output
            and not self.enable_accessory_output
        ):

            return

        # Without a MAVLink peer there is nowhere to send.
        if not self.connected:

            return

        if self.enable_command_output and allowed:

            x, y, z, r = (
                self.convert_to_manual_control(
                    self.last_cmd
                )
            )

        else:

            # Neutral command
            x = 0
            y = 0
            z = 500
            r = 0

        buttons = self.pending_manual_buttons

        self.master.mav.manual_control_send(
            self.target_system_id,
            x,
            y,
            z,
            r,
            buttons
        )

        # A single transmitted frame creates the button rising edge.
        # The next 20 Hz frame contains zero and releases the button.
        self.pending_manual_buttons = 0

    # =============================================================
    # Shutdown safety
    # =============================================================

    def send_shutdown_neutral(self):

        # =========================================================
        # Safe shutdown
        #
        # Do NOT use ROS logging here because the ROS context may
        # already be shutting down after Ctrl+C.
        # =========================================================

        if not self.connected:
            return

        # Send several neutral MANUAL_CONTROL packets.
        #
        # x = 0
        # y = 0
        # z = 500  -> neutral
        # r = 0
        #
        # Total duration:
        # 5 * 20 ms = ~100 ms

        for _ in range(5 if self.enable_command_output else 0):

            try:

                self.master.mav.manual_control_send(
                    self.target_system_id,
                    0,
                    0,
                    500,
                    0,
                    0
                )

                time.sleep(0.02)

            except Exception:
                break

        if self.enable_accessory_output and self.connected:
            try:
                self.send_rc_override(
                    self.camera_tilt_channel,
                    self.accessory_neutral_pwm
                )

                # ArduSub controls Lights1 through configured joystick
                # functions. BTN13 is dimmer on this vehicle. Repeated
                # press/release edges guarantee a zero-light shutdown.
                dimmer_mask = 1 << self.lights_dimmer_button

                for _ in range(self.lights_steps):
                    self.master.mav.manual_control_send(
                        self.target_system_id,
                        0,
                        0,
                        500,
                        0,
                        dimmer_mask
                    )
                    time.sleep(0.02)
                    self.master.mav.manual_control_send(
                        self.target_system_id,
                        0,
                        0,
                        500,
                        0,
                        0
                    )
                    time.sleep(0.02)
            except Exception:
                pass


def main(args=None):

    rclpy.init(args=args)

    node = MavlinkBridgeNode()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        # -----------------------------------------------------
        # FIRST:
        # Stop any possible vehicle command.
        # -----------------------------------------------------

        node.send_shutdown_neutral()

        # -----------------------------------------------------
        # SECOND:
        # Close MAVLink socket.
        # -----------------------------------------------------

        try:
            node.master.close()
        except Exception:
            pass

        # -----------------------------------------------------
        # THIRD:
        # Destroy ROS node.
        # -----------------------------------------------------

        try:
            node.destroy_node()
        except Exception:
            pass

        # -----------------------------------------------------
        # LAST:
        # Shutdown ROS only if still active.
        # -----------------------------------------------------

        if rclpy.ok():

            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == '__main__':
    main()
