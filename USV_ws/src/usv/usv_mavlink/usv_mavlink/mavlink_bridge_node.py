#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import (
    Bool,
    Float32,
    Float64,
    Int8,
    String,
    UInt8,
)

from pymavlink import mavutil


class UsvMavlinkBridge(Node):
    """Read-only MAVLink telemetry bridge for the BlueBoat."""

    def __init__(self) -> None:
        super().__init__('usv_mavlink_bridge')

        # --------------------------------------------------------------
        # MAVLink configuration
        # --------------------------------------------------------------

        self.declare_parameter('udp_port', 14553)

        self.declare_parameter(
            'target_system_id',
            2,
        )

        self.declare_parameter(
            'target_component_id',
            1,
        )

        self.declare_parameter(
            'heartbeat_timeout',
            3.0,
        )

        self.udp_port = int(
            self.get_parameter(
                'udp_port'
            ).value
        )

        self.target_system_id = int(
            self.get_parameter(
                'target_system_id'
            ).value
        )

        self.target_component_id = int(
            self.get_parameter(
                'target_component_id'
            ).value
        )

        self.heartbeat_timeout = float(
            self.get_parameter(
                'heartbeat_timeout'
            ).value
        )

        # --------------------------------------------------------------
        # State
        # --------------------------------------------------------------

        self.last_heartbeat_time = None
        self.connected = False

        self.armed = False
        self.mode = 'UNKNOWN'

        # --------------------------------------------------------------
        # ROS publishers
        # --------------------------------------------------------------

        self.connected_pub = self.create_publisher(
            Bool,
            '/usv/connected',
            10,
        )

        self.armed_pub = self.create_publisher(
            Bool,
            '/usv/armed',
            10,
        )

        self.mode_pub = self.create_publisher(
            String,
            '/usv/mode',
            10,
        )

        self.voltage_pub = self.create_publisher(
            Float32,
            '/usv/power/voltage',
            10,
        )

        self.current_pub = self.create_publisher(
            Float32,
            '/usv/power/current',
            10,
        )

        self.remaining_pub = self.create_publisher(
            Int8,
            '/usv/power/remaining',
            10,
        )

        self.gps_fix_pub = self.create_publisher(
            UInt8,
            '/usv/gps/fix_type',
            10,
        )

        self.gps_satellites_pub = self.create_publisher(
            UInt8,
            '/usv/gps/satellites',
            10,
        )

        self.latitude_pub = self.create_publisher(
            Float64,
            '/usv/gps/latitude',
            10,
        )

        self.longitude_pub = self.create_publisher(
            Float64,
            '/usv/gps/longitude',
            10,
        )

        self.yaw_pub = self.create_publisher(
            Float32,
            '/usv/attitude/yaw_deg',
            10,
        )

        # --------------------------------------------------------------
        # MAVLink receive socket
        #
        # IMPORTANT:
        # This first implementation is intentionally read-only.
        # It does not ARM, DISARM, change mode or command propulsion.
        # --------------------------------------------------------------

        self.master = mavutil.mavlink_connection(
            f'udpin:0.0.0.0:{self.udp_port}'
        )

        # Fast non-blocking receive loop.
        self.rx_timer = self.create_timer(
            0.02,
            self.process_mavlink,
        )

        # Connection supervision.
        self.connection_timer = self.create_timer(
            0.20,
            self.update_connection_state,
        )

        self.get_logger().info(
            'BlueBoat MAVLink bridge started in READ-ONLY mode.'
        )

        self.get_logger().info(
            f'Listening on UDP 0.0.0.0:{self.udp_port}'
        )

        self.get_logger().info(
            'Target autopilot: '
            f'SYSID={self.target_system_id}, '
            f'COMPID={self.target_component_id}'
        )

    # ==================================================================
    # Helpers
    # ==================================================================

    def is_target_system(self, msg) -> bool:
        return (
            msg.get_srcSystem()
            == self.target_system_id
        )

    def is_target_autopilot(self, msg) -> bool:
        return (
            msg.get_srcSystem()
            == self.target_system_id
            and
            msg.get_srcComponent()
            == self.target_component_id
        )

    # ==================================================================
    # MAVLink processing
    # ==================================================================

    def process_mavlink(self) -> None:

        while True:

            msg = self.master.recv_match(
                blocking=False
            )

            if msg is None:
                break

            msg_type = msg.get_type()

            if msg_type == 'BAD_DATA':
                continue

            # ----------------------------------------------------------
            # HEARTBEAT
            #
            # Only SYSID=2 / COMPID=1 is allowed to define the
            # BlueBoat armed state and flight/navigation mode.
            # ----------------------------------------------------------

            if (
                msg_type == 'HEARTBEAT'
                and self.is_target_autopilot(msg)
            ):

                self.process_heartbeat(msg)
                continue

            # Ignore telemetry from other systems such as QGC,
            # companion computers or MAVLink routing components.
            if not self.is_target_system(msg):
                continue

            if msg_type == 'SYS_STATUS':
                self.process_sys_status(msg)

            elif msg_type == 'BATTERY_STATUS':
                self.process_battery_status(msg)

            elif msg_type == 'GPS_RAW_INT':
                self.process_gps_raw(msg)

            elif msg_type == 'GLOBAL_POSITION_INT':
                self.process_global_position(msg)

            elif msg_type == 'ATTITUDE':
                self.process_attitude(msg)

    # ==================================================================
    # HEARTBEAT
    # ==================================================================

    def process_heartbeat(self, msg) -> None:

        self.last_heartbeat_time = time.monotonic()

        armed = bool(
            msg.base_mode
            & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        )

        mode = mavutil.mode_string_v10(msg)

        if not self.connected:
            self.get_logger().info(
                'BlueBoat autopilot connected.'
            )

        if armed != self.armed:
            self.get_logger().info(
                'BlueBoat armed state: '
                f'{armed}'
            )

        if mode != self.mode:
            self.get_logger().info(
                f'BlueBoat mode: {mode}'
            )

        self.connected = True
        self.armed = armed
        self.mode = mode

        armed_msg = Bool()
        armed_msg.data = armed
        self.armed_pub.publish(armed_msg)

        mode_msg = String()
        mode_msg.data = mode
        self.mode_pub.publish(mode_msg)

    # ==================================================================
    # Power
    # ==================================================================

    def publish_power(
        self,
        voltage_mv,
        current_ca,
        remaining,
    ) -> None:

        # MAVLink uses millivolts.
        if voltage_mv not in (
            None,
            0,
            65535,
        ):
            msg = Float32()
            msg.data = float(
                voltage_mv
            ) / 1000.0
            self.voltage_pub.publish(msg)

        # MAVLink current is in centiamps.
        if current_ca not in (
            None,
            -1,
        ):
            msg = Float32()
            msg.data = float(
                current_ca
            ) / 100.0
            self.current_pub.publish(msg)

        if remaining is not None:

            value = int(remaining)

            if -1 <= value <= 100:
                msg = Int8()
                msg.data = value
                self.remaining_pub.publish(msg)

    def process_sys_status(self, msg) -> None:

        self.publish_power(
            msg.voltage_battery,
            msg.current_battery,
            msg.battery_remaining,
        )

    def process_battery_status(self, msg) -> None:

        voltage_mv = None

        if msg.voltages:
            voltage_mv = msg.voltages[0]

        self.publish_power(
            voltage_mv,
            msg.current_battery,
            msg.battery_remaining,
        )

    # ==================================================================
    # GPS
    # ==================================================================

    def process_gps_raw(self, msg) -> None:

        fix_msg = UInt8()
        fix_msg.data = int(msg.fix_type)
        self.gps_fix_pub.publish(fix_msg)

        satellites = int(
            msg.satellites_visible
        )

        if satellites < 0:
            satellites = 0

        if satellites > 255:
            satellites = 255

        satellites_msg = UInt8()
        satellites_msg.data = satellites

        self.gps_satellites_pub.publish(
            satellites_msg
        )

    def process_global_position(self, msg) -> None:

        latitude_msg = Float64()
        latitude_msg.data = (
            float(msg.lat) / 1.0e7
        )

        longitude_msg = Float64()
        longitude_msg.data = (
            float(msg.lon) / 1.0e7
        )

        self.latitude_pub.publish(
            latitude_msg
        )

        self.longitude_pub.publish(
            longitude_msg
        )

    # ==================================================================
    # Attitude
    # ==================================================================

    def process_attitude(self, msg) -> None:

        yaw_deg = math.degrees(
            float(msg.yaw)
        )

        yaw_msg = Float32()
        yaw_msg.data = yaw_deg

        self.yaw_pub.publish(
            yaw_msg
        )

    # ==================================================================
    # Connection supervision
    # ==================================================================

    def update_connection_state(self) -> None:

        now = time.monotonic()

        connected = False

        if self.last_heartbeat_time is not None:

            age = (
                now
                - self.last_heartbeat_time
            )

            connected = (
                age
                <= self.heartbeat_timeout
            )

        if (
            self.connected
            and not connected
        ):
            self.get_logger().warning(
                'BlueBoat heartbeat timeout.'
            )

            self.mode = 'UNKNOWN'
            self.armed = False

            mode_msg = String()
            mode_msg.data = self.mode
            self.mode_pub.publish(
                mode_msg
            )

            armed_msg = Bool()
            armed_msg.data = False
            self.armed_pub.publish(
                armed_msg
            )

        self.connected = connected

        connected_msg = Bool()
        connected_msg.data = connected

        self.connected_pub.publish(
            connected_msg
        )

    # ==================================================================
    # Shutdown
    # ==================================================================

    def destroy_node(self) -> None:

        try:
            self.master.close()
        except Exception:
            pass

        super().destroy_node()


def main(args=None) -> None:

    rclpy.init(args=args)

    node = None

    try:
        node = UsvMavlinkBridge()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
