import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import String

from pymavlink import mavutil


class MavlinkBridgeNode(Node):

    def __init__(self):
        super().__init__('uuv_mavlink_bridge')

        # ---------------------------------------------------------
        # Parameters
        # ---------------------------------------------------------

        self.declare_parameter('udp_port', 14552)
        self.declare_parameter('target_system_id', 1)
        self.declare_parameter('target_component_id', 1)
        self.declare_parameter('heartbeat_timeout', 3.0)

        # Propulsion power bus thresholds
        self.declare_parameter('motor_power_on_threshold', 10.0)
        self.declare_parameter('motor_power_off_threshold', 5.0)

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

        # ---------------------------------------------------------
        # Publishers
        # ---------------------------------------------------------

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

        # Initial state.
        # The real state will be inferred from SYS_STATUS voltage.
        self.motors_enabled = False

        # ---------------------------------------------------------
        # MAVLink
        # ---------------------------------------------------------

        connection_string = f'udpin:0.0.0.0:{self.udp_port}'

        self.get_logger().info(
            f'Opening MAVLink connection on {connection_string}'
        )

        self.master = mavutil.mavlink_connection(
            connection_string
        )

        self.last_heartbeat_time = None
        self.last_connected_state = None
        self.last_motor_power_state = None

        # Read MAVLink without blocking the ROS executor
        self.read_timer = self.create_timer(
            0.01,
            self.read_mavlink
        )

        # Connection watchdog at 5 Hz
        self.status_timer = self.create_timer(
            0.2,
            self.update_connection_status
        )

        self.get_logger().info(
            'UUV MAVLink bridge started in READ-ONLY mode.'
        )

        self.get_logger().info(
            f'Target autopilot: '
            f'SYSID={self.target_system_id}, '
            f'COMPID={self.target_component_id}'
        )

        self.get_logger().info(
            f'Motor power thresholds: '
            f'OFF <= {self.motor_power_off_threshold:.1f} V | '
            f'ON >= {self.motor_power_on_threshold:.1f} V'
        )

    # -------------------------------------------------------------
    # MAVLink receiver
    # -------------------------------------------------------------

    def read_mavlink(self):

        while True:

            msg = self.master.recv_match(
                blocking=False
            )

            if msg is None:
                break

            # Only process messages from the ArduSub autopilot
            if (
                msg.get_srcSystem() != self.target_system_id or
                msg.get_srcComponent() != self.target_component_id
            ):
                continue

            msg_type = msg.get_type()

            if msg_type == 'HEARTBEAT':
                self.process_heartbeat(msg)

            elif msg_type == 'SYS_STATUS':
                self.process_sys_status(msg)

    # -------------------------------------------------------------
    # HEARTBEAT
    # -------------------------------------------------------------

    def process_heartbeat(self, msg):

        self.last_heartbeat_time = time.monotonic()

        armed = bool(
            msg.base_mode
            & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        )

        mode = mavutil.mode_string_v10(msg)

        armed_msg = Bool()
        armed_msg.data = armed
        self.armed_pub.publish(armed_msg)

        mode_msg = String()
        mode_msg.data = mode
        self.mode_pub.publish(mode_msg)

    # -------------------------------------------------------------
    # SYS_STATUS
    # -------------------------------------------------------------

    def process_sys_status(self, msg):

        # ---------------------------------------------------------
        # Voltage
        # ---------------------------------------------------------

        if msg.voltage_battery != 65535:

            voltage_value = msg.voltage_battery / 1000.0

            voltage_msg = Float32()
            voltage_msg.data = voltage_value
            self.voltage_pub.publish(voltage_msg)

            # -----------------------------------------------------
            # Motor power state
            #
            # Hysteresis:
            #   voltage >= ON threshold  -> motors_enabled = True
            #   voltage <= OFF threshold -> motors_enabled = False
            #   between thresholds       -> preserve previous state
            # -----------------------------------------------------

            if voltage_value >= self.motor_power_on_threshold:
                self.motors_enabled = True

            elif voltage_value <= self.motor_power_off_threshold:
                self.motors_enabled = False

            motors_msg = Bool()
            motors_msg.data = self.motors_enabled
            self.motors_enabled_pub.publish(motors_msg)

            if self.motors_enabled != self.last_motor_power_state:

                if self.motors_enabled:
                    self.get_logger().info(
                        f'Propulsion power ENABLED '
                        f'({voltage_value:.2f} V)'
                    )
                else:
                    self.get_logger().warning(
                        f'Propulsion power DISABLED '
                        f'({voltage_value:.2f} V)'
                    )

                self.last_motor_power_state = self.motors_enabled

        # ---------------------------------------------------------
        # Current
        # ---------------------------------------------------------

        if msg.current_battery >= 0:

            current_value = msg.current_battery / 100.0

            current_msg = Float32()
            current_msg.data = current_value
            self.current_pub.publish(current_msg)

    # -------------------------------------------------------------
    # Connection status
    # -------------------------------------------------------------

    def update_connection_status(self):

        connected = False

        if self.last_heartbeat_time is not None:

            elapsed = (
                time.monotonic()
                - self.last_heartbeat_time
            )

            connected = elapsed < self.heartbeat_timeout

        connected_msg = Bool()
        connected_msg.data = connected
        self.connected_pub.publish(connected_msg)

        if connected != self.last_connected_state:

            if connected:
                self.get_logger().info(
                    'BlueROV2 autopilot CONNECTED'
                )
            else:
                self.get_logger().warning(
                    'BlueROV2 autopilot DISCONNECTED'
                )

            self.last_connected_state = connected


def main(args=None):

    rclpy.init(args=args)

    node = MavlinkBridgeNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
