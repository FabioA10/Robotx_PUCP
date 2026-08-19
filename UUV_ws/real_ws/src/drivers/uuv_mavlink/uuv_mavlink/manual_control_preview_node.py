import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_msgs.msg import Int16MultiArray


class ManualControlPreviewNode(Node):

    def __init__(self):
        super().__init__('manual_control_preview')

        self.declare_parameter('command_timeout', 0.30)

        self.command_timeout = (
            self.get_parameter('command_timeout')
            .get_parameter_value()
            .double_value
        )

        self.connected = False
        self.motors_enabled = False
        self.deadman = False
        self.armed = False

        self.last_cmd_time = None
        self.last_cmd = Twist()

        self.last_input_valid = None
        self.last_motion_allowed = None

        # ---------------------------------------------------------
        # Subscribers
        # ---------------------------------------------------------

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
            Bool,
            '/uuv/connected',
            self.connected_callback,
            10
        )

        self.create_subscription(
            Bool,
            '/uuv/power/motors_enabled',
            self.motors_callback,
            10
        )

        self.create_subscription(
            Bool,
            '/uuv/armed',
            self.armed_callback,
            10
        )

        # ---------------------------------------------------------
        # Publishers
        # ---------------------------------------------------------

        self.preview_pub = self.create_publisher(
            Int16MultiArray,
            '/uuv/control/manual_control_preview',
            10
        )

        self.input_valid_pub = self.create_publisher(
            Bool,
            '/uuv/control/input_valid',
            10
        )

        self.motion_allowed_pub = self.create_publisher(
            Bool,
            '/uuv/control/motion_allowed',
            10
        )

        self.timer = self.create_timer(
            0.1,
            self.update
        )

        self.get_logger().info(
            'MANUAL_CONTROL preview started.'
        )

        self.get_logger().info(
            'NO MAVLink commands are transmitted by this node.'
        )

    # -------------------------------------------------------------
    # Callbacks
    # -------------------------------------------------------------

    def cmd_callback(self, msg):
        self.last_cmd = msg
        self.last_cmd_time = time.monotonic()

    def deadman_callback(self, msg):
        self.deadman = msg.data

    def connected_callback(self, msg):
        self.connected = msg.data

    def motors_callback(self, msg):
        self.motors_enabled = msg.data

    def armed_callback(self, msg):
        self.armed = msg.data

    # -------------------------------------------------------------
    # Helpers
    # -------------------------------------------------------------

    @staticmethod
    def clamp(value, minimum, maximum):
        return max(minimum, min(maximum, value))

    def command_is_fresh(self):

        if self.last_cmd_time is None:
            return False

        age = time.monotonic() - self.last_cmd_time

        return age <= self.command_timeout

    def convert_to_manual_control(self, cmd):

        linear_x = self.clamp(cmd.linear.x, -1.0, 1.0)
        linear_y = self.clamp(cmd.linear.y, -1.0, 1.0)
        linear_z = self.clamp(cmd.linear.z, -1.0, 1.0)
        yaw = self.clamp(cmd.angular.z, -1.0, 1.0)

        # MAVLink MANUAL_CONTROL:
        #
        # x:
        #   +1000 forward
        #   -1000 backward
        #
        # y:
        #   +1000 right
        #   -1000 left
        #
        # Our ROS convention:
        #   +Y = left
        #
        # Therefore Y must be inverted.
        #
        # r:
        #   +1000 clockwise
        #
        # Our ROS convention:
        #   +angular.z = CCW
        #
        # Therefore yaw must be inverted.
        #
        # ArduSub legacy Z:
        #   0    = down
        #   500  = neutral
        #   1000 = up

        x = int(linear_x * 1000.0)
        y = int(-linear_y * 1000.0)
        z = int(500.0 + linear_z * 500.0)
        r = int(-yaw * 1000.0)

        return x, y, z, r

    # -------------------------------------------------------------
    # Safety evaluation
    # -------------------------------------------------------------

    def update(self):

        cmd_fresh = self.command_is_fresh()

        input_valid = (
            self.connected
            and self.motors_enabled
            and self.deadman
            and cmd_fresh
        )

        motion_allowed = (
            input_valid
            and self.armed
        )

        if input_valid:
            x, y, z, r = self.convert_to_manual_control(
                self.last_cmd
            )
        else:
            # Neutral MANUAL_CONTROL command
            x = 0
            y = 0
            z = 500
            r = 0

        preview_msg = Int16MultiArray()
        preview_msg.data = [x, y, z, r]

        self.preview_pub.publish(preview_msg)

        valid_msg = Bool()
        valid_msg.data = input_valid
        self.input_valid_pub.publish(valid_msg)

        allowed_msg = Bool()
        allowed_msg.data = motion_allowed
        self.motion_allowed_pub.publish(allowed_msg)

        if input_valid != self.last_input_valid:

            self.get_logger().info(
                f'Input valid: {input_valid}'
            )

            self.last_input_valid = input_valid

        if motion_allowed != self.last_motion_allowed:

            self.get_logger().info(
                f'Motion allowed: {motion_allowed}'
            )

            self.last_motion_allowed = motion_allowed


def main(args=None):

    rclpy.init(args=args)

    node = ManualControlPreviewNode()

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
