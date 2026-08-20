import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Empty, Int8


class XboxTeleopNode(Node):

    def __init__(self):
        super().__init__('xbox_teleop_node')

        # =========================================================
        # Xbox mapping measured on this controller
        # =========================================================

        self.AXIS_LEFT_X = 0
        self.AXIS_LEFT_Y = 1

        self.AXIS_RIGHT_X = 2
        self.AXIS_RIGHT_Y = 3

        self.AXIS_LT = 4
        self.AXIS_RT = 5

        self.BUTTON_A = 0
        self.BUTTON_B = 1
        self.BUTTON_X = 2
        self.BUTTON_Y = 3

        self.BUTTON_LB = 9
        self.BUTTON_RB = 10

        self.BUTTON_DPAD_UP = 11
        self.BUTTON_DPAD_DOWN = 12
        self.BUTTON_DPAD_LEFT = 13
        self.BUTTON_DPAD_RIGHT = 14

        # =========================================================
        # Parameters
        # =========================================================

        self.declare_parameter('deadzone', 0.08)
        self.declare_parameter('arm_hold_seconds', 1.5)

        self.declare_parameter('max_linear_x', 1.0)
        self.declare_parameter('max_linear_y', 1.0)
        self.declare_parameter('max_linear_z', 1.0)
        self.declare_parameter('max_yaw', 1.0)

        self.deadzone = (
            self.get_parameter('deadzone')
            .get_parameter_value()
            .double_value
        )

        self.arm_hold_seconds = (
            self.get_parameter('arm_hold_seconds')
            .get_parameter_value()
            .double_value
        )

        self.max_linear_x = (
            self.get_parameter('max_linear_x')
            .get_parameter_value()
            .double_value
        )

        self.max_linear_y = (
            self.get_parameter('max_linear_y')
            .get_parameter_value()
            .double_value
        )

        self.max_linear_z = (
            self.get_parameter('max_linear_z')
            .get_parameter_value()
            .double_value
        )

        self.max_yaw = (
            self.get_parameter('max_yaw')
            .get_parameter_value()
            .double_value
        )

        # =========================================================
        # State
        # =========================================================


        # =========================================================
        # ArduSub operational mode sequence
        #
        # MOTOR_DETECT intentionally excluded because it is
        # a motor configuration/detection routine.
        # =========================================================


        self.arm_hold_start = None
        self.arm_request_sent = False

        self.previous_b_pressed = False

        self.previous_dpad_up_pressed = False
        self.previous_dpad_down_pressed = False

        # =========================================================
        # Publishers
        # =========================================================

        self.publisher = self.create_publisher(
            Twist,
            '/uuv/cmd_vel_manual',
            10
        )

        self.mode_step_pub = self.create_publisher(
            Int8,
            '/uuv/control/mode_step',
            10
        )

        self.deadman_publisher = self.create_publisher(
            Bool,
            '/uuv/deadman',
            10
        )

        self.arm_request_pub = self.create_publisher(
            Empty,
            '/uuv/control/arm_request',
            10
        )

        self.disarm_request_pub = self.create_publisher(
            Empty,
            '/uuv/control/disarm_request',
            10
        )

        # =========================================================
        # Subscriber
        # =========================================================

        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )



        # =========================================================
        # Startup information
        # =========================================================

        self.get_logger().info(
            'Xbox teleoperation node ready.'
        )

        self.get_logger().info(
            'X held for 1.5 s -> ARM'
        )

        self.get_logger().info(
            'B -> immediate DISARM'
        )

        self.get_logger().info(
            'Hold RB to enable motion commands.'
        )

        self.get_logger().info(
            'LB + D-pad UP/DOWN -> change vehicle mode.'
        )




    # =============================================================     -------------------------------------------------------------------
    # Helpers
    # =============================================================

    def apply_deadzone(self, value):

        if abs(value) < self.deadzone:
            return 0.0

        return value

    def controls_are_neutral(self, msg):

        # Check all six analog axes.
        # On this controller both triggers are 0 when released.

        return (
            abs(msg.axes[self.AXIS_LEFT_X]) < self.deadzone
            and abs(msg.axes[self.AXIS_LEFT_Y]) < self.deadzone
            and abs(msg.axes[self.AXIS_RIGHT_X]) < self.deadzone
            and abs(msg.axes[self.AXIS_RIGHT_Y]) < self.deadzone
            and abs(msg.axes[self.AXIS_LT]) < self.deadzone
            and abs(msg.axes[self.AXIS_RT]) < self.deadzone
        )

    # =============================================================
    # Joy callback
    # =============================================================

    def joy_callback(self, msg):

        cmd = Twist()

        # ---------------------------------------------------------
        # Validate message before accessing axes/buttons
        # ---------------------------------------------------------

        if len(msg.axes) < 6:

            self.get_logger().warning(
                'Joystick message does not contain expected axes.'
            )

            deadman_msg = Bool()
            deadman_msg.data = False

            self.deadman_publisher.publish(deadman_msg)
            self.publisher.publish(cmd)

            self.arm_hold_start = None
            self.arm_request_sent = False

            return

        if len(msg.buttons) <= self.BUTTON_DPAD_DOWN:

            self.get_logger().warning(
                'Joystick message does not contain expected buttons.'
            )

            deadman_msg = Bool()
            deadman_msg.data = False

            self.deadman_publisher.publish(deadman_msg)
            self.publisher.publish(cmd)

            self.arm_hold_start = None
            self.arm_request_sent = False

            return

        # ---------------------------------------------------------
        # Read buttons
        # ---------------------------------------------------------

        rb_pressed = (
            msg.buttons[self.BUTTON_RB] == 1
        )

        b_pressed = (
            msg.buttons[self.BUTTON_B] == 1
        )

        x_pressed = (
            msg.buttons[self.BUTTON_X] == 1
        )

        lb_pressed = (
            msg.buttons[self.BUTTON_LB] == 1
        )

        dpad_up_pressed = (
            msg.buttons[self.BUTTON_DPAD_UP] == 1
        )

        dpad_down_pressed = (
            msg.buttons[self.BUTTON_DPAD_DOWN] == 1
        )

        # =========================================================
        # DISARM
        #
        # B has highest priority.
        # Immediate request on rising edge.
        # Does NOT require RB.
        # =========================================================

        if (
            b_pressed
            and not self.previous_b_pressed
        ):

            self.disarm_request_pub.publish(
                Empty()
            )

            self.get_logger().warning(
                'DISARM requested from Xbox B button'
            )

            # Cancel any pending ARM sequence
            self.arm_hold_start = None
            self.arm_request_sent = False

        self.previous_b_pressed = b_pressed

        # =========================================================
        # ARM
        #
        # X + RB must be held for arm_hold_seconds.
        #
        # Requirements:
        #   - X pressed
        #   - RB pressed
        #   - B not pressed
        #   - all analog controls neutral
        # =========================================================

        controls_neutral = (
            self.controls_are_neutral(msg)
        )


        # =========================================================
        # MODE SELECTION
        #
        # LB + D-pad UP   -> next mode
        # LB + D-pad DOWN -> previous mode
        #
        # Mode changes are accepted only with:
        #   - RB released
        #   - analog controls neutral
        #   - B not pressed
        # =========================================================

        mode_change_allowed = (
            lb_pressed
            and not rb_pressed
            and not b_pressed
            and controls_neutral
        )


        if mode_change_allowed:

            # -----------------------------------------------------
            # NEXT MODE
            # -----------------------------------------------------

            if (
                dpad_up_pressed
                and not self.previous_dpad_up_pressed
            ):

                step_msg = Int8()
                step_msg.data = 1

                self.mode_step_pub.publish(
                    step_msg
                )

                self.get_logger().warning(
                    'NEXT mode requested from Xbox'
                )


            # -----------------------------------------------------
            # PREVIOUS MODE
            # -----------------------------------------------------

            if (
                dpad_down_pressed
                and not self.previous_dpad_down_pressed
            ):

                step_msg = Int8()
                step_msg.data = -1

                self.mode_step_pub.publish(
                    step_msg
                )

                self.get_logger().warning(
                    'PREVIOUS mode requested from Xbox'
                )


        self.previous_dpad_up_pressed = (
            dpad_up_pressed
        )

        self.previous_dpad_down_pressed = (
            dpad_down_pressed
        )



        arm_conditions = (
            x_pressed
            and not b_pressed
            and rb_pressed
            and controls_neutral
        )

        if arm_conditions:

            if self.arm_hold_start is None:

                self.arm_hold_start = (
                    time.monotonic()
                )

                self.arm_request_sent = False

            elapsed = (
                time.monotonic()
                - self.arm_hold_start
            )

            if (
                elapsed >= self.arm_hold_seconds
                and not self.arm_request_sent
            ):

                self.arm_request_pub.publish(
                    Empty()
                )

                self.get_logger().warning(
                    'ARM requested from Xbox X button'
                )

                self.arm_request_sent = True

        else:

            self.arm_hold_start = None
            self.arm_request_sent = False

        # =========================================================
        # DEADMAN
        # =========================================================

        deadman_msg = Bool()
        deadman_msg.data = rb_pressed

        self.deadman_publisher.publish(
            deadman_msg
        )

        # ---------------------------------------------------------
        # No RB -> always send neutral ROS command
        # ---------------------------------------------------------

        if not rb_pressed:

            self.publisher.publish(cmd)
            return

        # =========================================================
        # Movement
        # =========================================================

        left_x = self.apply_deadzone(
            msg.axes[self.AXIS_LEFT_X]
        )

        left_y = self.apply_deadzone(
            msg.axes[self.AXIS_LEFT_Y]
        )

        right_x = self.apply_deadzone(
            msg.axes[self.AXIS_RIGHT_X]
        )

        lt = msg.axes[self.AXIS_LT]
        rt = msg.axes[self.AXIS_RT]

        # ---------------------------------------------------------
        # Forward / backward
        # ---------------------------------------------------------

        cmd.linear.x = (
            left_y
            * self.max_linear_x
        )

        # ---------------------------------------------------------
        # Left / right
        #
        # ROS:
        # +Y = left
        # ---------------------------------------------------------

        cmd.linear.y = (
            left_x
            * self.max_linear_y
        )

        # ---------------------------------------------------------
        # Vertical
        #
        # RT -> up
        # LT -> down
        # ---------------------------------------------------------

        vertical = lt - rt

        vertical = self.apply_deadzone(
            vertical
        )

        cmd.linear.z = (
            vertical
            * self.max_linear_z
        )

        # ---------------------------------------------------------
        # Yaw
        #
        # +angular.z = left / CCW
        # ---------------------------------------------------------

        cmd.angular.z = (
            right_x
            * self.max_yaw
        )

        self.publisher.publish(cmd)


def main(args=None):

    rclpy.init(args=args)

    node = XboxTeleopNode()

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
