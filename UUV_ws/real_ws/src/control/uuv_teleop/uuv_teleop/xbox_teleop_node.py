import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class XboxTeleopNode(Node):

    def __init__(self):
        super().__init__('xbox_teleop_node')

        # ---------------------------------------------------------
        # Xbox mapping measured on this controller
        # ---------------------------------------------------------

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

        # ---------------------------------------------------------
        # Parameters
        # ---------------------------------------------------------

        self.declare_parameter('deadzone', 0.08)

        self.declare_parameter('max_linear_x', 1.0)
        self.declare_parameter('max_linear_y', 1.0)
        self.declare_parameter('max_linear_z', 1.0)
        self.declare_parameter('max_yaw', 1.0)

        self.deadzone = (
            self.get_parameter('deadzone')
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

        # ---------------------------------------------------------
        # ROS interfaces
        # ---------------------------------------------------------

        self.publisher = self.create_publisher(
            Twist,
            '/uuv/cmd_vel_manual',
            10
        )

        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        self.get_logger().info(
            'Xbox teleoperation node ready.'
        )

        self.get_logger().info(
            'Hold RB to enable motion commands.'
        )

    def apply_deadzone(self, value):
        if abs(value) < self.deadzone:
            return 0.0

        return value

    def joy_callback(self, msg):

        cmd = Twist()

        # ---------------------------------------------------------
        # Safety: RB must be held
        # ---------------------------------------------------------

        if len(msg.buttons) <= self.BUTTON_RB:
            self.publisher.publish(cmd)
            return

        enabled = msg.buttons[self.BUTTON_RB] == 1

        if not enabled:
            self.publisher.publish(cmd)
            return

        if len(msg.axes) < 6:
            self.get_logger().warning(
                'Joystick message does not contain expected axes.'
            )
            self.publisher.publish(cmd)
            return

        # ---------------------------------------------------------
        # Horizontal movement
        # ---------------------------------------------------------

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

        # Forward / backward
        cmd.linear.x = (
            left_y * self.max_linear_x
        )

        # Left / right
        #
        # Positive Y in ROS = left.
        # Your controller:
        # left = +1, right = -1
        cmd.linear.y = (
            left_x * self.max_linear_y
        )

        # ---------------------------------------------------------
        # Vertical movement
        #
        # RT -> up
        # LT -> down
        #
        # Both triggers:
        # released = 0
        # pressed  = -1
        #
        # RT pressed:
        # 0 - (-1) = +1
        #
        # LT pressed:
        # (-1) - 0 = -1
        # ---------------------------------------------------------

        vertical = lt - rt

        vertical = self.apply_deadzone(vertical)

        cmd.linear.z = (
            vertical * self.max_linear_z
        )

        # ---------------------------------------------------------
        # Yaw
        #
        # Positive angular.z = left / CCW
        #
        # Your right stick:
        # left = +1
        # right = -1
        # ---------------------------------------------------------

        cmd.angular.z = (
            right_x * self.max_yaw
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
