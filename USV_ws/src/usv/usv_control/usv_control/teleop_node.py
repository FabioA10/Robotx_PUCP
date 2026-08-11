#!/usr/bin/env python3
import sys
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

BANNER = """
============================================================
              USV KEYBOARD TELEOP NODE
============================================================
Control your Unmanned Surface Vehicle (USV) via Keyboard:

  [ W / i / Up Arrow ]   : Increase Forward Speed (+linear.x)
  [ S / k / Down Arrow ] : Increase Reverse Speed (-linear.x)
  [ A / j / Left Arrow ] : Turn Port / Left (+angular.z)
  [ D / l / Right Arrow ]: Turn Starboard / Right (-angular.z)

  [ Space / X ]          : EMERGENCY STOP (Zero velocity)
  [ Q / Z ]              : Increase / Decrease Max Linear Speed Limit
  [ E / C ]              : Increase / Decrease Max Angular Speed Limit
  [ R ]                  : Reset Velocities to Zero

Press Ctrl+C to exit.
============================================================
"""

class USVTeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        
        # Declare parameters
        self.declare_parameter('cmd_vel_topic', '/mavros/setpoint_velocity/cmd_vel')
        self.declare_parameter('max_linear_speed', 2.0)
        self.declare_parameter('max_angular_speed', 1.5)
        self.declare_parameter('linear_step', 0.1)
        self.declare_parameter('angular_step', 0.1)
        self.declare_parameter('publish_rate', 10.0)

        # Get parameter values
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.max_linear_speed = self.get_parameter('max_linear_speed').get_parameter_value().double_value
        self.max_angular_speed = self.get_parameter('max_angular_speed').get_parameter_value().double_value
        self.linear_step = self.get_parameter('linear_step').get_parameter_value().double_value
        self.angular_step = self.get_parameter('angular_step').get_parameter_value().double_value
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        # Velocity state
        self.target_linear_x = 0.0
        self.target_angular_z = 0.0

        # Determine message type based on topic name
        self.use_stamped = 'cmd_vel_unstamped' not in self.cmd_vel_topic
        if self.use_stamped:
            self.publisher_ = self.create_publisher(TwistStamped, self.cmd_vel_topic, 10)
        else:
            self.publisher_ = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        
        # Timer for continuous publishing (required for MAVROS setpoints)
        timer_period = 1.0 / publish_rate if publish_rate > 0 else 0.1
        self.timer = self.create_timer(timer_period, self.publish_cmd_vel)

        msg_type_str = "TwistStamped" if self.use_stamped else "Twist"
        self.get_logger().info(f"USV Teleop Node started. Publishing {msg_type_str} to '{self.cmd_vel_topic}' at {publish_rate} Hz.")

    def publish_cmd_vel(self):
        if self.use_stamped:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            msg.twist.linear.x = self.target_linear_x
            msg.twist.angular.z = self.target_angular_z
            self.publisher_.publish(msg)
        else:
            msg = Twist()
            msg.linear.x = self.target_linear_x
            msg.angular.z = self.target_angular_z
            self.publisher_.publish(msg)

    def print_status(self):
        sys.stdout.write(
            f"\r[STATUS] Linear Speed Limit: {self.max_linear_speed:.2f} m/s | "
            f"Angular Speed Limit: {self.max_angular_speed:.2f} rad/s | "
            f"Current Cmd -> Linear X: {self.target_linear_x:+.2f} m/s, Angular Z: {self.target_angular_z:+.2f} rad/s   "
        )
        sys.stdout.flush()

    def get_key(self, settings, timeout=0.05):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        key = ''
        if rlist:
            key = sys.stdin.read(1)
            # Handle multi-byte escape sequences (e.g. arrow keys)
            if key == '\x1b':
                # Check for additional arrow key bytes with brief timeout
                rlist_seq, _, _ = select.select([sys.stdin], [], [], 0.01)
                if rlist_seq:
                    key += sys.stdin.read(2)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def run(self):
        settings = termios.tcgetattr(sys.stdin)
        print(BANNER)
        self.print_status()

        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.01)
                key = self.get_key(settings, timeout=0.05)
                
                if not key:
                    continue

                if key in ['w', 'W', 'i', 'I', '\x1b[A']:  # Forward / Up Arrow
                    self.target_linear_x = min(self.target_linear_x + self.linear_step, self.max_linear_speed)
                elif key in ['s', 'S', 'k', 'K', '\x1b[B']:  # Backward / Down Arrow
                    self.target_linear_x = max(self.target_linear_x - self.linear_step, -self.max_linear_speed)
                elif key in ['a', 'A', 'j', 'J', '\x1b[D']:  # Turn Left / Left Arrow
                    self.target_angular_z = min(self.target_angular_z + self.angular_step, self.max_angular_speed)
                elif key in ['d', 'D', 'l', 'L', '\x1b[C']:  # Turn Right / Right Arrow
                    self.target_angular_z = max(self.target_angular_z - self.angular_step, -self.max_angular_speed)
                elif key in [' ', 'x', 'X']:  # Emergency Stop
                    self.target_linear_x = 0.0
                    self.target_angular_z = 0.0
                elif key in ['r', 'R']:  # Reset velocities
                    self.target_linear_x = 0.0
                    self.target_angular_z = 0.0
                elif key in ['q', 'Q']:  # Increase max linear speed
                    self.max_linear_speed += 0.1
                elif key in ['z', 'Z']:  # Decrease max linear speed
                    self.max_linear_speed = max(0.1, self.max_linear_speed - 0.1)
                elif key in ['e', 'E']:  # Increase max angular speed
                    self.max_angular_speed += 0.1
                elif key in ['c', 'C']:  # Decrease max angular speed
                    self.max_angular_speed = max(0.1, self.max_angular_speed - 0.1)
                elif key == '\x03':  # Ctrl+C
                    break

                # Clamp targets within max limits
                self.target_linear_x = max(-self.max_linear_speed, min(self.target_linear_x, self.max_linear_speed))
                self.target_angular_z = max(-self.max_angular_speed, min(self.target_angular_z, self.max_angular_speed))

                self.print_status()

        except Exception as e:
            self.get_logger().error(f"Error reading keyboard input: {e}")
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            # Stop vehicle on node exit
            self.target_linear_x = 0.0
            self.target_angular_z = 0.0
            self.publish_cmd_vel()
            print("\nExiting USV Teleop Node. Vehicle stopped.")

def main(args=None):
    rclpy.init(args=args)
    node = USVTeleopNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
