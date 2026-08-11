#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped, TwistStamped
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data

def wrap_to_pi(angle):
    """Wraps an angle in radians to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))

def quaternion_to_yaw(q):
    """Extracts yaw (heading) in radians from a quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class USVPositionControlNode(Node):
    def __init__(self):
        super().__init__('position_control_node')

        # Declare parameters
        self.declare_parameter('kp_yaw', 1.2)
        self.declare_parameter('kp_dist', 0.8)
        self.declare_parameter('max_linear_speed', 1.5)      # m/s
        self.declare_parameter('max_angular_speed', 1.0)     # rad/s
        self.declare_parameter('pivot_angle_thresh', 0.6)    # rad (~34 deg)
        self.declare_parameter('acceptance_radius', 0.5)     # meters
        self.declare_parameter('cmd_vel_topic', '/mavros/setpoint_velocity/cmd_vel')
        self.declare_parameter('pose_topic', '/mavros/local_position/pose')
        self.declare_parameter('publish_rate', 10.0)        # Hz

        # Read parameters
        self.kp_yaw = self.get_parameter('kp_yaw').value
        self.kp_dist = self.get_parameter('kp_dist').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.pivot_angle_thresh = self.get_parameter('pivot_angle_thresh').value
        self.acceptance_radius = self.get_parameter('acceptance_radius').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.pose_topic = self.get_parameter('pose_topic').value
        publish_rate = self.get_parameter('publish_rate').value

        # Current state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.has_pose = False

        # Target state
        self.target_x = 0.0
        self.target_y = 0.0
        self.has_target = False
        self.waypoint_queue = []

        # Subscriptions
        self.sub_pose = self.create_subscription(
            PoseStamped,
            self.pose_topic,
            self.pose_callback,
            qos_profile_sensor_data
        )

        self.sub_goal_point = self.create_subscription(
            Point,
            '/usv/goal_point',
            self.goal_point_callback,
            10
        )

        self.sub_robotx_goal = self.create_subscription(
            Point,
            '/robotx/waypoint_objetivo',
            self.goal_point_callback,
            10
        )

        self.sub_goal_pose = self.create_subscription(
            PoseStamped,
            '/usv/goal_pose',
            self.goal_pose_callback,
            10
        )

        # Publishers
        self.pub_cmd_vel = self.create_publisher(TwistStamped, self.cmd_vel_topic, 10)
        self.pub_status = self.create_publisher(String, '/usv/waypoint_status', 10)

        # Timer loop at 10 Hz
        timer_period = 1.0 / publish_rate if publish_rate > 0 else 0.1
        self.timer = self.create_timer(timer_period, self.control_loop)

        self.get_logger().info(
            f"=== USV Point-to-Point Position Controller Started ===\n"
            f"Listening for Pose: '{self.pose_topic}'\n"
            f"Listening for Goals: '/usv/goal_point' or '/robotx/waypoint_objetivo'\n"
            f"Publishing Velocities: '{self.cmd_vel_topic}' at {publish_rate} Hz\n"
            f"Acceptance Radius: {self.acceptance_radius} m | Max Speed: {self.max_linear_speed} m/s"
        )

    def pose_callback(self, msg: PoseStamped):
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        self.current_yaw = quaternion_to_yaw(msg.pose.orientation)
        self.has_pose = True

    def goal_point_callback(self, msg: Point):
        self.target_x = msg.x
        self.target_y = msg.y
        self.has_target = True
        self.get_logger().info(f"New Target Received: X={self.target_x:.2f}, Y={self.target_y:.2f}")

    def goal_pose_callback(self, msg: PoseStamped):
        self.target_x = msg.pose.position.x
        self.target_y = msg.pose.position.y
        self.has_target = True
        self.get_logger().info(f"New Target Pose Received: X={self.target_x:.2f}, Y={self.target_y:.2f}")

    def control_loop(self):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'

        if not self.has_pose:
            self.get_logger().warn("Waiting for USV local pose on '/mavros/local_position/pose'...", throttle_duration_sec=5.0)
            self.pub_cmd_vel.publish(cmd)
            return

        if not self.has_target:
            self.get_logger().info("IDLE: Send a target point to '/usv/goal_point' or '/robotx/waypoint_objetivo'...", throttle_duration_sec=10.0)
            self.pub_cmd_vel.publish(cmd)
            return

        # Vector & Distance to Target
        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        distance = math.hypot(dx, dy)

        # Target Acceptance Check
        if distance <= self.acceptance_radius:
            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = 0.0
            self.pub_cmd_vel.publish(cmd)

            if self.waypoint_queue:
                next_pt = self.waypoint_queue.pop(0)
                self.target_x = next_pt.x
                self.target_y = next_pt.y
                self.get_logger().info(f"Reached waypoint! Proceeding to next: ({self.target_x:.2f}, {self.target_y:.2f})")
            else:
                status_str = f"GOAL REACHED at ({self.target_x:.2f}, {self.target_y:.2f}) | Dist: {distance:.2f}m"
                self.pub_status.publish(String(data=status_str))
                self.get_logger().info(status_str, throttle_duration_sec=3.0)
            return

        # Calculate bearing to target and heading error
        target_bearing = math.atan2(dy, dx)
        heading_error = wrap_to_pi(target_bearing - self.current_yaw)

        # Proportional Angular Control
        angular_z = float(np.clip(self.kp_yaw * heading_error, -self.max_angular_speed, self.max_angular_speed))

        # Velocity Control Logic
        if abs(heading_error) > self.pivot_angle_thresh:
            # 1. Large heading error -> Pivot in place first
            linear_x = 0.0
            state = f"PIVOTING (Yaw Err: {math.degrees(heading_error):+.1f}°)"
        else:
            # 2. Aligned with target -> Drive forward, scaling speed by cos(heading_error)
            target_speed = min(self.kp_dist * distance, self.max_linear_speed)
            linear_x = float(target_speed * max(0.0, math.cos(heading_error)))
            state = f"DRIVING (Dist: {distance:.2f}m, Speed: {linear_x:.2f}m/s)"

        cmd.twist.linear.x = linear_x
        cmd.twist.angular.z = angular_z
        self.pub_cmd_vel.publish(cmd)

        status_msg = f"{state} | Pos: ({self.current_x:.2f}, {self.current_y:.2f}) -> Goal: ({self.target_x:.2f}, {self.target_y:.2f})"
        self.pub_status.publish(String(data=status_msg))
        self.get_logger().info(status_msg, throttle_duration_sec=1.0)

def main(args=None):
    rclpy.init(args=args)
    node = USVPositionControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
