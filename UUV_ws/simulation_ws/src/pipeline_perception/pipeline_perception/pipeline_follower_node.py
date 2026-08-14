import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist


class PipelineFollowerNode(Node):

    def __init__(self):
        super().__init__('pipeline_follower_node')

        self.pipeline_distance = None
        self.pipeline_bearing = None
        self.pipeline_heading = None

        self.follower_enabled = True

        self.distance_sub = self.create_subscription(
            Float32,
            '/pipeline/distance',
            self.distance_callback,
            10
        )

        self.bearing_sub = self.create_subscription(
            Float32,
            '/pipeline/bearing',
            self.bearing_callback,
            10
        )

        self.heading_sub = self.create_subscription(
            Float32,
            '/pipeline/heading',
            self.heading_callback,
            10
        )

        self.enable_sub = self.create_subscription(
            Bool,
            '/pipeline/follower_enable',
            self.enable_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            '/rov/cmd_vel',
            10
        )

        self.kp_bearing = 0.006
        self.kp_heading = 0.01

        self.target_distance = 1.0
        self.kp_distance = 0.45

        self.forward_speed = 0.12

        self.max_yaw = 0.4
        self.max_lateral = 0.20

        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info('Pipeline follower iniciado')

    def distance_callback(self, msg):
        self.pipeline_distance = msg.data

    def bearing_callback(self, msg):
        self.pipeline_bearing = msg.data

    def heading_callback(self, msg):
        self.pipeline_heading = msg.data

    def enable_callback(self, msg):
        self.follower_enabled = msg.data

        if self.follower_enabled:
            self.get_logger().info('Follower ENABLED')
        else:
            self.get_logger().warn('Follower DISABLED')

    def publish_stop(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

    def clamp(self, value, limit):
        return max(min(value, limit), -limit)

    def wrap_angle_deg(self, angle):
        while angle > 180.0:
            angle -= 360.0

        while angle < -180.0:
            angle += 360.0

        return angle

    def control_loop(self):

        if not self.follower_enabled:
            self.publish_stop()
            return

        if (
            self.pipeline_distance is None or
            self.pipeline_bearing is None or
            self.pipeline_heading is None
        ):
            return

        cmd = Twist()

        heading_error = self.wrap_angle_deg(
            self.pipeline_heading
        )

        if heading_error > 90.0:
            heading_error -= 180.0

        if heading_error < -90.0:
            heading_error += 180.0

        yaw_cmd = self.kp_heading * heading_error

        target_bearing = 90.0

        bearing_error = self.wrap_angle_deg(
            self.pipeline_bearing - target_bearing
        )

        bearing_cmd = self.kp_bearing * bearing_error

        distance_error = self.pipeline_distance - self.target_distance
        distance_cmd = self.kp_distance * distance_error

        lateral_cmd = -distance_cmd + 0.5 * bearing_cmd

        yaw_cmd = self.clamp(yaw_cmd, self.max_yaw)
        lateral_cmd = self.clamp(lateral_cmd, self.max_lateral)

        cmd.linear.x = self.forward_speed
        cmd.linear.y = lateral_cmd
        cmd.angular.z = -yaw_cmd

        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f'enabled={self.follower_enabled} '
            f'target={self.target_distance:.2f} '
            f'dist={self.pipeline_distance:.2f} '
            f'err={distance_error:.2f} '
            f'bearing={self.pipeline_bearing:.1f} '
            f'heading={self.pipeline_heading:.1f} '
            f'lat={lateral_cmd:.2f} '
            f'yaw={yaw_cmd:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)

    node = PipelineFollowerNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
