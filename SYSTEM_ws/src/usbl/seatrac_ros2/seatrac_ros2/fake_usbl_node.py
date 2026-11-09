#!/usr/bin/env python3
"""Deterministic USBL simulator for testing without connected hardware."""

import math
import random

from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.node import Node


class FakeUsblNode(Node):
    """Publish the relative-pose test contract with optional dropouts."""

    def __init__(self) -> None:
        super().__init__("fake_usbl_node")
        self.declare_parameter("publish_rate_hz", 2.0)
        self.declare_parameter("forward_m", 2.0)
        self.declare_parameter("lateral_amplitude_m", 0.5)
        self.declare_parameter("motion_period_s", 12.0)
        self.declare_parameter("noise_stddev_m", 0.02)
        self.declare_parameter("dropout_every_s", 0.0)
        self.declare_parameter("dropout_duration_s", 3.0)
        self.declare_parameter("random_seed", 42)

        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            "/usbl/relative_pose",
            10,
        )
        self.start_s = self.get_clock().now().nanoseconds * 1.0e-9
        seed = int(self.get_parameter("random_seed").value)
        self.random = random.Random(seed)
        rate = max(0.1, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(1.0 / rate, self.publish_fix)
        self.get_logger().info(
            "Publishing simulated USBL fixes on /usbl/relative_pose."
        )

    def publish_fix(self) -> None:
        now = self.get_clock().now()
        elapsed = now.nanoseconds * 1.0e-9 - self.start_s
        dropout_every = float(self.get_parameter("dropout_every_s").value)
        dropout_duration = max(
            0.0,
            float(self.get_parameter("dropout_duration_s").value),
        )
        if dropout_every > 0.0 and elapsed % dropout_every < dropout_duration:
            return

        period = max(0.1, float(self.get_parameter("motion_period_s").value))
        forward = float(self.get_parameter("forward_m").value)
        amplitude = float(self.get_parameter("lateral_amplitude_m").value)
        noise = max(0.0, float(self.get_parameter("noise_stddev_m").value))
        variance = max(0.15, noise) ** 2

        message = PoseWithCovarianceStamped()
        message.header.stamp = now.to_msg()
        message.header.frame_id = "base_link"
        message.pose.pose.position.x = forward + self.random.gauss(0.0, noise)
        message.pose.pose.position.y = (
            amplitude * math.sin(2.0 * math.pi * elapsed / period)
            + self.random.gauss(0.0, noise)
        )
        message.pose.pose.orientation.w = 1.0
        message.pose.covariance[0] = variance
        message.pose.covariance[7] = variance
        message.pose.covariance[14] = 0.25 ** 2
        message.pose.covariance[21] = 1.0e6
        message.pose.covariance[28] = 1.0e6
        message.pose.covariance[35] = 1.0e6
        self.publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FakeUsblNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
