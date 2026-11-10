#!/usr/bin/env python3
"""ROS adapter for previewing UUV-follow commands without actuation."""

import json
import math

from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

from usv_control.core import (
    FollowConfig,
    FollowState,
    RelativeFix,
    UuvFollowController,
)


class UuvFollowPreviewNode(Node):
    """Consume canonical USBL poses and publish only a safe preview topic."""

    def __init__(self, *, context=None, parameter_overrides=None) -> None:
        """Create the preview-only adapter with optional test overrides."""
        super().__init__(
            "uuv_follow_preview_node",
            context=context,
            parameter_overrides=parameter_overrides,
        )
        self.declare_parameter("follow_enabled", False)
        self.declare_parameter("input_topic", "/usbl/relative_pose")
        # Input must already include the verified X150 mounting transform.
        # Guessing sensor orientation in a controller would be unsafe.
        self.declare_parameter("expected_frame", "base_link")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("follow_distance_m", 1.0)
        self.declare_parameter("distance_deadband_m", 0.20)
        self.declare_parameter("max_linear_speed_mps", 0.20)
        self.declare_parameter("max_angular_speed_radps", 0.30)
        self.declare_parameter("fix_timeout_s", 2.50)
        self.declare_parameter("acquisition_fixes", 3)
        self.declare_parameter("max_range_m", 20.0)
        self.declare_parameter("max_relative_speed_mps", 3.0)
        self.declare_parameter("max_horizontal_stddev_m", 0.75)

        config = FollowConfig(
            follow_distance_m=float(self.get_parameter("follow_distance_m").value),
            distance_deadband_m=float(
                self.get_parameter("distance_deadband_m").value
            ),
            max_linear_speed_mps=float(
                self.get_parameter("max_linear_speed_mps").value
            ),
            max_angular_speed_radps=float(
                self.get_parameter("max_angular_speed_radps").value
            ),
            fix_timeout_s=float(self.get_parameter("fix_timeout_s").value),
            acquisition_fixes=int(self.get_parameter("acquisition_fixes").value),
            max_range_m=float(self.get_parameter("max_range_m").value),
            max_relative_speed_mps=float(
                self.get_parameter("max_relative_speed_mps").value
            ),
            expected_frame=str(self.get_parameter("expected_frame").value),
            require_covariance=True,
            max_horizontal_stddev_m=float(
                self.get_parameter("max_horizontal_stddev_m").value
            ),
        )
        self.controller = UuvFollowController(config)
        self.expected_frame = str(self.get_parameter("expected_frame").value)
        self.latest_fix = None

        input_topic = str(self.get_parameter("input_topic").value)
        self.create_subscription(
            PoseWithCovarianceStamped,
            input_topic,
            self.fix_callback,
            qos_profile_sensor_data,
        )
        self.preview_publisher = self.create_publisher(
            TwistStamped,
            "/usv/follow/cmd_vel_preview",
            10,
        )
        self.candidate_publisher = self.create_publisher(
            TwistStamped,
            "/usv/follow/cmd_vel_candidate",
            10,
        )
        self.status_publisher = self.create_publisher(
            String,
            "/usv/follow/status",
            10,
        )
        rate = max(0.1, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(1.0 / rate, self.control_loop)
        self.get_logger().warning(
            "Preview only: this node never publishes to MAVROS or arms the USV."
        )

    def fix_callback(self, message: PoseWithCovarianceStamped) -> None:
        if message.header.frame_id != self.expected_frame:
            self.get_logger().warning(
                "Ignoring USBL fix in frame '%s'; expected '%s'."
                % (message.header.frame_id, self.expected_frame)
            )
            self.latest_fix = None
            return
        stamp = message.header.stamp
        timestamp_s = float(stamp.sec) + float(stamp.nanosec) * 1.0e-9
        if timestamp_s <= 0.0:
            # Receipt time must not make an old/replayed acoustic sample look
            # fresh. The USBL adapter must supply its source timestamp.
            self.get_logger().warning(
                "Ignoring USBL fix without a valid source timestamp."
            )
            self.latest_fix = None
            return
        covariance = message.pose.covariance
        covariance_xy = float(covariance[1])
        covariance_yx = float(covariance[6])
        if (
            not math.isfinite(covariance_xy)
            or not math.isfinite(covariance_yx)
            or abs(covariance_xy - covariance_yx) > 1.0e-6
        ):
            self.get_logger().warning(
                "Ignoring USBL fix with an asymmetric horizontal covariance."
            )
            self.latest_fix = None
            return
        covariance_xy = 0.5 * (covariance_xy + covariance_yx)
        self.latest_fix = RelativeFix(
            x_m=float(message.pose.pose.position.x),
            y_m=float(message.pose.pose.position.y),
            timestamp_s=timestamp_s,
            frame_id=message.header.frame_id,
            variance_x_m2=float(covariance[0]),
            variance_y_m2=float(covariance[7]),
            covariance_xy_m2=covariance_xy,
        )

    def control_loop(self) -> None:
        now = self.get_clock().now()
        now_s = now.nanoseconds * 1.0e-9
        enabled = bool(self.get_parameter("follow_enabled").value)
        decision = self.controller.evaluate(self.latest_fix, now_s, enabled)

        preview = TwistStamped()
        preview.header.stamp = now.to_msg()
        preview.header.frame_id = self.expected_frame
        preview.twist.linear.x = decision.linear_x_mps
        preview.twist.angular.z = decision.angular_z_radps
        self.preview_publisher.publish(preview)
        if decision.state in (FollowState.TRACKING, FollowState.HOLDING):
            self.candidate_publisher.publish(preview)

        status = {
            "state": decision.state.value,
            "reason": decision.reason,
            "linear_x_mps": decision.linear_x_mps,
            "angular_z_radps": decision.angular_z_radps,
            "age_s": decision.age_s,
            "range_m": decision.range_m,
            "bearing_rad": decision.bearing_rad,
            "actuation": False,
        }
        self.status_publisher.publish(String(data=json.dumps(status)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UuvFollowPreviewNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass


if __name__ == "__main__":
    main()
