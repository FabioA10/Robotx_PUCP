#!/usr/bin/env python3
"""ROS adapter for command arbitration that intentionally cannot actuate."""

import json

from geometry_msgs.msg import TwistStamped

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, String

from usv_control.core import (
    ArbiterConfig,
    ArbiterMode,
    CandidateCommand,
    ShadowCommandArbiter,
)


class UsvCommandShadowNode(Node):
    """Audit a candidate command and publish only a non-physical shadow."""

    def __init__(self, *, context=None, parameter_overrides=None) -> None:
        """Create subscriptions and shadow-only publishers."""
        super().__init__(
            'usv_command_shadow_node',
            context=context,
            parameter_overrides=parameter_overrides,
        )
        self.declare_parameter('shadow_enabled', False)
        self.declare_parameter('requested_mode', 'uuv_follow')
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('command_timeout_s', 0.30)
        self.declare_parameter('safety_gate_timeout_s', 0.50)
        self.declare_parameter('max_clock_step_s', 1.00)
        self.declare_parameter('max_linear_speed_mps', 0.20)
        self.declare_parameter('max_angular_speed_radps', 0.30)
        self.declare_parameter('max_linear_acceleration_mps2', 0.20)
        self.declare_parameter('max_angular_acceleration_radps2', 0.50)
        self.declare_parameter('assume_vehicle_connected', False)
        self.declare_parameter('expected_candidate_frame_id', 'base_link')
        self.declare_parameter('candidate_source', 'uuv_follow')

        self.arbiter = ShadowCommandArbiter(ArbiterConfig(
            command_timeout_s=float(
                self.get_parameter('command_timeout_s').value
            ),
            safety_gate_timeout_s=float(
                self.get_parameter('safety_gate_timeout_s').value
            ),
            max_clock_step_s=float(
                self.get_parameter('max_clock_step_s').value
            ),
            max_linear_speed_mps=float(
                self.get_parameter('max_linear_speed_mps').value
            ),
            max_angular_speed_radps=float(
                self.get_parameter('max_angular_speed_radps').value
            ),
            max_linear_acceleration_mps2=float(
                self.get_parameter('max_linear_acceleration_mps2').value
            ),
            max_angular_acceleration_radps2=float(
                self.get_parameter('max_angular_acceleration_radps2').value
            ),
            expected_frame_id=str(
                self.get_parameter('expected_candidate_frame_id').value
            ),
        ))
        self.latest_candidate = None
        self.vehicle_connected = False
        self.kill_active = False
        self.vehicle_state_timestamp_s = None
        self.kill_state_timestamp_s = None
        self.invalid_requested_mode = None

        if bool(self.get_parameter('assume_vehicle_connected').value):
            self.get_logger().warning(
                'assume_vehicle_connected is ignored: a fresh /usv/connected '
                'observation is required.'
            )

        self.create_subscription(
            TwistStamped,
            '/usv/follow/cmd_vel_candidate',
            self.candidate_callback,
            10,
        )
        self.create_subscription(
            Bool, '/usv/connected', self.connected_callback, 10
        )
        self.create_subscription(
            Bool, '/usv/control/killswitch', self.killswitch_callback, 10
        )
        self.shadow_publisher = self.create_publisher(
            TwistStamped, '/usv/control/cmd_vel_shadow', 10
        )
        self.status_publisher = self.create_publisher(
            String, '/usv/control/arbiter_status', 10
        )
        rate = max(0.1, float(self.get_parameter('publish_rate_hz').value))
        self.create_timer(1.0 / rate, self.control_loop)
        self.get_logger().warning(
            'Shadow only: this node has no MAVROS or actuator publisher.'
        )

    def candidate_callback(self, message: TwistStamped) -> None:
        """Store the latest timestamped follower candidate."""
        stamp = message.header.stamp
        timestamp_s = float(stamp.sec) + float(stamp.nanosec) * 1.0e-9
        self.latest_candidate = CandidateCommand(
            linear_x_mps=float(message.twist.linear.x),
            angular_z_radps=float(message.twist.angular.z),
            timestamp_s=timestamp_s,
            source=str(self.get_parameter('candidate_source').value),
            frame_id=str(message.header.frame_id),
        )

    def connected_callback(self, message: Bool) -> None:
        """Update the observed vehicle connection gate."""
        self.vehicle_connected = bool(message.data)
        self.vehicle_state_timestamp_s = (
            self.get_clock().now().nanoseconds * 1.0e-9
        )

    def killswitch_callback(self, message: Bool) -> None:
        """Update the independent kill gate."""
        self.kill_active = bool(message.data)
        self.kill_state_timestamp_s = (
            self.get_clock().now().nanoseconds * 1.0e-9
        )

    def control_loop(self) -> None:
        """Evaluate and publish one auditable shadow command."""
        now = self.get_clock().now()
        now_s = now.nanoseconds * 1.0e-9
        try:
            requested_mode_text = str(
                self.get_parameter('requested_mode').value
            ).lower()
            requested_mode = ArbiterMode(requested_mode_text)
            self.invalid_requested_mode = None
        except ValueError:
            requested_mode = None
            self.invalid_requested_mode = requested_mode_text
        decision = self.arbiter.evaluate(
            self.latest_candidate,
            now_s,
            requested_mode,
            bool(self.get_parameter('shadow_enabled').value),
            self.vehicle_connected,
            self.kill_active,
            self.vehicle_state_timestamp_s,
            self.kill_state_timestamp_s,
        )

        def observation_age(timestamp_s):
            if timestamp_s is None:
                return None
            return max(0.0, now_s - timestamp_s)

        output = TwistStamped()
        output.header.stamp = now.to_msg()
        output.header.frame_id = self.arbiter.config.expected_frame_id
        output.twist.linear.x = decision.linear_x_mps
        output.twist.angular.z = decision.angular_z_radps
        self.shadow_publisher.publish(output)
        self.status_publisher.publish(String(data=json.dumps({
            'state': decision.state.value,
            'mode': decision.mode.value,
            'reason': decision.reason,
            'linear_x_mps': decision.linear_x_mps,
            'angular_z_radps': decision.angular_z_radps,
            'input_age_s': decision.input_age_s,
            'candidate_present': self.latest_candidate is not None,
            'candidate_source': (
                self.latest_candidate.source if self.latest_candidate else None
            ),
            'candidate_frame_id': (
                self.latest_candidate.frame_id if self.latest_candidate else None
            ),
            'candidate_timestamp_s': (
                self.latest_candidate.timestamp_s
                if self.latest_candidate else None
            ),
            'candidate_age_s': (
                observation_age(self.latest_candidate.timestamp_s)
                if self.latest_candidate else None
            ),
            'expected_candidate_source': self.arbiter.config.expected_source,
            'expected_candidate_frame_id': (
                self.arbiter.config.expected_frame_id
            ),
            'vehicle_connected': self.vehicle_connected,
            'vehicle_state_observed': (
                self.vehicle_state_timestamp_s is not None
            ),
            'vehicle_state_age_s': observation_age(
                self.vehicle_state_timestamp_s
            ),
            'kill_active': self.kill_active,
            'kill_state_observed': self.kill_state_timestamp_s is not None,
            'kill_state_age_s': observation_age(self.kill_state_timestamp_s),
            'invalid_requested_mode': self.invalid_requested_mode,
            'abort_latched': decision.abort_latched,
            'actuation': False,
        })))


def main(args=None) -> None:
    """Run the non-actuating ROS adapter."""
    rclpy.init(args=args)
    node = UsvCommandShadowNode()
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


if __name__ == '__main__':
    main()
