"""ROS boundary tests for the non-actuating command shadow node."""

import json
import time

from geometry_msgs.msg import TwistStamped

import pytest

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter

from std_msgs.msg import Bool, String

from usv_control.usv_command_shadow_node import UsvCommandShadowNode


@pytest.fixture
def ros_context():
    """Provide a clean ROS context for every boundary test."""
    context = rclpy.context.Context()
    rclpy.init(context=context)
    try:
        yield context
    finally:
        rclpy.shutdown(context=context)


def test_shadow_node_has_no_physical_publishers(ros_context):
    """The ROS boundary exposes diagnostics and shadow output only."""
    node = UsvCommandShadowNode(context=ros_context)
    try:
        topics = {publisher.topic_name for publisher in node.publishers}
        assert topics == {
            '/parameter_events',
            '/usv/control/arbiter_status',
            '/usv/control/cmd_vel_shadow',
        }
        assert not any('mavros' in topic for topic in topics)
    finally:
        node.destroy_node()


def test_shadow_pipeline_publishes_auditable_non_actuating_status(ros_context):
    """Each shadow cycle reports that physical actuation is disabled."""
    node = UsvCommandShadowNode(context=ros_context, parameter_overrides=[
        Parameter('shadow_enabled', value=True),
        Parameter('publish_rate_hz', value=1.0),
    ])
    collector = Node('shadow_test_collector', context=ros_context)
    outputs = []
    statuses = []
    collector.create_subscription(
        TwistStamped, '/usv/control/cmd_vel_shadow', outputs.append, 10
    )
    collector.create_subscription(
        String,
        '/usv/control/arbiter_status',
        lambda msg: statuses.append(json.loads(msg.data)),
        10,
    )
    executor = SingleThreadedExecutor(context=ros_context)
    executor.add_node(node)
    executor.add_node(collector)
    try:
        candidate = TwistStamped()
        candidate.header.stamp = node.get_clock().now().to_msg()
        candidate.header.frame_id = 'base_link'
        candidate.twist.linear.x = 0.1
        node.candidate_callback(candidate)
        node.connected_callback(Bool(data=True))
        node.killswitch_callback(Bool(data=False))
        node.control_loop()
        node.control_loop()
        deadline = time.monotonic() + 0.5
        while (
            time.monotonic() < deadline
            and not any(item['state'] == 'active' for item in statuses)
        ):
            executor.spin_once(timeout_sec=0.01)
        assert outputs
        assert statuses
        assert statuses[-1]['actuation'] is False
        assert statuses[-1]['mode'] == 'uuv_follow'
        assert statuses[-1]['state'] == 'active'
        assert statuses[-1]['vehicle_state_observed'] is True
        assert statuses[-1]['kill_state_observed'] is True
        assert statuses[-1]['candidate_frame_id'] == 'base_link'
    finally:
        executor.remove_node(collector)
        executor.remove_node(node)
        collector.destroy_node()
        node.destroy_node()
        executor.shutdown()


def test_missing_safety_observations_abort_fail_closed(ros_context):
    """No connection/kill publishers is an observable latched abort."""
    node = UsvCommandShadowNode(context=ros_context, parameter_overrides=[
        Parameter('shadow_enabled', value=True),
        Parameter('assume_vehicle_connected', value=True),
    ])
    collector = Node('missing_gate_collector', context=ros_context)
    statuses = []
    collector.create_subscription(
        String,
        '/usv/control/arbiter_status',
        lambda msg: statuses.append(json.loads(msg.data)),
        10,
    )
    executor = SingleThreadedExecutor(context=ros_context)
    executor.add_node(node)
    executor.add_node(collector)
    try:
        node.control_loop()
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline and not statuses:
            executor.spin_once(timeout_sec=0.01)
        assert statuses[-1]['state'] == 'aborted'
        assert statuses[-1]['vehicle_state_observed'] is False
        assert 'not been observed' in statuses[-1]['reason']
    finally:
        executor.remove_node(collector)
        executor.remove_node(node)
        collector.destroy_node()
        node.destroy_node()
        executor.shutdown()


def test_invalid_mode_is_reported_and_latched(ros_context):
    """An invalid mode parameter cannot masquerade as disabled."""
    node = UsvCommandShadowNode(context=ros_context, parameter_overrides=[
        Parameter('shadow_enabled', value=True),
        Parameter('requested_mode', value='uuv_folow'),
    ])
    try:
        node.control_loop()
        assert node.arbiter._abort_latched is True
        assert node.invalid_requested_mode == 'uuv_folow'
    finally:
        node.destroy_node()
