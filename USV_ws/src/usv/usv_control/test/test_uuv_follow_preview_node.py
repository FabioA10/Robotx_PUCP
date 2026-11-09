"""ROS integration tests for the preview-only follower adapter."""

import json
import time

from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import String

from usv_control.uuv_follow_preview_node import UuvFollowPreviewNode


class PreviewCollector(Node):
    """Collect status and preview messages emitted during a test."""

    def __init__(self, context):
        super().__init__('preview_test_collector', context=context)
        self.previews = []
        self.statuses = []
        self.create_subscription(
            TwistStamped,
            '/usv/follow/cmd_vel_preview',
            self.previews.append,
            10,
        )
        self.create_subscription(
            String,
            '/usv/follow/status',
            self._status_callback,
            10,
        )

    def _status_callback(self, message):
        self.statuses.append(json.loads(message.data))


@pytest.fixture
def ros_context():
    """Provide a fresh ROS context for each adapter integration test."""
    context = rclpy.context.Context()
    rclpy.init(context=context)
    try:
        yield context
    finally:
        rclpy.shutdown(context=context)


def _fix(node, x=2.0, y=0.0):
    message = PoseWithCovarianceStamped()
    message.header.frame_id = 'base_link'
    message.header.stamp = node.get_clock().now().to_msg()
    message.pose.pose.position.x = x
    message.pose.pose.position.y = y
    message.pose.covariance[0] = 0.04
    message.pose.covariance[7] = 0.04
    return message


def _spin_until(executor, predicate, timeout_s=0.5):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.01)
        if predicate():
            return
    raise AssertionError('timed out waiting for a preview publication')


def test_adapter_stops_on_loss_and_requires_reacquisition(ros_context):
    node = UuvFollowPreviewNode(context=ros_context, parameter_overrides=[
        Parameter('follow_enabled', value=True),
        Parameter('acquisition_fixes', value=2),
        Parameter('fix_timeout_s', value=0.05),
        Parameter('publish_rate_hz', value=1.0),
    ])
    collector = PreviewCollector(ros_context)
    executor = SingleThreadedExecutor(context=ros_context)
    executor.add_node(node)
    executor.add_node(collector)

    try:
        node.fix_callback(_fix(node))
        node.control_loop()
        _spin_until(executor, lambda: len(collector.statuses) >= 1)
        assert collector.statuses[-1]['state'] == 'waiting_fix'
        assert collector.previews[-1].twist.linear.x == 0.0

        time.sleep(0.01)
        node.fix_callback(_fix(node))
        node.control_loop()
        _spin_until(executor, lambda: len(collector.statuses) >= 2)
        assert collector.statuses[-1]['state'] == 'tracking'
        assert collector.previews[-1].twist.linear.x > 0.0
        assert collector.statuses[-1]['actuation'] is False

        time.sleep(0.07)
        node.control_loop()
        _spin_until(executor, lambda: len(collector.statuses) >= 3)
        assert collector.statuses[-1]['state'] == 'lost_target'
        assert collector.previews[-1].twist.linear.x == 0.0
        assert collector.previews[-1].twist.angular.z == 0.0

        node.fix_callback(_fix(node))
        node.control_loop()
        _spin_until(executor, lambda: len(collector.statuses) >= 4)
        assert collector.statuses[-1]['state'] == 'waiting_fix'

        time.sleep(0.01)
        node.fix_callback(_fix(node))
        node.control_loop()
        _spin_until(executor, lambda: len(collector.statuses) >= 5)
        assert collector.statuses[-1]['state'] == 'tracking'
    finally:
        executor.remove_node(collector)
        executor.remove_node(node)
        collector.destroy_node()
        node.destroy_node()
        executor.shutdown()


def test_adapter_has_no_physical_command_publishers(ros_context):
    node = UuvFollowPreviewNode(context=ros_context)
    try:
        topics = {publisher.topic_name for publisher in node.publishers}
        assert topics == {
            '/parameter_events',
            '/usv/follow/cmd_vel_preview',
            '/usv/follow/status',
        }
        assert not any('mavros' in topic for topic in topics)
        usbl_subscriptions = [
            subscription
            for subscription in node.subscriptions
            if subscription.topic_name == '/usbl/relative_pose'
        ]
        assert len(usbl_subscriptions) == 1
        assert (
            usbl_subscriptions[0].qos_profile.reliability
            == ReliabilityPolicy.BEST_EFFORT
        )
    finally:
        node.destroy_node()
