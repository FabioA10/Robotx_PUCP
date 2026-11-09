"""Small in-process subset of rclpy/std_msgs used by the V6 controller."""

import logging
import sys
import time
import types


class String:
    def __init__(self):
        self.data = ''


class Parameter:
    def __init__(self, value):
        self.value = value


class Timer:
    def __init__(self, period, callback):
        self.period = float(period)
        self.callback = callback
        self.next_call = time.monotonic() + self.period


class Publisher:
    def __init__(self, bus, topic):
        self.bus = bus
        self.topic = topic

    def publish(self, message):
        self.bus.publish(self.topic, message)


class Bus:
    def __init__(self):
        self.subscriptions = {}
        self.timers = []

    def subscribe(self, topic, callback):
        self.subscriptions.setdefault(topic, []).append(callback)

    def publish(self, topic, message):
        for callback in tuple(self.subscriptions.get(topic, ())):
            callback(message)

    def timer(self, period, callback):
        timer = Timer(period, callback)
        self.timers.append(timer)
        return timer

    def spin_once(self, now=None):
        current = time.monotonic() if now is None else float(now)
        for timer in tuple(self.timers):
            if current >= timer.next_call:
                timer.next_call = current + timer.period
                timer.callback()


BUS = Bus()
PARAMETER_OVERRIDES = {}


class Node:
    def __init__(self, name):
        self._name = name
        self._parameters = dict(PARAMETER_OVERRIDES.get(name, {}))
        self._logger = logging.getLogger(name)

    def get_name(self):
        return self._name

    def declare_parameter(self, name, default):
        self._parameters.setdefault(name, default)
        return Parameter(self._parameters[name])

    def get_parameter(self, name):
        return Parameter(self._parameters[name])

    def create_publisher(self, _message_type, topic, _depth):
        return Publisher(BUS, topic)

    def create_subscription(self, _message_type, topic, callback, _depth):
        BUS.subscribe(topic, callback)
        return callback

    def create_timer(self, period, callback):
        return BUS.timer(period, callback)

    def get_logger(self):
        return self._logger

    def destroy_node(self):
        pass


def install_ros_shims():
    """Make the hardware-independent ROS nodes importable on Debian/armv7."""
    rclpy = types.ModuleType('rclpy')
    rclpy.init = lambda *args, **kwargs: None
    rclpy.spin = lambda *args, **kwargs: None
    rclpy.ok = lambda: False
    rclpy.shutdown = lambda: None
    rclpy_node = types.ModuleType('rclpy.node')
    rclpy_node.Node = Node
    std_msgs = types.ModuleType('std_msgs')
    std_msgs_msg = types.ModuleType('std_msgs.msg')
    std_msgs_msg.String = String
    std_msgs.msg = std_msgs_msg
    sys.modules.setdefault('rclpy', rclpy)
    sys.modules.setdefault('rclpy.node', rclpy_node)
    sys.modules.setdefault('std_msgs', std_msgs)
    sys.modules.setdefault('std_msgs.msg', std_msgs_msg)
