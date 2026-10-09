"""Transport-state tests; all ROS and MAVLink endpoints are in-memory fakes."""

import json
import sys
import types
import unittest
from unittest.mock import patch


class String:
    """Minimal std_msgs/String replacement."""

    def __init__(self, data=''):
        self.data = data


class Publisher:
    """Collect status payloads."""

    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(json.loads(message.data))


fake_mavlink = types.SimpleNamespace(MAV_PARAM_TYPE_REAL32=9)
fake_mavutil = types.SimpleNamespace(mavlink=fake_mavlink)
fake_pymavlink = types.ModuleType('pymavlink')
fake_pymavlink.mavutil = fake_mavutil
fake_std_messages = types.ModuleType('std_msgs.msg')
fake_std_messages.String = String

with patch.dict(
    sys.modules,
    {
        'pymavlink': fake_pymavlink,
        'std_msgs': types.ModuleType('std_msgs'),
        'std_msgs.msg': fake_std_messages,
    },
):
    from uuv_mavlink.parameter_manager import AutopilotParameterManager


class Clock:
    """Controllable monotonic clock."""

    def __init__(self):
        self.value = 10.0

    def __call__(self):
        return self.value


class Mav:
    """Collect parameter requests and writes."""

    def __init__(self):
        self.reads = []
        self.writes = []

    def param_request_read_send(self, *args):
        self.reads.append(args)

    def param_set_send(self, *args):
        self.writes.append(args)


class Bridge:
    """Minimal safe bridge state."""

    def __init__(self, clock):
        self.parameters = {'enable_parameter_write': True}
        self.connected = True
        self.armed = False
        self.mode = 'MANUAL'
        self.deadman = False
        self.last_heartbeat_time = clock()
        self.heartbeat_timeout = 3.0
        self.target_system_id = 1
        self.target_component_id = 1
        self.guided = types.SimpleNamespace(
            owner=False,
            holding_completed_goal=False,
            guided_request_at=None,
            abort_mode=None,
            version=[4, 5, 7],
        )
        self.master = types.SimpleNamespace(mav=Mav())
        self.publisher = Publisher()

    def declare_parameter(self, name, value):
        self.parameters.setdefault(name, value)

    def get_parameter(self, name):
        return types.SimpleNamespace(value=self.parameters[name])

    def create_publisher(self, *_args):
        return self.publisher

    def create_subscription(self, *args):
        return args

    def create_timer(self, *args):
        return args


class Packet:
    """Minimal PARAM_VALUE message."""

    def __init__(self, name, value, param_type=9):
        self.param_id = name.encode('ascii')
        self.param_value = value
        self.param_type = param_type

    def get_type(self):
        return 'PARAM_VALUE'


OLD_VALUES = {
    'MOT_PWM_MIN': 1300.0,
    'MOT_PWM_MAX': 1700.0,
    'WPNAV_SPEED_UP': 18.0,
    'WPNAV_SPEED_DN': 18.0,
    'WPNAV_ACCEL_Z': 50.0,
}

NEW_VALUES = {
    'MOT_PWM_MIN': 1350.0,
    'MOT_PWM_MAX': 1650.0,
    'WPNAV_SPEED_UP': 10.0,
    'WPNAV_SPEED_DN': 10.0,
    'WPNAV_ACCEL_Z': 50.0,
}


class ParameterManagerTests(unittest.TestCase):
    """Verify snapshot, sequential confirmation and final independent readback."""

    def setUp(self):
        self.clock = Clock()
        self.bridge = Bridge(self.clock)
        self.manager = AutopilotParameterManager(self.bridge, clock=self.clock)
        self.manager.tick()

    def request(self, action='apply', values=NEW_VALUES, vehicle=None):
        packet = {'action': action, 'request_id': 'test:1', 'values': values}
        if vehicle is not None:
            packet['vehicle'] = vehicle
        self.manager.receive_request(
            String(json.dumps(packet))
        )

    def observe_set(self, values):
        for name, value in values.items():
            self.manager.observe(Packet(name, value))

    def advance(self, seconds):
        self.clock.value += seconds
        self.bridge.last_heartbeat_time = self.clock()

    def test_success_keeps_previous_values_as_restore_snapshot(self):
        self.request()
        self.observe_set(OLD_VALUES)
        self.manager.tick()
        self.assertEqual(len(self.bridge.master.mav.writes), 1)
        self.observe_set(NEW_VALUES)
        self.assertEqual(self.manager.transaction['phase'], 'verify_apply')
        self.observe_set(NEW_VALUES)
        self.manager.tick()
        self.assertIsNone(self.manager.transaction)
        self.assertEqual(self.manager.last_result, 'applied')
        self.assertEqual(self.manager.last_snapshot, OLD_VALUES)
        self.assertEqual(self.bridge.master.mav.writes[0][-1], 9)

    def test_armed_vehicle_rejects_without_any_write(self):
        self.bridge.armed = True
        self.request()
        self.assertEqual(self.manager.last_result, 'rejected')
        self.assertEqual(self.bridge.master.mav.writes, [])

    def test_unconfirmed_firmware_rejects_without_any_write(self):
        self.bridge.guided.version = None
        self.request()
        self.assertEqual(self.manager.last_result, 'rejected')
        self.assertIn('4.5.7', self.manager.note)
        self.assertEqual(self.bridge.master.mav.writes, [])

    def test_missing_confirmation_rolls_back_and_verifies_snapshot(self):
        self.request()
        self.observe_set(OLD_VALUES)
        self.manager.tick()
        for _ in range(3):
            self.advance(1.1)
            self.manager.tick()
        self.assertEqual(self.manager.transaction['phase'], 'rollback')
        self.observe_set(OLD_VALUES)
        self.assertEqual(self.manager.transaction['phase'], 'verify_rollback')
        self.observe_set(OLD_VALUES)
        self.manager.tick()
        self.assertIsNone(self.manager.transaction)
        self.assertEqual(self.manager.last_result, 'rolled_back')

    def test_explicit_restore_requires_matching_vehicle_and_firmware(self):
        self.request('restore', OLD_VALUES, vehicle={
            'system_id': 1,
            'component_id': 1,
            'firmware': 'ArduSub 4.5.8',
        })
        self.assertEqual(self.manager.last_result, 'rejected')
        self.assertIn('no pertenece', self.manager.note)
        self.request('restore', OLD_VALUES, vehicle={
            'system_id': 1,
            'component_id': 1,
            'firmware': 'ArduSub 4.5.7',
        })
        self.assertEqual(self.manager.last_result, 'reading_snapshot')

    def test_read_only_electrical_parameter_is_published_but_never_written(self):
        self.advance(0.6)
        self.manager.observe(Packet('MOT_BAT_CURR_MAX', 35.0, param_type=9))
        payload = self.bridge.publisher.messages[-1]
        self.assertEqual(payload['read_only_values']['MOT_BAT_CURR_MAX'], 35.0)
        self.assertEqual(self.bridge.master.mav.writes, [])


if __name__ == '__main__':
    unittest.main()
