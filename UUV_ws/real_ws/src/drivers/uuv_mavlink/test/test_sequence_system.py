"""Fault-injection unit tests. ROS and MAVLink transport are mocked, never hardware."""

import json
import math
import sys
import types
import unittest
from unittest.mock import patch

from uuv_mavlink.sequence_core import Limits, Pose, Sequence, parse_steps, return_steps


class String:
    def __init__(self, data=''):
        self.data = data


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg.data)


class FakeNode:
    def __init__(self, *args):
        self.parameters = {}
        self.publishers = {}
        self.errors = []

    def declare_parameter(self, name, value):
        self.parameters.setdefault(name, value)

    def get_parameter(self, name):
        return types.SimpleNamespace(value=self.parameters[name])

    def create_publisher(self, _, topic, __):
        pub = Publisher()
        self.publishers[topic] = pub
        return pub

    def create_subscription(self, *args):
        return args

    def create_timer(self, *args):
        return types.SimpleNamespace(cancel=lambda: None)

    def get_logger(self):
        return types.SimpleNamespace(error=self.errors.append)


fake_ros = types.ModuleType('rclpy')
fake_node = types.ModuleType('rclpy.node')
fake_node.Node = FakeNode
fake_std = types.ModuleType('std_msgs.msg')
fake_std.String = String
with patch.dict(sys.modules, {'rclpy': fake_ros, 'rclpy.node': fake_node,
                              'std_msgs': types.ModuleType('std_msgs'), 'std_msgs.msg': fake_std}):
    from uuv_mavlink.guided_bridge import GuidedBridge
    from uuv_mavlink.sequence_executor_node import SequenceExecutor


class Mav:
    def __init__(self):
        self.sent = []

    def __getattr__(self, name):
        return lambda *args: self.sent.append((name, args))


class FakeBridge(FakeNode):
    def __init__(self, enabled=True):
        super().__init__()
        self.parameters.update(sequence_monitor_enabled=True, enable_sequence_output=enabled, dvl_rest_base_url='')
        self.enable_command_output = enabled
        self.require_manual_mode = True
        self.connected = True
        self.armed = True
        self.mode = 'GUIDED'
        self.mode_name_by_id = {19: 'MANUAL', 2: 'ALT_HOLD', 16: 'POSHOLD', 4: 'GUIDED'}
        self.motors_enabled = True
        self.deadman = False
        self.last_cmd_time = 10.0
        self.last_heartbeat_time = 10.0
        self.target_system_id = 1
        self.target_component_id = 1
        self.source_system_id = 255
        self.master = types.SimpleNamespace(mav=Mav())


class Packet(types.SimpleNamespace):
    def get_type(self):
        return self.kind

    def get_srcSystem(self):
        return getattr(self, 'sysid', 1)

    def get_srcComponent(self):
        return getattr(self, 'compid', 1)


class TestSequenceCore(unittest.TestCase):
    def test_absolute_depth_and_relative_turn_from_new_pose(self):
        core = Sequence(Limits(max_depth=3, radius=5))
        core.start(Pose(10, 20, 1, math.pi / 2), [('advance', 1), ('depth', 2)], 0)
        self.assertAlmostEqual(core.goal.n, 10)
        self.assertAlmostEqual(core.goal.e, 21)
        core.cancel('manual')
        core.start(Pose(2, 3, 1, 0), [('depth', 2)], 1)
        self.assertEqual(core.goal.depth, 2)
        self.assertEqual(core.goal.n, 2)

    def test_invalid_inputs_and_limits(self):
        for steps in ([], [('turn', 360)], [('wait', -1)], [('advance', math.nan)], [('depth', True)]):
            with self.assertRaises(ValueError):
                parse_steps(steps)
        core = Sequence()
        for steps in ([('advance', 3)], [('depth', 3)], [('advance', 1.5), ('advance', 1.5)]):
            with self.assertRaises(ValueError):
                core.start(Pose(0, 0, 0, 0), steps, 0)

    def test_complete_sequence_in_ideal_kinematic_plant(self):
        core = Sequence()
        pose = Pose(0, 0, 0.2, math.radians(179))
        core.start(pose, [('depth', 0.5), ('turn', 30), ('advance', 0.5), ('wait', 1)], 0)
        speed = 0
        for tick in range(1, 2500):
            cmd = core.update(pose, tick * 0.05, speed=speed)
            if core.state != 'RUNNING':
                break
            vn, ve, vd, yaw = cmd
            speed = math.sqrt(vn*vn + ve*ve + vd*vd)
            self.assertLessEqual(math.hypot(vn, ve), 0.200001)
            self.assertLessEqual(abs(vd), 0.100001)
            pose = Pose(pose.n + vn * 0.05, pose.e + ve * 0.05, pose.depth + vd * 0.05, yaw)
        self.assertEqual(core.state, 'COMPLETED', core.reason)

    def test_wait_requires_staying_inside_tolerances(self):
        core = Sequence()
        origin = Pose(0, 0, 0.2, 0)
        core.start(origin, [('wait', 2)], 0)
        for i in range(1, 31):
            core.update(origin, i * 0.05)
        core.update(Pose(0.5, 0, 0.2, 0), 1.55)
        for i in range(32, 70):
            core.update(origin, i * 0.05)
        self.assertEqual(core.state, 'RUNNING')
        for i in range(70, 76):
            core.update(origin, i * 0.05)
        self.assertEqual(core.state, 'COMPLETED')

    def test_timeout_and_loop_stall_cancel_without_resuming(self):
        for delay in (0.7, 2.0):
            core = Sequence()
            pose = Pose(0, 0, 0, 0)
            core.start(pose, [('advance', 1)], 0)
            self.assertIsNone(core.update(pose, delay))
            self.assertEqual(core.state, 'CANCELLED')
            self.assertEqual(core.steps, [])
            self.assertIsNone(core.update(pose, delay + 0.05))
        core = Sequence(Limits(step_timeout=1))
        core.start(pose, [('advance', 1)], 0)
        for i in range(1, 25):
            core.update(pose, i * 0.05)
        self.assertEqual(core.state, 'CANCELLED')

    def test_home_has_fixed_endpoint_despite_heading_error(self):
        home = Pose(0, 0, 0.5, math.radians(-179))
        start = Pose(1, 1, 0.5, math.radians(179))
        core = Sequence()
        core.start(start, return_steps(start, home), 0, home, returning=True)
        core.index = 2
        core.begin_step(Pose(1.05, 1.03, 0.51, -2.3), 1)
        self.assertEqual((core.goal.n, core.goal.e, core.goal.depth), (0, 0, 0.5))


class TestGuidedTransport(unittest.TestCase):
    def setUp(self):
        self.now = 10.0
        self.clock_patch = patch('time.monotonic', lambda: self.now)
        self.clock_patch.start()
        self.addCleanup(self.clock_patch.stop)
        self.b = FakeBridge()
        self.g = GuidedBridge(self.b, clock=lambda: self.now)
        self.refresh()
        self.g.tick()

    def refresh(self):
        self.b.last_heartbeat_time = self.now
        self.b.last_cmd_time = self.now
        for key, value in dict(position=[0, 0, 0], velocity=[0, 0, 0], yaw=0,
                               pressure=1000, ekf=47, dvl=90, voltage=16).items():
            self.g.put(key, value)
        self.g.version = [4, 5, 7]
        self.g.params = {k: (v, self.now) for k, v in dict(FS_PILOT_INPUT=2, FS_PILOT_TIMEOUT=3, SYSID_MYGCS=255).items()}

    def command(self, **overrides):
        status = json.loads(self.g.status_pub.messages[-1])
        packet = dict(session=status['session'], permit=status['permit'], ticket=status['ticket'],
                      run='run1', serial=0, command=[0.1, 0, 0, 0])
        packet.update(overrides)
        return String(json.dumps(packet))

    def test_disabled_output_never_sends_motion_arm_or_modes(self):
        b = FakeBridge(enabled=False)
        g = GuidedBridge(b, clock=lambda: self.now)
        g.tick()
        g.receive(self.command())
        g.tick()
        names = [name for name, _ in b.master.mav.sent]
        self.assertNotIn('set_position_target_local_ned_send', names)
        self.assertNotIn('set_mode_send', names)
        self.assertNotIn('manual_control_send', names)

    def test_velocity_frame_mask_and_values(self):
        self.g.receive(self.command())
        self.g.tick()
        packets = [a for n, a in self.b.master.mav.sent if n == 'set_position_target_local_ned_send']
        self.assertEqual(packets[-1][1:5], (1, 1, 1, 2503))
        self.assertEqual(packets[-1][8:11], (0.1, 0.0, 0.0))

    def test_dead_executor_stops_and_old_packet_cannot_restart(self):
        old = self.command()
        self.g.receive(old)
        self.now += 0.45
        self.refresh()
        self.g.tick()
        self.assertIsNone(self.g.owner)
        self.assertIsNotNone(self.g.abort_mode)
        self.g.receive(old)
        self.assertIsNone(self.g.owner)
        packets = [a for n, a in self.b.master.mav.sent if n == 'set_position_target_local_ned_send']
        self.assertEqual(packets[-1][8:11], (0.0, 0.0, 0.0))

    def test_only_one_run_and_increasing_serial(self):
        self.g.receive(self.command())
        self.g.receive(self.command(run='other', serial=100, command=[0.2, 0, 0, 0]))
        self.assertEqual(self.g.owner, 'run1')
        self.assertEqual(self.g.last_command[0], 0.1)
        self.g.receive(self.command(serial=0, command=[0.2, 0, 0, 0]))
        self.assertEqual(self.g.last_command[0], 0.1)

    def test_stale_telemetry_dvl_and_manual_takeover(self):
        for missing in ('position', 'pressure', 'dvl', 'ekf'):
            self.refresh()
            self.g.abort_mode = None
            self.g.tick()
            self.g.receive(self.command())
            del self.g.samples[missing]
            self.g.tick()
            self.assertIsNone(self.g.owner, missing)
        self.refresh()
        self.g.abort_mode = None
        self.g.tick()
        self.g.receive(self.command())
        self.b.deadman = True
        self.g.tick()
        self.assertIsNone(self.g.owner)
        self.assertEqual(self.g.abort_mode, 'MANUAL')

    def test_real_manual_mode_cancels_and_never_restarts(self):
        old = self.command()
        self.g.receive(old)
        self.b.mode = 'MANUAL'
        self.g.tick()
        self.assertIsNone(self.g.owner)
        self.b.mode = 'GUIDED'
        self.g.tick()
        self.g.receive(old)
        self.assertIsNone(self.g.owner)

    def test_invalid_command_and_version_are_rejected(self):
        self.g.receive(self.command(command=[float('nan'), 0, 0, 0]))
        self.assertIsNone(self.g.owner)
        self.g.receive(self.command(command=[1, 0, 0, 0]))
        self.assertIsNone(self.g.owner)
        self.g.version = [4, 5, 6]
        self.g.receive(self.command())
        self.assertIsNone(self.g.owner)

    def test_duplicate_sensor_time_does_not_refresh_cached_data(self):
        packet = Packet(kind='VISION_POSITION_DELTA', sysid=255, compid=0, confidence=90,
                        position_delta=[0, 0, 0], angle_delta=[0, 0, 0], time_usec=100000, time_delta_usec=100000)
        self.g.observe(packet)
        self.now += 0.9
        self.g.observe(packet)
        self.assertIsNone(self.g.get('dvl'))

    def test_wrong_dvl_source_ignored_and_reboot_changes_epoch(self):
        self.g.samples.pop('dvl')
        self.g.observe(Packet(kind='VISION_POSITION_DELTA', sysid=99, compid=0, confidence=100))
        self.assertIsNone(self.g.get('dvl'))
        self.g.put('yaw', 0, 5000)
        epoch = self.g.nav_epoch
        self.g.put('yaw', 0, 1)
        self.assertGreater(self.g.nav_epoch, epoch)


    def test_zero_dvl_timestamp_refreshes_arrival_and_expires_on_silence(self):
        packet = Packet(kind='VISION_POSITION_DELTA', sysid=255, compid=0,
                        confidence=99.7, position_delta=[0.0001, 0, 0],
                        angle_delta=[0, 0, 0], time_usec=0, time_delta_usec=78440)
        epoch = self.g.nav_epoch
        # Identical deltas can legitimately occur while the ROV is stationary.
        for _ in range(25):
            self.now += 0.08
            self.g.observe(packet)
            self.assertEqual(self.g.get('dvl'), 99.7)
        self.assertEqual(self.g.nav_epoch, epoch)
        self.now += 0.79
        self.assertEqual(self.g.get('dvl'), 99.7)
        self.now += 0.02
        self.assertIsNone(self.g.get('dvl'))
        self.assertIn('Sin datos recientes: dvl', self.g.health())

    def test_zero_timestamp_still_requires_correct_source_and_valid_fields(self):
        self.g.samples.pop('dvl')
        fields = dict(kind='VISION_POSITION_DELTA', sysid=255, compid=0,
                      confidence=99.7, position_delta=[0, 0, 0],
                      angle_delta=[0, 0, 0], time_usec=0, time_delta_usec=75000)
        for change in ({'sysid': 99}, {'compid': 1}, {'confidence': float('nan')},
                       {'time_delta_usec': 0}, {'time_usec': -1}):
            self.g.observe(Packet(**dict(fields, **change)))
            self.assertIsNone(self.g.get('dvl'), change)
        self.g.observe(Packet(**dict(fields, confidence=20)))
        self.assertIn('Confianza del mensaje DVL insuficiente', self.g.health())

    def test_positive_dvl_timestamp_checks_survive_zero_timestamp_packets(self):
        fields = dict(kind='VISION_POSITION_DELTA', sysid=255, compid=0,
                      confidence=90, position_delta=[0, 0, 0],
                      angle_delta=[0, 0, 0], time_delta_usec=75000)
        self.g.observe(Packet(**fields, time_usec=100000))
        self.now += 0.1
        self.g.observe(Packet(**fields, time_usec=0))
        self.now += 0.81
        self.g.observe(Packet(**fields, time_usec=100000))
        self.assertIsNone(self.g.get('dvl'))
        self.g.observe(Packet(**fields, time_usec=175000))
        self.assertEqual(self.g.get('dvl'), 90)
        epoch = self.g.nav_epoch
        self.g.observe(Packet(**fields, time_usec=1))
        self.assertGreater(self.g.nav_epoch, epoch)

    @staticmethod
    def rest_dvl(counter, confidence=99.7, updated=None, **changes):
        message = dict(type='VISION_POSITION_DELTA', confidence=confidence,
                       position_delta=[0.0001, 0, 0], angle_delta=[0, 0, 0],
                       time_delta_usec=80000, time_usec=0)
        message.update(changes)
        return {'message': message,
                'status': {'time': {'counter': counter,
                                    'last_update': updated or f'update-{counter}'}}}

    def test_rest_dvl_needs_advancing_counter_and_expires(self):
        self.g.samples.pop('dvl')
        self.g.observe_rest_dvl(self.rest_dvl(100))
        self.assertIsNone(self.g.get('dvl'))
        self.g.observe_rest_dvl(self.rest_dvl(101))
        self.assertEqual(self.g.get('dvl'), 99.7)
        self.assertEqual(self.g.dvl_input, 'mavlink2rest')
        self.now += 0.81
        self.g.observe_rest_dvl(self.rest_dvl(101))
        self.assertIsNone(self.g.get('dvl'))
        self.g.observe_rest_dvl(self.rest_dvl(102))
        self.assertEqual(self.g.get('dvl'), 99.7)

    def test_rest_dvl_rejects_bad_data_and_invalidates_on_counter_reset(self):
        self.g.samples.pop('dvl')
        self.g.observe_rest_dvl(self.rest_dvl(200))
        self.g.observe_rest_dvl(self.rest_dvl(201, confidence=float('nan')))
        self.assertIsNone(self.g.get('dvl'))
        self.g.observe_rest_dvl(self.rest_dvl(202, time_delta_usec=0))
        self.assertIsNone(self.g.get('dvl'))
        self.g.observe_rest_dvl(self.rest_dvl(203))
        self.assertEqual(self.g.get('dvl'), 99.7)
        epoch = self.g.nav_epoch
        self.g.observe_rest_dvl(self.rest_dvl(1))
        self.assertGreater(self.g.nav_epoch, epoch)
        self.assertIsNone(self.g.get('dvl'))
        self.g.observe_rest_dvl(self.rest_dvl(2))
        self.assertEqual(self.g.get('dvl'), 99.7)


class TestExecutor(unittest.TestCase):
    def setUp(self):
        self.now = 100.0
        clock = patch('time.monotonic', lambda: self.now)
        clock.start()
        self.addCleanup(clock.stop)
        self.ex = SequenceExecutor()
        self.ui_counter = 0
        self.action_counter = 0
        self.f = dict(session='bridge', nav_epoch=0, permit='permit', ticket='bridge-ticket',
                      samples=dict(position=[0, 0, 0], velocity=[0, 0, 0], pressure=1000, yaw=0, dvl=90),
                      health=[], blockers=['Salida real deshabilitada'], mode='MANUAL', armed=False,
                      enabled=False, reason='test')
        for _ in range(45):
            self.advance()

    def advance(self):
        self.now += 0.05
        self.ex.on_feedback(String(json.dumps(self.f)))
        self.ex.tick()

    def req(self, action, **extra):
        s = json.loads(self.ex.state_pub.messages[-1])
        packet = dict(session=s['session'], ticket=s['ticket'], ui='ui', serial=self.action_counter,
                      action=action, sensor_depth=0.2, limits_confirmed=True, max_depth=1, radius=2,
                      steps=[['advance', 0.5]])
        self.action_counter += 1
        packet.update(extra)
        self.ex.on_request(String(json.dumps(packet)))

    def ui(self):
        s = json.loads(self.ex.state_pub.messages[-1])
        self.ex.on_ui(String(json.dumps(dict(session=s['session'], ticket=s['ticket'], ui='ui', serial=self.ui_counter))))
        self.ui_counter += 1

    def start(self):
        self.req('surface')
        self.assertIsNotNone(self.ex.reference, self.ex.note)
        self.req('home')
        self.req('validate')
        self.assertIsNotNone(self.ex.validated, self.ex.note)
        self.f.update(armed=True, mode='GUIDED', enabled=True, blockers=[])
        self.advance()
        self.ui()
        self.req('execute')
        self.assertEqual(self.ex.core.state, 'RUNNING', self.ex.note)

    def test_disabled_output_blocks_execution_after_validation(self):
        self.req('surface')
        self.req('validate')
        self.req('execute')
        self.assertNotEqual(self.ex.core.state, 'RUNNING')
        self.assertEqual(self.ex.command_pub.messages, [])

    def test_pressure_reference_depth_and_guard_against_ekf_reset(self):
        self.req('surface', sensor_depth=0.2)
        self.f['samples']['pressure'] += 1000 * 9.80665 * 0.3 / 100
        self.f['samples']['position'][2] = 0.3
        self.advance()
        self.assertAlmostEqual(self.ex.pose().depth, 0.5)
        self.f['samples']['position'][2] = 2
        self.advance()
        self.assertIsNone(self.ex.reference)

    def test_ui_loss_cancels_without_auto_resume(self):
        self.start()
        self.advance()
        self.assertTrue(self.ex.command_pub.messages)
        for _ in range(12):
            self.advance()
        self.assertEqual(self.ex.core.state, 'CANCELLED')
        self.assertIsNone(self.ex.validated)
        self.ui()
        self.advance()
        self.assertEqual(self.ex.core.state, 'CANCELLED')

    def test_manual_cancels_and_discards_goals(self):
        self.start()
        self.f['mode'] = 'MANUAL'
        self.advance()
        self.assertEqual(self.ex.core.state, 'CANCELLED')
        self.assertEqual(self.ex.core.steps, [])

    def test_nav_loss_invalidates_surface_home_and_plan(self):
        self.start()
        self.f['health'] = ['DVL sin datos']
        self.advance()
        self.assertIsNone(self.ex.reference)
        self.assertIsNone(self.ex.home)
        self.assertIsNone(self.ex.validated)
        self.assertEqual(self.ex.core.state, 'CANCELLED')

    def test_edit_after_validation_and_duplicate_execute_are_rejected(self):
        self.req('surface')
        self.req('validate')
        self.f.update(armed=True, mode='GUIDED', enabled=True, blockers=[])
        self.advance()
        self.ui()
        self.req('execute', steps=[['advance', 1]])
        self.assertNotEqual(self.ex.core.state, 'RUNNING')
        self.req('execute')
        self.assertEqual(self.ex.core.state, 'RUNNING', self.ex.note)
        run = self.ex.run
        self.req('execute')
        self.assertEqual(self.ex.run, run)


if __name__ == '__main__':
    unittest.main()
