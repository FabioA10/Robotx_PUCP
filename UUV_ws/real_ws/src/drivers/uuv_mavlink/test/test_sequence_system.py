"""Fault-injection unit tests. ROS and MAVLink transport are mocked, never hardware."""

import json
import math
import sys
import types
import unittest
from unittest.mock import patch

from uuv_mavlink.sequence_core import Limits, parse_steps, Pose, return_steps, Sequence


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
with patch.dict(
    sys.modules,
    {
        'rclpy': fake_ros,
        'rclpy.node': fake_node,
        'std_msgs': types.ModuleType('std_msgs'),
        'std_msgs.msg': fake_std,
    },
):
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
        self.parameters.update(
            sequence_monitor_enabled=True,
            enable_sequence_output=enabled,
            dvl_rest_base_url='',
            dvl_tcp_host='',
        )
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
    @staticmethod
    def absolute_point(**overrides):
        point = {
            'kind': 'absolute_waypoint',
            'x': 0.50,
            'y': 0.00,
            'depth': 0.30,
            'yaw_deg': 90.0,
            'speed': 0.15,
            'vertical_speed': 0.08,
            'yaw_speed_deg': 10.0,
            'tolerance_x': 0.04,
            'tolerance_y': 0.04,
            'tolerance_depth': 0.02,
            'tolerance_yaw_deg': 5.0,
            'dwell': 1.0,
        }
        point.update(overrides)
        return point

    def test_absolute_waypoint_is_fixed_to_prepared_mission_frame(self):
        origin = Pose(3.0, 4.0, 0.10, math.radians(90))
        core = Sequence(Limits(max_depth=2.0, radius=2.0))
        core.start(origin, [self.absolute_point(x=1.0, y=0.5)], 0, origin)
        self.assertAlmostEqual(core.goal.n, 2.5)
        self.assertAlmostEqual(core.goal.e, 5.0)
        self.assertAlmostEqual(core.goal.depth, 0.30)
        self.assertAlmostEqual(abs(core.goal.yaw), math.pi)
        self.assertEqual(core.waypoint_phase, 'DEPTH')

    def test_absolute_waypoint_executes_depth_then_xy_then_final_yaw(self):
        limits = Limits(max_depth=2.0, radius=2.0, waypoint_timeout=120.0)
        core = Sequence(limits)
        pose = Pose(0.0, 0.0, 0.10, 0.0)
        core.start(pose, [self.absolute_point()], 0.0, pose)
        phases = []
        velocity = [0.0, 0.0, 0.0]
        for tick in range(1, 2401):
            command = core.update(
                pose,
                tick * 0.05,
                velocity=velocity,
            )
            if core.state != 'RUNNING':
                break
            phases.append(core.waypoint_phase)
            vn, ve, vd, yaw = command
            velocity = [vn, ve, vd]
            pose = Pose(
                pose.n + vn * 0.05,
                pose.e + ve * 0.05,
                pose.depth + vd * 0.05,
                yaw,
            )
            self.assertLessEqual(math.hypot(vn, ve), 0.150001)
            self.assertLessEqual(abs(vd), 0.080001)
        self.assertEqual(core.state, 'COMPLETED', core.reason)
        first_translate = phases.index('TRANSLATE')
        first_yaw = phases.index('YAW')
        self.assertGreater(first_translate, phases.index('DEPTH'))
        self.assertGreater(first_yaw, first_translate)
        self.assertAlmostEqual(pose.depth, 0.30, delta=0.02)
        self.assertAlmostEqual(pose.n, 0.50, delta=0.04)
        self.assertAlmostEqual(pose.e, 0.00, delta=0.04)
        self.assertAlmostEqual(pose.yaw, math.radians(90), delta=math.radians(5))

    def test_absolute_waypoint_pauses_xy_when_depth_leaves_band(self):
        origin = Pose(0.0, 0.0, 0.30, 0.0)
        core = Sequence(Limits(max_depth=2.0, radius=2.0))
        core.start(origin, [self.absolute_point()], 0.0, origin)
        self.assertEqual(core.waypoint_phase, 'TRANSLATE')
        command = core.update(
            Pose(0.10, 0.0, 0.34, 0.0),
            0.05,
            velocity=[0.02, 0.0, 0.0],
        )
        self.assertEqual(core.waypoint_phase, 'DEPTH')
        self.assertLessEqual(math.hypot(command[0], command[1]), 0.0061)
        self.assertLess(command[2], 0.0)

    def test_absolute_waypoint_rejects_unsafe_geometry_and_tolerances(self):
        core = Sequence(Limits(max_depth=1.0, radius=0.5))
        for point in (
            self.absolute_point(x=0.6),
            self.absolute_point(depth=1.1),
            self.absolute_point(tolerance_x=0.001),
            self.absolute_point(tolerance_yaw_deg=30.0),
        ):
            with self.subTest(point=point), self.assertRaises(ValueError):
                core.validate(Pose(0, 0, 0.2, 0), [point])

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
        for steps in (
            [],
            [('turn', 360)],
            [('wait', -1)],
            [('advance', math.nan)],
            [('depth', True)],
        ):
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
            speed = math.sqrt(vn * vn + ve * ve + vd * vd)
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

    def test_hold_freezes_step_timeout_and_resets_dwell(self):
        core = Sequence(Limits(step_timeout=1))
        pose = Pose(0, 0, 0, 0)
        core.start(pose, [('wait', 0.5)], 0)
        core.update(pose, 0.2)
        self.assertEqual(core.inside_since, 0.2)
        core.hold(0.9)
        self.assertAlmostEqual(core.started, 0.7)
        self.assertIsNone(core.inside_since)
        command = core.update(pose, 0.95)
        self.assertIsNotNone(command)
        self.assertEqual(core.state, 'RUNNING')

    def test_home_has_fixed_endpoint_despite_heading_error(self):
        home = Pose(0, 0, 0.5, math.radians(-179))
        start = Pose(1, 1, 0.5, math.radians(179))
        core = Sequence()
        core.start(start, return_steps(start, home), 0, home, returning=True)
        core.index = 2
        core.begin_step(Pose(1.05, 1.03, 0.51, -2.3), 1)
        self.assertEqual((core.goal.n, core.goal.e, core.goal.depth), (0, 0, 0.5))

    def test_mixed_position_and_timed_velocity_queue(self):
        core = Sequence(Limits(max_depth=3, radius=5))
        pose = Pose(0, 0, 0.2, 0)
        steps = [
            {'kind': 'lateral', 'value': 0.5, 'speed': 0.15},
            {'kind': 'velocity_surge', 'speed': 0.10, 'duration': 1.0},
            {'kind': 'velocity_yaw', 'speed': math.radians(5), 'duration': 1.0},
        ]
        core.start(pose, steps, 0)
        self.assertAlmostEqual(core.goal.e, 0.5)
        core.index = 1
        core.begin_step(pose, 1.0)
        core.last_time = 1.0
        command = core.update(pose, 1.05)
        self.assertAlmostEqual(command[0], 0.10)
        self.assertAlmostEqual(command[1], 0.0)
        self.assertEqual(core.state, 'RUNNING')

    def test_timed_velocity_limits_are_enforced(self):
        for steps in (
            [{'kind': 'velocity_surge', 'speed': 0.21, 'duration': 1}],
            [{'kind': 'velocity_heave', 'speed': 0.11, 'duration': 1}],
            [{'kind': 'velocity_yaw', 'speed': math.radians(11), 'duration': 1}],
            [{'kind': 'velocity_surge', 'speed': 0.1, 'duration': 5.1}],
        ):
            with self.assertRaises(ValueError):
                Sequence().validate(Pose(0, 0, 0, 0), steps)

    def test_operator_can_raise_motion_limits(self):
        limits = Limits(horizontal_speed=0.8, vertical_speed=0.4, yaw_speed=math.radians(45))
        steps = [
            {'kind': 'velocity_surge', 'speed': 0.7, 'duration': 1},
            {'kind': 'velocity_heave', 'speed': -0.3, 'duration': 1},
            {'kind': 'velocity_yaw', 'speed': math.radians(40), 'duration': 1},
        ]
        parsed = Sequence(limits).validate(Pose(0, 0, 1.0, 0), steps)
        self.assertEqual(parsed, steps)

    def test_timed_plan_includes_braking_margin_in_bounds(self):
        limits = Limits(radius=0.5, horizontal_speed=0.2)
        steps = [
            {'kind': 'velocity_surge', 'speed': 0.1, 'duration': 5.0},
        ]
        with self.assertRaisesRegex(ValueError, 'radio'):
            Sequence(limits).validate(Pose(0, 0, 0.2, 0), steps)

    def test_timed_motion_cancels_if_pose_is_lost_or_exits_bounds(self):
        pose = Pose(0, 0, 0.2, 0)
        step = [{'kind': 'velocity_surge', 'speed': 0.1, 'duration': 1.0}]
        core = Sequence()
        core.start(pose, step, 0)
        self.assertIsNone(core.update(None, 0.05))
        self.assertEqual(core.state, 'CANCELLED')

        core = Sequence(Limits(radius=0.5))
        core.start(pose, step, 0)
        self.assertIsNone(core.update(Pose(0.6, 0, 0.2, 0), 0.05))
        self.assertEqual(core.state, 'CANCELLED')

    def test_low_requested_speed_is_preserved_when_vehicle_responds(self):
        core = Sequence(Limits(recovery_delay=1.0))
        origin = Pose(0, 0, 0.2, 0)
        core.start(
            origin,
            [{'kind': 'advance', 'value': 0.2, 'speed': 0.005}],
            0,
        )
        command = None
        for tick in range(1, 61):
            pose = Pose((tick - 1) * 0.005 * 0.05, 0, 0.2, 0)
            command = core.update(pose, tick * 0.05, velocity=[0.005, 0, 0])
        self.assertFalse(core.recovery_active)
        self.assertLessEqual(abs(command[0]), 0.0050001)

    def test_stalled_low_speed_step_escalates_after_configured_delay(self):
        core = Sequence(Limits(recovery_delay=1.0))
        origin = Pose(0, 0, 0.2, 0)
        core.start(
            origin,
            [{'kind': 'lateral', 'value': 0.2, 'speed': 0.005}],
            0,
        )
        commands = []
        for tick in range(1, 24):
            commands.append(
                core.update(origin, tick * 0.05, velocity=[0, 0, 0])
            )
        self.assertAlmostEqual(commands[18][1], 0.005)
        self.assertTrue(core.recovery_active)
        self.assertIn('lenta', core.recovery_reason)
        self.assertGreater(commands[-1][1], 0.005)
        self.assertLessEqual(math.hypot(commands[-1][0], commands[-1][1]), 0.080001)

    def test_velocity_damping_brakes_before_crossing_target(self):
        core = Sequence()
        origin = Pose(0, 0, 0.2, 0)
        core.start(
            origin,
            [{'kind': 'advance', 'value': 0.2, 'speed': 0.05}],
            0,
        )
        command = None
        for tick in range(1, 11):
            command = core.update(
                Pose(0.17, 0, 0.2, 0),
                tick * 0.05,
                velocity=[0.04, 0, 0],
            )
        self.assertLess(command[0], 0.0)
        self.assertLess(0.17, core.goal.n)

    def test_large_hold_drift_uses_recovery_and_vector_speed_cap(self):
        core = Sequence()
        origin = Pose(0, 0, 0.2, 0)
        core.start(origin, [('wait', 2)], 0)
        command = core.update(
            Pose(0.10, 0.10, 0.2, 0),
            0.05,
            velocity=[0, 0, 0],
        )
        self.assertTrue(core.recovery_active)
        self.assertIn('fuera del umbral', core.recovery_reason)
        self.assertLessEqual(math.hypot(command[0], command[1]), 0.080001)

    def test_adaptive_limit_relationships_are_rejected(self):
        invalid = (
            {'recovery_radius': 0.03},
            {'horizontal_speed': 0.05, 'recovery_speed': 0.10},
            {'recovery_speed': 0.02, 'recovery_min_speed': 0.03},
            {'response_ratio': 1.1},
        )
        for changes in invalid:
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                Limits(**changes)


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
        for key, value in {
            'position': [0, 0, 0],
            'velocity': [0, 0, 0],
            'yaw': 0,
            'pressure': 1000,
            'ekf': 47,
            'dvl': 90,
            'voltage': 16,
        }.items():
            self.g.put(key, value)
        self.g.version = [4, 5, 7]
        self.g.params = {
            k: (v, self.now)
            for k, v in {'FS_PILOT_INPUT': 2, 'FS_PILOT_TIMEOUT': 3, 'SYSID_MYGCS': 255}.items()
        }

    def command(self, **overrides):
        status = json.loads(self.g.status_pub.messages[-1])
        packet = {
            'session': status['session'],
            'permit': status['permit'],
            'ticket': status['ticket'],
            'run': 'run1',
            'serial': 0,
            'command': [0.1, 0, 0, 0],
            'limits': {
                'horizontal_speed': 0.2,
                'vertical_speed': 0.1,
                'yaw_speed': math.radians(10),
            },
        }
        packet.update(overrides)
        return String(json.dumps(packet))

    def action(self, action):
        status = json.loads(self.g.status_pub.messages[-1])
        return String(
            json.dumps(
                {
                    'session': status['session'],
                    'permit': status['permit'],
                    'ticket': status['ticket'],
                    'action': action,
                }
            )
        )

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
        packets = [
            a for n, a in self.b.master.mav.sent if n == 'set_position_target_local_ned_send'
        ]
        self.assertEqual(packets[-1][1:5], (1, 1, 1, 2503))
        self.assertEqual(packets[-1][8:11], (0.1, 0.0, 0.0))

    def test_guided_entry_keeps_one_position_controller_and_manual_pose(self):
        self.b.mode = 'MANUAL'
        self.g.tick()
        self.g.put('position', [1.2, -0.4, 0.7])
        self.g.put('yaw', 0.3)
        self.g.tick()
        before = len(
            [1 for n, _ in self.b.master.mav.sent if n == 'set_position_target_local_ned_send']
        )
        self.g.action(self.action('guided'))
        self.assertEqual(self.g.hold_target, (1.2, -0.4, 0.7, 0.3))
        self.assertEqual(self.g.guided_entry_reference, (1.2, -0.4, 0.7, 0.3))
        after = len(
            [1 for n, _ in self.b.master.mav.sent if n == 'set_position_target_local_ned_send']
        )
        self.assertEqual(after, before + 2)
        self.assertEqual(
            [name for name, _ in self.b.master.mav.sent[-3:]],
            [
                'set_mode_send',
                'set_position_target_local_ned_send',
                'set_position_target_local_ned_send',
            ],
        )
        transition_packets = [
            args
            for name, args in self.b.master.mav.sent
            if name == 'set_position_target_local_ned_send'
        ][before:after]
        self.assertTrue(transition_packets)
        self.assertTrue(all(packet[4] == 2552 for packet in transition_packets))
        self.assertTrue(
            all(packet[5:8] == (1.2, -0.4, 0.7) for packet in transition_packets)
        )
        self.assertTrue(all(packet[14] == 0.3 for packet in transition_packets))
        mode_packets = [a for n, a in self.b.master.mav.sent if n == 'set_mode_send']
        self.assertEqual(mode_packets[-1], (1, 1, 4))

        self.g.tick()
        repeated = len(
            [1 for n, _ in self.b.master.mav.sent if n == 'set_position_target_local_ned_send']
        )
        self.assertEqual(repeated, after + 2)
        pending_packets = [
            args
            for name, args in self.b.master.mav.sent
            if name == 'set_position_target_local_ned_send'
        ][after:repeated]
        self.assertTrue(all(packet[4] == 2552 for packet in pending_packets))
        self.assertTrue(all(packet[5:8] == (1.2, -0.4, 0.7) for packet in pending_packets))

        self.g.put('position', [1.25, -0.45, 0.72])
        self.g.put('velocity', [0, 0, 0])
        self.g.put('yaw', 0.32)
        self.b.mode = 'GUIDED'
        self.g.tick()
        self.assertEqual(self.g.hold_target, (1.2, -0.4, 0.7, 0.3))
        packets = [
            a for n, a in self.b.master.mav.sent if n == 'set_position_target_local_ned_send'
        ]
        self.assertEqual(packets[-1][1:5], (1, 1, 1, 2552))
        self.assertEqual(packets[-1][5:8], (1.2, -0.4, 0.7))
        self.assertEqual(packets[-1][8:14], (0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        self.assertAlmostEqual(packets[-1][14], 0.3)
        self.assertTrue(all(packet[4] == 2552 for packet in packets[before:]))
        self.assertIn('Estabilizando entrada a GUIDED', self.g.output_reasons())

        self.now += 5.01
        self.refresh()
        self.g.put('position', [1.2, -0.4, 0.7])
        self.g.put('velocity', [0, 0, 0])
        self.g.put('yaw', 0.3)
        self.g.tick()
        self.assertNotIn('Estabilizando entrada a GUIDED', self.g.output_reasons())
        self.assertIn('listo para iniciar', self.g.last_reason)

    def test_second_guided_entry_never_reuses_first_hold_target(self):
        self.b.mode = 'MANUAL'
        self.g.tick()
        self.g.put('position', [1, 2, 0.3])
        self.g.put('yaw', 0.1)
        self.g.tick()
        self.g.action(self.action('guided'))
        self.assertEqual(self.g.hold_target, (1, 2, 0.3, 0.1))
        self.b.mode = 'GUIDED'
        self.g.put('position', [1.01, 2.01, 0.31])
        self.g.put('velocity', [0, 0, 0])
        self.g.put('yaw', 0.11)
        self.g.tick()
        self.assertEqual(self.g.hold_target, (1, 2, 0.3, 0.1))

        self.b.mode = 'MANUAL'
        self.g.tick()
        self.g.put('position', [4, 5, 0.6])
        self.g.put('velocity', [0, 0, 0])
        self.g.put('yaw', 0.4)
        self.g.tick()
        before = len(
            [1 for n, _ in self.b.master.mav.sent if n == 'set_position_target_local_ned_send']
        )
        self.g.action(self.action('guided'))
        self.assertEqual(
            len(
                [1 for n, _ in self.b.master.mav.sent if n == 'set_position_target_local_ned_send']
            ),
            before + 2,
        )
        self.b.mode = 'GUIDED'
        self.g.put('position', [4.01, 5.01, 0.62])
        self.g.put('velocity', [0, 0, 0])
        self.g.put('yaw', 0.42)
        self.g.tick()
        self.assertEqual(self.g.hold_target, (4, 5, 0.6, 0.4))
        packets = [
            a for n, a in self.b.master.mav.sent if n == 'set_position_target_local_ned_send'
        ]
        self.assertEqual(packets[-1][5:8], (4, 5, 0.6))

    def test_guided_entry_rejects_vehicle_already_moving(self):
        self.b.mode = 'MANUAL'
        self.g.tick()
        for velocity, text in [([0.151, 0, 0], 'horizontal'), ([0, 0, -0.151], 'vertical')]:
            self.g.put('velocity', velocity)
            before = len([1 for name, _ in self.b.master.mav.sent if name == 'set_mode_send'])
            self.g.action(self.action('guided'))
            after = len([1 for name, _ in self.b.master.mav.sent if name == 'set_mode_send'])
            self.assertEqual(after, before)
            self.assertIn('movimiento ' + text, self.g.last_reason)
            self.g.tick()

    def test_guided_entry_allows_small_water_drift_without_moving_target(self):
        self.b.mode = 'MANUAL'
        self.g.tick()
        self.g.put('velocity', [0.12, 0, 0])
        self.g.action(self.action('guided'))
        self.assertIsNotNone(self.g.guided_request_at)

        self.g.put('position', [0.11, 0.02, 0.03])
        self.g.put('velocity', [0.02, 0, 0])
        self.g.put('yaw', math.radians(3))
        self.g.tick()
        self.assertIsNone(self.g.abort_mode)

        self.b.mode = 'GUIDED'
        self.g.tick()
        self.assertEqual(
            self.g.hold_target,
            (0, 0, 0, 0),
        )

    def test_guided_entry_vertical_transient_returns_to_manual(self):
        self.b.mode = 'MANUAL'
        self.g.tick()
        self.g.action(self.action('guided'))
        self.b.mode = 'GUIDED'
        self.g.tick()
        self.g.put('position', [0, 0, 0.02])
        self.g.put('velocity', [0, 0, 0.16])
        self.g.tick()
        self.assertEqual(self.g.abort_mode, 'MANUAL')
        self.assertIn('Velocidad vertical', self.g.last_reason)
        packets = [
            a for n, a in self.b.master.mav.sent if n == 'set_position_target_local_ned_send'
        ]
        self.assertEqual(packets[-1][4], 1479)
        mode_packets = [a for n, a in self.b.master.mav.sent if n == 'set_mode_send']
        self.assertEqual(mode_packets[-1], (1, 1, 19))

    def test_delayed_guided_drift_is_guarded_after_settling(self):
        self.b.mode = 'MANUAL'
        self.g.tick()
        self.g.action(self.action('guided'))
        self.b.mode = 'GUIDED'
        self.g.tick()

        self.now += 5.01
        self.refresh()
        self.g.tick()
        self.assertIsNone(self.g.guided_settle_started)
        self.assertIsNotNone(self.g.guided_entry_reference)
        self.assertIn('listo para iniciar', self.g.last_reason)

        self.g.put('position', [0, 0, 0.121])
        self.g.put('velocity', [0, 0, 0.01])
        self.g.tick()
        self.assertEqual(self.g.abort_mode, 'MANUAL')
        self.assertIn('Desviación vertical', self.g.last_reason)

    def test_guided_entry_guards_horizontal_position_speed_and_yaw(self):
        cases = [
            ([0.201, 0, 0], [0, 0, 0], 0, 'Desviación horizontal'),
            ([0, 0, 0], [0.151, 0, 0], 0, 'Velocidad horizontal'),
            ([0, 0, 0], [0, 0, 0], math.radians(15.1), 'Desviación de rumbo'),
        ]
        for position, velocity, yaw, text in cases:
            self.g.guided_entry_reference = (0, 0, 0, 0)
            self.g.put('position', position)
            self.g.put('velocity', velocity)
            self.g.put('yaw', yaw)
            self.assertIn(text, self.g.guided_entry_fault())

    def test_external_guided_entry_is_rejected_without_reusing_target(self):
        self.g.hold_target = (9, 9, 9, 0.9)
        self.b.mode = 'MANUAL'
        self.g.tick()
        self.g.put('position', [2, 3, 0.4])
        self.g.put('velocity', [0, 0, 0])
        self.g.put('yaw', 0.2)
        self.b.mode = 'GUIDED'
        self.g.tick()
        self.assertIsNone(self.g.hold_target)
        self.assertEqual(self.g.abort_mode, 'MANUAL')
        self.assertIn('entrada externa sin protocolo seguro', self.g.last_reason)
        packets = [
            a for n, a in self.b.master.mav.sent if n == 'set_position_target_local_ned_send'
        ]
        self.assertEqual(packets[-1][1:5], (1, 1, 1, 1479))
        self.assertEqual(packets[-1][8:11], (0.0, 0.0, 0.0))
        mode_packets = [a for n, a in self.b.master.mav.sent if n == 'set_mode_send']
        self.assertEqual(mode_packets[-1], (1, 1, 19))

    def test_unconfirmed_guided_request_expires_without_stale_target(self):
        self.b.mode = 'MANUAL'
        self.g.tick()
        self.g.action(self.action('guided'))
        self.assertIsNotNone(self.g.guided_request_at)
        self.now += 2.01
        self.refresh()
        self.g.tick()
        self.assertIsNone(self.g.guided_request_at)
        self.assertIsNone(self.g.hold_target)
        self.assertIn('no confirmado', self.g.last_reason)

    def test_guided_entry_without_pose_is_rejected(self):
        self.b.mode = 'MANUAL'
        self.g.tick()
        self.g.samples.pop('position', None)
        self.g.action(self.action('guided'))
        mode_packets = [a for n, a in self.b.master.mav.sent if n == 'set_mode_send']
        self.assertEqual(mode_packets, [])
        self.assertIn('posición o rumbo no disponibles', self.g.last_reason)

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
        packets = [
            a for n, a in self.b.master.mav.sent if n == 'set_position_target_local_ned_send'
        ]
        self.assertEqual(packets[-1][8:11], (0.0, 0.0, 0.0))

    def test_only_one_run_and_increasing_serial(self):
        self.g.receive(self.command())
        self.g.receive(self.command(run='other', serial=100, command=[0.2, 0, 0, 0]))
        self.assertEqual(self.g.owner, 'run1')
        self.assertEqual(self.g.last_command[0], 0.1)
        self.g.receive(self.command(serial=0, command=[0.2, 0, 0, 0]))
        self.assertEqual(self.g.last_command[0], 0.1)

    def test_sensor_diagnostics_do_not_take_motion_authority(self):
        for missing in ('position', 'pressure', 'ekf'):
            self.refresh()
            self.g.abort_mode = None
            self.g.tick()
            self.g.receive(self.command())
            del self.g.samples[missing]
            self.g.tick()
            self.assertEqual(self.g.owner, 'run1', missing)
            self.g.stop('fin de caso')
        self.refresh()
        self.g.abort_mode = None
        self.g.tick()
        self.g.receive(self.command())
        self.b.deadman = True
        self.g.tick()
        self.assertIsNone(self.g.owner)
        self.assertEqual(self.g.abort_mode, 'MANUAL')

    def test_short_dvl_loss_is_diagnostic_and_command_continues(self):
        self.g.receive(self.command())
        del self.g.samples['dvl']
        self.g.tick()
        self.assertEqual(self.g.owner, 'run1')
        packets = [
            a for n, a in self.b.master.mav.sent if n == 'set_position_target_local_ned_send'
        ]
        self.assertEqual(packets[-1][8:11], (0.1, 0.0, 0.0))
        status = json.loads(self.g.status_pub.messages[-1])
        self.assertIn('Sin datos recientes: dvl', status['diagnostics'])
        self.assertNotIn('Sin datos recientes: dvl', status['blockers'])

    def test_normal_mavlink_jitter_does_not_pause_transport(self):
        self.g.receive(self.command())
        for key in ('position', 'velocity', 'yaw', 'pressure', 'ekf'):
            value, _ = self.g.samples[key]
            self.g.samples[key] = (value, self.now - 0.9)
        self.assertEqual(self.g.health(), [])
        self.g.receive(self.command(serial=1, command=[0, 0, 0, 0]))
        self.g.tick()
        self.assertEqual(self.g.owner, 'run1')

    def test_dvl_loss_never_times_out_or_rejects_motion(self):
        self.g.receive(self.command())
        del self.g.samples['dvl']
        self.g.tick()
        for serial in range(1, 17):
            self.now += 0.20
            self.refresh()
            del self.g.samples['dvl']
            self.g.receive(self.command(serial=serial, command=[0.1, 0, 0, 0]))
            self.g.tick()
        self.assertEqual(self.g.owner, 'run1')

    def test_direct_invalid_dvl_and_related_ekf_are_diagnostics(self):
        self.g.receive(self.command())
        self.g.dvl_tcp_seen = True
        self.g.dvl_tcp_valid = False
        self.g.dvl_tcp_at = self.now
        self.g.put('ekf', 1)
        self.g.tick()
        self.assertEqual(self.g.owner, 'run1')

        self.g.put('pressure', 2000)
        self.g.tick()
        self.assertIsNone(self.g.owner)

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

    def test_bridge_accepts_configured_speed_and_locks_run_limits(self):
        limits = {'horizontal_speed': 0.6, 'vertical_speed': 0.3, 'yaw_speed': math.radians(30)}
        self.g.receive(self.command(command=[0.5, 0, 0, 0], limits=limits))
        self.assertEqual(self.g.owner, 'run1')
        self.assertAlmostEqual(self.g.last_command[0], 0.5)
        self.g.receive(self.command(serial=1, command=[0.61, 0, 0, 0], limits=limits))
        self.assertIsNone(self.g.owner)

    def test_duplicate_sensor_time_does_not_refresh_cached_data(self):
        packet = Packet(
            kind='VISION_POSITION_DELTA',
            sysid=255,
            compid=0,
            confidence=90,
            position_delta=[0, 0, 0],
            angle_delta=[0, 0, 0],
            time_usec=100000,
            time_delta_usec=100000,
        )
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
        packet = Packet(
            kind='VISION_POSITION_DELTA',
            sysid=255,
            compid=0,
            confidence=99.7,
            position_delta=[0.0001, 0, 0],
            angle_delta=[0, 0, 0],
            time_usec=0,
            time_delta_usec=78440,
        )
        epoch = self.g.nav_epoch
        # Identical deltas can legitimately occur while the ROV is stationary.
        for _ in range(25):
            self.now += 0.08
            self.g.observe(packet)
            self.assertEqual(self.g.get('dvl'), 99.7)
        self.assertEqual(self.g.nav_epoch, epoch)
        self.now += 1.49
        self.assertEqual(self.g.get('dvl', 1.5), 99.7)
        self.now += 0.02
        self.assertIsNone(self.g.get('dvl', 1.5))
        self.assertIn('Sin datos recientes: dvl', self.g.health())

    def test_zero_timestamp_still_requires_correct_source_and_valid_fields(self):
        self.g.samples.pop('dvl')
        fields = {
            'kind': 'VISION_POSITION_DELTA',
            'sysid': 255,
            'compid': 0,
            'confidence': 99.7,
            'position_delta': [0, 0, 0],
            'angle_delta': [0, 0, 0],
            'time_usec': 0,
            'time_delta_usec': 75000,
        }
        for change in (
            {'sysid': 99},
            {'compid': 1},
            {'confidence': float('nan')},
            {'time_delta_usec': 0},
            {'time_usec': -1},
        ):
            self.g.observe(Packet(**dict(fields, **change)))
            self.assertIsNone(self.g.get('dvl'), change)
        self.g.observe(Packet(**dict(fields, confidence=20)))
        self.assertIn('Confianza del mensaje DVL insuficiente', self.g.health())

    def test_positive_dvl_timestamp_checks_survive_zero_timestamp_packets(self):
        fields = {
            'kind': 'VISION_POSITION_DELTA',
            'sysid': 255,
            'compid': 0,
            'confidence': 90,
            'position_delta': [0, 0, 0],
            'angle_delta': [0, 0, 0],
            'time_delta_usec': 75000,
        }
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
        self.assertEqual(self.g.nav_epoch, epoch)
        self.assertIsNone(self.g.get('dvl'))

    @staticmethod
    def rest_dvl(counter, confidence=99.7, updated=None, **changes):
        message = {
            'type': 'VISION_POSITION_DELTA',
            'confidence': confidence,
            'position_delta': [0.0001, 0, 0],
            'angle_delta': [0, 0, 0],
            'time_delta_usec': 80000,
            'time_usec': 0,
        }
        message.update(changes)
        return {
            'message': message,
            'status': {
                'time': {'counter': counter, 'last_update': updated or f'update-{counter}'}
            },
        }

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
        self.assertEqual(self.g.nav_epoch, epoch)
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
        self.f = {
            'session': 'bridge',
            'nav_epoch': 0,
            'permit': 'permit',
            'ticket': 'bridge-ticket',
            'samples': {
                'position': [0, 0, 0],
                'velocity': [0, 0, 0],
                'pressure': 1000,
                'yaw': 0,
                'dvl': 90,
            },
            'health': [],
            'blockers': ['Salida real deshabilitada'],
            'mode': 'MANUAL',
            'armed': False,
            'enabled': False,
            'reason': 'test',
        }
        for _ in range(45):
            self.advance()

    def advance(self):
        self.now += 0.05
        self.ex.on_feedback(String(json.dumps(self.f)))
        self.ex.tick()

    def req(self, action, **extra):
        s = json.loads(self.ex.state_pub.messages[-1])
        packet = {
            'session': s['session'],
            'ticket': s['ticket'],
            'ui': 'ui',
            'serial': self.action_counter,
            'action': action,
            'sensor_depth': 0.2,
            'max_depth': 1,
            'radius': 2,
            'max_horizontal_speed': 0.2,
            'max_vertical_speed': 0.1,
            'max_yaw_speed_deg': 10,
            'xy_tolerance': 0.03,
            'recovery_radius': 0.08,
            'recovery_speed': 0.08,
            'recovery_min_speed': 0.020,
            'recovery_delay': 2.0,
            'position_gain': 0.80,
            'velocity_damping': 1.20,
            'steps': [['advance', 0.5]],
        }
        self.action_counter += 1
        packet.update(extra)
        self.ex.on_request(String(json.dumps(packet)))

    def ui(self):
        s = json.loads(self.ex.state_pub.messages[-1])
        self.ex.on_ui(
            String(
                json.dumps(
                    {
                        'session': s['session'],
                        'ticket': s['ticket'],
                        'ui': 'ui',
                        'serial': self.ui_counter,
                    }
                )
            )
        )
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

    def test_prepare_transaction_does_not_arm_or_move(self):
        self.req('prepare', sensor_depth=0.15, max_depth=0.2, radius=0.5, steps=[['wait', 3]])
        self.assertIsNotNone(self.ex.reference, self.ex.note)
        self.assertIsNotNone(self.ex.home)
        self.assertIsNotNone(self.ex.validated)
        self.assertEqual(self.ex.core.limits.max_depth, 0.2)
        self.assertEqual(self.ex.core.limits.horizontal_speed, 0.2)
        self.assertEqual(self.ex.prepared_by, ['ui', 0])
        self.assertEqual(self.ex.command_pub.messages, [])
        self.assertEqual(self.ex.action_pub.messages, [])

    def test_prepare_and_execute_absolute_plan_keep_fixed_home_origin(self):
        point = TestSequenceCore.absolute_point(x=0.5, y=0.25, depth=0.3)
        self.req('prepare', plan_mode='absolute', steps=[point])
        self.assertIsNotNone(self.ex.validated, self.ex.note)
        self.assertEqual(self.ex.validated['plan_mode'], 'absolute')
        self.assertEqual(self.ex.plan_mode, 'absolute')
        self.f['samples']['position'] = [0.08, -0.03, 0.0]
        self.f.update(armed=True, mode='GUIDED', enabled=True, blockers=[])
        self.advance()
        self.ui()
        self.req('execute', plan_mode='absolute', steps=[point])
        self.assertEqual(self.ex.core.state, 'RUNNING', self.ex.note)
        self.assertAlmostEqual(self.ex.core.goal.n, 0.5)
        self.assertAlmostEqual(self.ex.core.goal.e, 0.25)
        self.assertEqual(self.ex.core.waypoint_phase, 'DEPTH')

    def test_prepare_rejects_mixed_or_mislabeled_absolute_plan(self):
        point = TestSequenceCore.absolute_point()
        self.req('prepare', plan_mode='absolute', steps=[point, ['wait', 1]])
        self.assertIsNone(self.ex.validated)
        for _ in range(45):
            self.advance()
        self.req('prepare', plan_mode='relative', steps=[point])
        self.assertIsNone(self.ex.validated)

    def test_prepare_accepts_operator_motion_limits(self):
        self.req(
            'prepare',
            max_horizontal_speed=0.8,
            max_vertical_speed=0.4,
            max_yaw_speed_deg=45,
            steps=[{'kind': 'velocity_surge', 'speed': 0.7, 'duration': 1}],
        )
        self.assertIsNotNone(self.ex.validated, self.ex.note)
        self.assertAlmostEqual(self.ex.core.limits.horizontal_speed, 0.8)
        self.assertAlmostEqual(self.ex.core.limits.vertical_speed, 0.4)
        self.assertAlmostEqual(self.ex.core.limits.yaw_speed, math.radians(45))

    def test_prepare_accepts_adaptive_precision_controls(self):
        self.req(
            'prepare',
            xy_tolerance=0.025,
            recovery_radius=0.075,
            recovery_speed=0.12,
            recovery_min_speed=0.03,
            recovery_delay=1.5,
            position_gain=0.9,
            velocity_damping=1.1,
        )
        limits = self.ex.core.limits
        self.assertIsNotNone(self.ex.validated, self.ex.note)
        self.assertAlmostEqual(limits.xy_tolerance, 0.025)
        self.assertAlmostEqual(limits.recovery_radius, 0.075)
        self.assertAlmostEqual(limits.recovery_speed, 0.12)
        self.assertAlmostEqual(limits.recovery_min_speed, 0.03)
        self.assertAlmostEqual(limits.recovery_delay, 1.5)
        self.assertAlmostEqual(limits.position_gain, 0.9)
        self.assertAlmostEqual(limits.velocity_damping, 1.1)

    def test_prepare_rejects_incoherent_recovery_limits(self):
        for changes in (
            {'recovery_radius': 0.03},
            {'recovery_speed': 0.21},
            {'recovery_speed': 0.02, 'recovery_min_speed': 0.03},
        ):
            self.req('prepare', **changes)
            self.assertIsNone(self.ex.validated, changes)
            for _ in range(45):
                self.advance()

    def test_prepare_rejects_armed_mode_and_hard_health_fault(self):
        for changes in (
            {'armed': True},
            {'mode': 'GUIDED'},
            {'health': ['Sin heartbeat reciente']},
        ):
            old = dict(self.f)
            self.f.update(changes)
            self.advance()
            self.req('prepare')
            self.assertIsNone(self.ex.validated, changes)
            self.f = old
            for _ in range(45):
                self.advance()
        self.req('prepare')
        self.assertIsNotNone(self.ex.validated)

    def test_failed_prepare_clears_previous_plan_and_references(self):
        self.req('prepare')
        self.assertIsNotNone(self.ex.validated)
        self.req('prepare', steps=[['depth', 2]], max_depth=0.2)
        self.assertIsNone(self.ex.validated)
        self.assertIsNone(self.ex.reference)
        self.assertIsNone(self.ex.home)
        self.assertIsNone(self.ex.prepared_by)

    def test_prepare_needs_stable_pressure_and_rejects_changed_limits(self):
        self.ex.pressure_history.clear()
        self.req('prepare')
        self.assertIsNone(self.ex.validated)
        for _ in range(45):
            self.advance()
        self.req('prepare')
        self.assertIsNotNone(self.ex.validated, self.ex.note)
        self.f.update(armed=True, mode='GUIDED', enabled=True, blockers=[])
        self.advance()
        self.ui()
        self.req('execute', max_depth=2)
        self.assertNotEqual(self.ex.core.state, 'RUNNING')

    def test_pressure_reference_depth_and_guard_against_ekf_reset(self):
        self.req('surface', sensor_depth=0.2)
        self.f['samples']['pressure'] += 1000 * 9.80665 * 0.3 / 100
        self.f['samples']['position'][2] = 0.3
        self.advance()
        self.assertAlmostEqual(self.ex.pose().depth, 0.5)
        self.f['samples']['position'][2] = 2
        self.advance()
        self.assertIsNone(self.ex.reference)

    def test_execute_accepts_repositioning_and_starts_from_current_pose(self):
        self.req('prepare')
        self.assertIsNotNone(self.ex.validated, self.ex.note)
        self.f['samples']['position'][0] = 0.60
        self.f.update(armed=True, mode='GUIDED', enabled=True, blockers=[])
        self.advance()
        phase = self.ex.phase_status(self.now, self.ex.pose(), '')
        self.assertEqual(phase['title'], 'LISTO PARA INICIAR')
        self.ui()
        self.req('execute')
        self.assertEqual(self.ex.core.state, 'RUNNING', self.ex.note)
        self.assertAlmostEqual(self.ex.core.goal.n, 1.1)
        self.assertAlmostEqual(self.ex.home.n, 0.0)

    def test_small_pre_execute_drift_uses_current_reference(self):
        self.req('prepare')
        self.assertIsNotNone(self.ex.validated, self.ex.note)
        self.f['samples']['position'] = [0.05, 0, 0.04]
        self.f['samples']['pressure'] += 1000 * 9.80665 * 0.04 / 100
        self.f.update(armed=True, mode='GUIDED', enabled=True, blockers=[])
        self.advance()
        self.ui()
        self.req('execute')
        self.assertEqual(self.ex.core.state, 'RUNNING', self.ex.note)
        self.assertAlmostEqual(self.ex.core.goal.n, 0.55)
        self.assertAlmostEqual(self.ex.core.goal.depth, 0.24)

    def test_repositioned_plan_still_respects_original_home_radius(self):
        self.req('prepare')
        self.f['samples']['position'][0] = 1.8
        self.f.update(armed=True, mode='GUIDED', enabled=True, blockers=[])
        self.advance()
        self.ui()
        self.req('execute')
        self.assertNotEqual(self.ex.core.state, 'RUNNING')
        self.assertAlmostEqual(self.ex.home.n, 0.0)

    def test_guided_request_after_repositioning_preserves_plan(self):
        self.req('prepare')
        saved = self.ex.validated
        self.f['samples']['position'][0] = 0.6
        self.f.update(armed=True, mode='MANUAL', enabled=True, blockers=[])
        self.advance()
        self.req('guided')
        self.assertIs(self.ex.validated, saved)
        self.assertIn('GUIDED solicitado', self.ex.note)

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

    def test_hard_transport_loss_invalidates_surface_home_and_plan(self):
        self.start()
        self.f['health'] = ['Sin heartbeat reciente']
        self.advance()
        self.assertIsNone(self.ex.reference)
        self.assertIsNone(self.ex.home)
        self.assertIsNone(self.ex.validated)
        self.assertEqual(self.ex.core.state, 'CANCELLED')

    def test_dvl_diagnostic_keeps_run_and_publishes_motion(self):
        self.start()
        self.ui()
        self.advance()
        self.f.update(
            diagnostics=['DVL reporta velocidad inválida'],
            paused=False,
            dvl_velocity_valid=False,
            dvl_altitude=0.2,
        )
        self.ui()
        self.advance()
        self.assertEqual(self.ex.core.state, 'RUNNING')
        self.assertIsNotNone(self.ex.run)
        command = json.loads(self.ex.command_pub.messages[-1])['command']
        self.assertGreater(command[0], 0.0)
        status = json.loads(self.ex.state_pub.messages[-1])
        self.assertIn('DVL reporta velocidad inválida', status['diagnostics'])

    def test_operator_phase_distinguishes_position_wait_and_timed_motion(self):
        self.start()
        self.ui()
        self.advance()
        status = json.loads(self.ex.state_pub.messages[-1])
        self.assertEqual(status['phase']['code'], 'AVANCE')
        self.assertEqual(status['current_step']['kind'], 'advance')

        timed = Sequence()
        pose = Pose(0, 0, 0.2, 0)
        timed.start(pose, [{'kind': 'velocity_surge', 'speed': 0.1, 'duration': 3}], self.now)
        self.ex.core = timed
        self.ex.total = 1
        phase = self.ex.phase_status(self.now + 0.05, None, 'posición no disponible')
        self.assertEqual(phase['code'], 'BLOQUEADO')
        phase = self.ex.phase_status(self.now + 0.05, pose, '')
        self.assertEqual(phase['code'], 'TEMPORAL')

        core = Sequence()
        core.start(pose, [('wait', 3)], self.now)
        core.update(pose, self.now + 0.05)
        self.ex.core = core
        self.ex.total = 1
        self.ex.feedback['paused'] = False
        phase = self.ex.phase_status(self.now + 0.05, pose, '')
        self.assertEqual(phase['code'], 'ESPERA')
        self.assertGreater(phase['remaining_s'], 2.9)

    def test_stabilizing_phase_reports_blocking_tolerances(self):
        pose = Pose(0, 0, 0.2, 0)
        core = Sequence()
        core.start(pose, [('turn', 10)], self.now)
        core.errors = (0.05, 0.01, 3.0)
        core.measured_speed = 0.04
        self.ex.core = core
        self.ex.total = 1
        phase = self.ex.phase_status(self.now + 0.05, pose, '')
        self.assertEqual(phase['code'], 'ESTABILIZANDO')
        self.assertIn('XY 0.05>0.03 m', phase['detail'])
        self.assertIn('vel. 0.04>0.03 m/s', phase['detail'])

    def test_stabilizing_phase_reports_confirmation_time(self):
        pose = Pose(0, 0, 0.2, 0)
        core = Sequence()
        core.start(pose, [('turn', 10)], self.now)
        core.errors = (0.01, 0.01, 3.0)
        core.measured_speed = 0.01
        core.inside_since = self.now
        self.ex.core = core
        self.ex.total = 1
        phase = self.ex.phase_status(self.now + 0.25, pose, '')
        self.assertEqual(phase['code'], 'ESTABILIZANDO')
        self.assertIn('confirmando 0.8 s', phase['detail'])

    def test_stabilizing_phase_handles_initial_check_without_timestamp(self):
        pose = Pose(0, 0, 0.2, 0)
        core = Sequence()
        core.start(pose, [('turn', 10)], self.now)
        core.errors = (0.01, 0.01, 3.0)
        core.measured_speed = 0.01
        self.assertIsNone(core.inside_since)
        self.ex.core = core
        self.ex.total = 1
        phase = self.ex.phase_status(self.now + 0.05, pose, '')
        self.assertEqual(phase['code'], 'ESTABILIZANDO')
        self.assertIn('verificando tolerancias', phase['detail'])

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
