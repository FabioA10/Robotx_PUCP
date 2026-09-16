"""Geometry and freshness checks independent of ROS and vehicle hardware."""

import math
import unittest

from uuv_dashboard.pool_model import (
    Pose, Telemetry, ekf_description, home_steps, preview, wrap_deg,
)


class TestPoolModel(unittest.TestCase):
    def test_relative_advance_after_clockwise_turn(self):
        result = preview(Pose(10, 20, 2, 0),
                         [('advance', 1), ('turn', 90), ('advance', 2)])[-1]
        self.assertAlmostEqual(result.north, 11)
        self.assertAlmostEqual(result.east, 22)
        self.assertEqual(result.depth, 2)

    def test_depth_is_absolute_not_incremental(self):
        result = preview(Pose(0, 0, 2, 0), [('depth', 3)])[-1]
        self.assertEqual(result.depth, 3)

    def test_heading_wrap_and_new_reference(self):
        self.assertEqual(wrap_deg(-179 - 179), 2)
        self.assertEqual(preview(Pose(0, 0, 0, 179), [('turn', 2)])[-1].yaw, -179)
        first = preview(Pose(0, 0, 0, 0), [('advance', 1)])[-1]
        second = preview(Pose(10, 10, 0, 90), [('advance', 1)])[-1]
        self.assertEqual(first.north, 1)
        self.assertAlmostEqual(second.north, 10)
        self.assertAlmostEqual(second.east, 11)

    def test_home_uses_shortest_turn_and_restores_depth_heading(self):
        current = Pose(4, 3, 1, 170)
        home = Pose(0, 0, 2, -170)
        steps = home_steps(current, home)
        self.assertEqual(steps[0], ('depth', 2))
        self.assertTrue(all(abs(v) <= 180 for k, v in steps if k == 'turn'))
        result = preview(current, steps)[-1]
        self.assertAlmostEqual(result.north, home.north)
        self.assertAlmostEqual(result.east, home.east)
        self.assertEqual(result.depth, home.depth)
        self.assertAlmostEqual(result.yaw, home.yaw)

    def test_home_same_xy_has_no_unnecessary_travel(self):
        steps = home_steps(Pose(0, 0, 1, 0), Pose(0, 0, 3, 90))
        self.assertEqual(steps, [('depth', 3), ('turn', 90)])

    def test_invalid_values_cannot_be_previewed(self):
        for step in [('depth', -1), ('wait', -1), ('turn', math.nan), ('bad', 1)]:
            with self.assertRaises(ValueError):
                preview(Pose(0, 0, 0, 0), [step])
        with self.assertRaises(ValueError):
            Pose(math.inf, 0, 0, 0)

    def test_freshness_rejects_stale_nan_and_clock_reversal(self):
        now = [10.0]
        data = Telemetry(clock=lambda: now[0])
        self.assertIsNone(data.get('position'))
        data.put('position', (1, 2, 3))
        self.assertEqual(data.get('position'), (1, 2, 3))
        now[0] = 13.0
        self.assertIsNone(data.get('position'))
        now[0] = 9.0
        self.assertIsNone(data.get('position'))
        data.put('position', (1, math.nan, 3))
        self.assertIsNone(data.get('position'))

    def test_bench_flags_do_not_claim_horizontal_position(self):
        self.assertIn('posición horizontal no válida', ekf_description(39))
        self.assertIn('posición constante', ekf_description(167))
        self.assertIn('aún por comprobar', ekf_description(1 | 2 | 8))


if __name__ == '__main__':
    unittest.main()
