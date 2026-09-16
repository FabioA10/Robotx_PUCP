"""Checks for the read-only pool-session gate."""

import unittest

from uuv_dashboard.pool_preflight_model import REQUIRED_CHECKS, evaluate


def complete_checks():
    return {name: True for name in REQUIRED_CHECKS}


class TestPoolPreflight(unittest.TestCase):
    def test_ready_only_when_all_read_only_conditions_are_met(self):
        ready, reasons = evaluate(
            complete_checks(), connected=True, armed=False, output_enabled=False)
        self.assertTrue(ready)
        self.assertEqual(reasons, [])

    def test_manual_and_ros_gates_block_the_session(self):
        ready, reasons = evaluate(
            {}, connected=False, armed=True, output_enabled=True)
        self.assertFalse(ready)
        self.assertEqual(len(reasons), 4)

    def test_video_is_required_only_when_operator_selects_it(self):
        checks = complete_checks()
        ready, _ = evaluate(
            checks, connected=True, armed=False, output_enabled=False,
            camera_required=False, camera_available=False)
        self.assertTrue(ready)
        ready, reasons = evaluate(
            checks, connected=True, armed=False, output_enabled=False,
            camera_required=True, camera_available=False)
        self.assertFalse(ready)
        self.assertIn('video reciente', reasons[0])


if __name__ == '__main__':
    unittest.main()
