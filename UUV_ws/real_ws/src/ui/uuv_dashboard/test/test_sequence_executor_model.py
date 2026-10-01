"""Unit tests for the default-disabled sequence guard."""

import unittest

from uuv_dashboard.sequence_executor_model import (
    ExecutionGuard,
    Limits,
    move_step,
    replace_step,
    validate_steps,
)


class TestPlanEditing(unittest.TestCase):
    def test_replaces_selected_step_without_mutating_original(self):
        original = [
            {'kind': 'advance', 'value': 1.0, 'speed': 0.2},
            {'kind': 'wait', 'duration': 2.0},
        ]
        updated = replace_step(original, 0, {'kind': 'advance', 'value': -0.5, 'speed': 0.3})
        self.assertEqual(original[0]['value'], 1.0)
        self.assertEqual(updated[0]['value'], -0.5)
        self.assertEqual(updated[1], original[1])

    def test_moves_selected_step_and_reports_new_index(self):
        original = [{'kind': 'advance'}, {'kind': 'turn'}, {'kind': 'wait'}]
        updated, selected = move_step(original, 1, -1)
        self.assertEqual([step['kind'] for step in updated], ['turn', 'advance', 'wait'])
        self.assertEqual(selected, 0)
        self.assertEqual([step['kind'] for step in original], ['advance', 'turn', 'wait'])

    def test_move_at_boundary_leaves_order_unchanged(self):
        original = [{'kind': 'advance'}, {'kind': 'wait'}]
        updated, selected = move_step(original, 0, -1)
        self.assertEqual(updated, original)
        self.assertEqual(selected, 0)

    def test_editing_rejects_invalid_selection(self):
        with self.assertRaises(IndexError):
            replace_step([], 0, {'kind': 'wait'})
        with self.assertRaises(ValueError):
            move_step([{'kind': 'wait'}], 0, 2)


class TestSequenceValidation(unittest.TestCase):
    def test_normalizes_a_valid_plan(self):
        self.assertEqual(
            validate_steps([['depth', 1], ['advance', 2], ['turn', -90]]),
            [('depth', 1.0), ('advance', 2.0), ('turn', -90.0)],
        )

    def test_rejects_empty_and_unknown_plans(self):
        with self.assertRaises(ValueError):
            validate_steps([])
        with self.assertRaises(ValueError):
            validate_steps([['launch', 1]])

    def test_rejects_per_step_and_accumulated_limits(self):
        with self.assertRaises(ValueError):
            validate_steps([['advance', 2.1]])
        with self.assertRaises(ValueError):
            validate_steps([['advance', 2], ['advance', 2], ['advance', 2]])

    def test_rejects_invalid_depth_and_wait(self):
        with self.assertRaises(ValueError):
            validate_steps([['depth', -0.1]])
        with self.assertRaises(ValueError):
            validate_steps([['wait', 11]])
        with self.assertRaises(ValueError):
            validate_steps([['depth', 3.1]], Limits(max_depth_m=3.0))


class TestExecutionGuard(unittest.TestCase):
    def test_output_disabled_cannot_start(self):
        guard = ExecutionGuard()
        self.assertFalse(guard.start(10.0, output_enabled=False))
        self.assertEqual(guard.state, guard.CANCELLED)

    def test_watchdog_cancels_unresponsive_executor(self):
        guard = ExecutionGuard(heartbeat_timeout_s=0.5)
        self.assertTrue(guard.start(10.0, output_enabled=True))
        self.assertFalse(guard.watchdog(10.49))
        self.assertTrue(guard.watchdog(10.51))
        self.assertIn('dejó de responder', guard.reason)

    def test_gate_requires_all_runtime_conditions(self):
        guard = ExecutionGuard()
        guard.start(1.0, output_enabled=True)
        good = {
            'connected': True,
            'armed': True,
            'mode': 'GUIDED',
            'telemetry_ok': True,
            'failsafe_ok': True,
            'output_enabled': True,
        }
        self.assertTrue(guard.allowed(**good))
        for key in ('connected', 'armed', 'telemetry_ok', 'failsafe_ok', 'output_enabled'):
            blocked = dict(good)
            blocked[key] = False
            self.assertFalse(guard.allowed(**blocked))
        self.assertFalse(guard.allowed(**dict(good, mode='MANUAL')))

    def test_operator_cancellation_is_terminal(self):
        guard = ExecutionGuard()
        guard.start(1.0, output_enabled=True)
        guard.cancel('Cancelada por MANUAL')
        guard.heartbeat(2.0)
        self.assertEqual(guard.state, guard.CANCELLED)


if __name__ == '__main__':
    unittest.main()
