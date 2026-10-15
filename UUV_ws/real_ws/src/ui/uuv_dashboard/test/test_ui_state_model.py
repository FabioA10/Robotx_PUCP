"""Tests for pure sequence-dashboard presentation rules."""

import unittest

from uuv_dashboard.ui_state_model import (
    controller_tuning_payload,
    operator_guidance,
    workflow_states,
)


def status(**changes):
    """Build a minimal executor status for presentation tests."""
    value = {
        'state': 'IDLE',
        'mode': 'MANUAL',
        'armed': False,
        'validated': False,
        'home': None,
        'home_ready': False,
        'navigation_ready': True,
        'enabled': True,
        'blockers': [],
        'phase': {'code': 'IDLE'},
    }
    value.update(changes)
    return value


class UiStateModelTests(unittest.TestCase):
    """Keep visual guidance consistent with the supervised workflow."""

    def test_disconnected_state_never_claims_abort_is_available(self):
        guide = operator_guidance(None, False, False, False)
        self.assertFalse(guide['abort_available'])
        self.assertEqual(guide['state'], 'warning')

    def test_empty_plan_keeps_configuration_active(self):
        states = workflow_states(status(), True, False, False)
        self.assertEqual(states['configure'], 'active')
        self.assertEqual(states['prepare'], 'neutral')

    def test_prepared_plan_requests_deliberate_arm(self):
        current = status(validated=True)
        guide = operator_guidance(current, True, True, True)
        self.assertIn('armar', guide['message'])
        self.assertEqual(workflow_states(current, True, True, True)['arm'], 'active')

    def test_navigation_blocker_prevents_guided_stage(self):
        current = status(
            validated=True,
            armed=True,
            navigation_ready=False,
            blockers=['DVL sin referencia'],
        )
        guide = operator_guidance(current, True, True, True)
        self.assertIn('DVL sin referencia', guide['reasons'])
        self.assertEqual(workflow_states(current, True, True, True)['guided'], 'blocked')

    def test_guided_ready_requests_home(self):
        current = status(validated=True, armed=True, mode='GUIDED')
        guide = operator_guidance(current, True, True, True)
        self.assertIn('fijar Home', guide['message'])
        states = workflow_states(current, True, True, True)
        self.assertEqual(states['home'], 'active')
        self.assertEqual(states['execute'], 'neutral')

    def test_guided_settling_blocks_home_capture(self):
        current = status(
            validated=True,
            armed=True,
            mode='GUIDED',
            blockers=['Estabilizando entrada a GUIDED'],
        )
        guide = operator_guidance(current, True, True, True)
        self.assertIn('GUIDED estable', guide['message'])
        self.assertEqual(workflow_states(current, True, True, True)['home'], 'blocked')

    def test_home_ready_requests_execution(self):
        current = status(
            validated=True,
            armed=True,
            mode='GUIDED',
            home={'n': 1.0, 'e': 2.0, 'depth': 0.2, 'yaw': 0.3},
            home_ready=True,
        )
        guide = operator_guidance(current, True, True, True)
        self.assertIn('6 · iniciar', guide['message'])
        self.assertEqual(guide['state'], 'success')
        states = workflow_states(current, True, True, True)
        self.assertEqual(states['home'], 'done')
        self.assertEqual(states['execute'], 'active')

    def test_running_sequence_keeps_abort_available(self):
        current = status(
            state='RUNNING',
            validated=True,
            armed=True,
            mode='GUIDED',
            home={'n': 0.0, 'e': 0.0, 'depth': 0.2, 'yaw': 0.0},
            home_ready=True,
        )
        guide = operator_guidance(current, True, True, True)
        self.assertTrue(guide['abort_available'])
        self.assertEqual(workflow_states(current, True, True, True)['execute'], 'active')

    def test_controller_tuning_uses_the_physical_executor_contract(self):
        tuning = controller_tuning_payload(0.03, 0.08, 0.08, 0.02, 2.0, 0.8, 1.2)
        self.assertEqual(
            tuning,
            {
                'xy_tolerance': 0.03,
                'recovery_radius': 0.08,
                'recovery_speed': 0.08,
                'recovery_min_speed': 0.02,
                'recovery_delay': 2.0,
                'position_gain': 0.8,
                'velocity_damping': 1.2,
            },
        )


if __name__ == '__main__':
    unittest.main()
