"""Tests for pure sequence-dashboard presentation rules."""

import unittest

from uuv_dashboard.ui_state_model import operator_guidance, workflow_states


def status(**changes):
    """Build a minimal executor status for presentation tests."""
    value = {
        'state': 'IDLE',
        'mode': 'MANUAL',
        'armed': False,
        'validated': False,
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

    def test_guided_ready_requests_execution(self):
        current = status(validated=True, armed=True, mode='GUIDED')
        guide = operator_guidance(current, True, True, True)
        self.assertIn('iniciar', guide['message'])
        self.assertEqual(guide['state'], 'success')

    def test_running_sequence_keeps_abort_available(self):
        current = status(
            state='RUNNING',
            validated=True,
            armed=True,
            mode='GUIDED',
        )
        guide = operator_guidance(current, True, True, True)
        self.assertTrue(guide['abort_available'])
        self.assertEqual(workflow_states(current, True, True, True)['execute'], 'active')


if __name__ == '__main__':
    unittest.main()
