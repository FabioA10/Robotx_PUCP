"""Tests for the GUI mode-request gate in the manual pool profile."""

import unittest

from uuv_dashboard.pool_operator_model import OPERATOR_MODES, mode_request_allowed


class TestPoolOperator(unittest.TestCase):
    def test_guided_is_an_explicit_supported_request(self):
        self.assertIn('GUIDED', OPERATOR_MODES)

    def test_only_disarmed_idle_connected_state_allows_request(self):
        good = dict(enabled=True, connected=True, armed=False, deadman=False)
        self.assertTrue(mode_request_allowed(**good))
        for key in good:
            blocked = dict(good)
            blocked[key] = True if key in ('armed', 'deadman') else False
            self.assertFalse(mode_request_allowed(**blocked))


if __name__ == '__main__':
    unittest.main()
