"""Tests for the ROS-to-BlueOS transport state."""

import unittest
from unittest.mock import patch

from uuv_mavlink.embedded_protocol import (
    disconnected_status,
    initial_offline_status,
    normalise_agent_url,
)


class EmbeddedProtocolTests(unittest.TestCase):
    def test_normalises_plain_http_url(self):
        self.assertEqual(
            normalise_agent_url('http://192.168.2.2:9132/'),
            'http://192.168.2.2:9132',
        )

    def test_rejects_credentials_and_paths(self):
        for value in (
            'https://192.168.2.2:9132',
            'http://user:password@192.168.2.2:9132',
            'http://192.168.2.2:9132/v1',
        ):
            with self.subTest(value=value), self.assertRaises(ValueError):
                normalise_agent_url(value)

    def test_disconnection_does_not_invent_completion(self):
        status = initial_offline_status('session')
        status.update(state='RUNNING', step=2, total=4, enabled=True)
        result = disconnected_status(status, 'timeout', now=4.0)
        self.assertEqual(result['state'], 'RUNNING')
        self.assertEqual(result['step'], 2)
        self.assertFalse(result['connected'])
        self.assertFalse(result['agent_link'])
        self.assertEqual(result['phase']['code'], 'PAUSA')

    def test_cached_running_state_is_not_mutated(self):
        status = initial_offline_status('session')
        status.update(state='RUNNING', step=1, total=3)
        with patch('uuv_mavlink.embedded_protocol.time.monotonic', return_value=7.0):
            result = disconnected_status(status, 'wifi')
        self.assertEqual(status['connected'], False)
        self.assertNotIn('agent_link_error', status)
        self.assertEqual(result['agent_link_lost_at'], 7.0)


if __name__ == '__main__':
    unittest.main()
