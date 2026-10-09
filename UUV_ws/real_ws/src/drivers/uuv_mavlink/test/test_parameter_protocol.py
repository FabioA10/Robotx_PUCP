"""Tests for the bounded ArduSub parameter protocol."""

import unittest

from uuv_mavlink.parameter_protocol import (
    normalize_observed_parameter_values,
    normalize_parameter_values,
    parameter_name,
    parameter_sets_match,
    parameter_type,
    write_blockers,
)


VALUES = {
    'MOT_PWM_MIN': 1350.0,
    'MOT_PWM_MAX': 1650.0,
    'WPNAV_SPEED_UP': 10.0,
    'WPNAV_SPEED_DN': 10.0,
    'WPNAV_ACCEL_Z': 50.0,
}


class ParameterProtocolTests(unittest.TestCase):
    """Keep arbitrary parameters and unsafe vehicle states outside the channel."""

    def test_accepts_only_complete_bounded_symmetric_values(self):
        self.assertEqual(normalize_parameter_values(VALUES), VALUES)
        with self.assertRaises(ValueError):
            normalize_parameter_values(dict(VALUES, MOT_PWM_MIN=1200.0))
        with self.assertRaises(ValueError):
            normalize_parameter_values(dict(VALUES, MOT_PWM_MAX=1660.0))
        with self.assertRaises(ValueError):
            normalize_parameter_values(dict(VALUES, ATC_RAT_YAW_P=1.0))

    def test_readback_comparison_has_small_float_tolerance(self):
        readback = dict(VALUES, WPNAV_ACCEL_Z=50.02)
        self.assertTrue(parameter_sets_match(VALUES, readback))
        self.assertFalse(parameter_sets_match(VALUES, dict(VALUES, WPNAV_ACCEL_Z=50.2)))

    def test_legacy_readback_is_visible_but_cannot_be_requested(self):
        legacy = dict(VALUES, WPNAV_ACCEL_Z=20.0)
        self.assertEqual(normalize_observed_parameter_values(legacy), legacy)
        with self.assertRaises(ValueError):
            normalize_parameter_values(legacy)

    def test_mavlink_parameter_type_is_validated(self):
        self.assertEqual(parameter_type(9), 9)
        with self.assertRaises(ValueError):
            parameter_type(0)

    def test_mavlink_parameter_identifier_is_normalized(self):
        self.assertEqual(parameter_name(b'MOT_PWM_MIN\0\0'), 'MOT_PWM_MIN')

    def test_every_unsafe_state_produces_a_blocker(self):
        safe = {
            'write_enabled': True,
            'connected': True,
            'heartbeat_fresh': True,
            'armed': False,
            'mode': 'MANUAL',
            'deadman': False,
            'sequence_active': False,
        }
        self.assertEqual(write_blockers(**safe), [])
        unsafe_cases = {
            'write_enabled': False,
            'connected': False,
            'heartbeat_fresh': False,
            'armed': True,
            'mode': 'GUIDED',
            'deadman': True,
            'sequence_active': True,
        }
        for key, value in unsafe_cases.items():
            current = dict(safe, **{key: value})
            self.assertTrue(write_blockers(**current), key)


if __name__ == '__main__':
    unittest.main()
