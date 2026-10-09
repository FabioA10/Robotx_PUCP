"""Tests for live owner control of the manual command scale."""

import unittest

from uuv_mavlink.manual_control_model import normalize_command_scale


class ManualControlModelTests(unittest.TestCase):
    """Accept the full operator range and reject malformed values."""

    def test_accepts_zero_middle_and_full_scale(self):
        self.assertEqual(normalize_command_scale(0.0), 0.0)
        self.assertEqual(normalize_command_scale(0.55), 0.55)
        self.assertEqual(normalize_command_scale(1.0), 1.0)

    def test_rejects_values_outside_range_or_not_finite(self):
        for value in (-0.01, 1.01, float('nan'), float('inf'), True, 'bad'):
            with self.assertRaises(ValueError, msg=repr(value)):
                normalize_command_scale(value)


if __name__ == '__main__':
    unittest.main()
