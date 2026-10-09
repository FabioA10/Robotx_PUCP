"""Hardware-independent tests for camera/light accessory safety helpers."""

import unittest

from uuv_mavlink.accessory_model import (
    accessory_blocker,
    button_mask,
    level_to_percent,
    normalize_light_step,
    percent_to_level,
)


class TestAccessoryModel(unittest.TestCase):

    def test_button_mask_uses_zero_based_mavlink_bit(self):
        self.assertEqual(button_mask(13), 1 << 13)
        self.assertEqual(button_mask(15), 32768)
        for invalid in (-1, 16, 1.5, True):
            with self.assertRaises(ValueError):
                button_mask(invalid)

    def test_light_percent_uses_ten_real_steps(self):
        self.assertEqual(percent_to_level(0, 10), 0)
        self.assertEqual(percent_to_level(50, 10), 5)
        self.assertEqual(percent_to_level(100, 10), 10)
        self.assertEqual(level_to_percent(7, 10), 70.0)

    def test_rejects_invalid_light_requests(self):
        for value in (-1, 101, float('nan')):
            with self.assertRaises(ValueError):
                percent_to_level(value, 10)
        for value in (-2, 0, 2, True):
            with self.assertRaises(ValueError):
                normalize_light_step(value)

    def test_gate_explains_the_actual_blocker(self):
        self.assertIn('deshabilitada', accessory_blocker(False, True))
        self.assertIn('heartbeat', accessory_blocker(True, False))
        self.assertEqual(accessory_blocker(True, True), '')


if __name__ == '__main__':
    unittest.main()
