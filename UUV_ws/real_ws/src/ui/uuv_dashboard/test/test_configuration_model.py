"""Tests for persistent dashboard profiles without ROS or Qt."""

import json
from pathlib import Path
import tempfile
import unittest

from uuv_dashboard.configuration_model import (
    CONFIG_VERSION,
    ConfigurationStore,
    default_document,
    default_profile,
    delete_profile,
    normalize_document,
    normalize_profile,
    profile_to_raw_parameters,
    raw_parameters_match,
    raw_parameters_to_vehicle,
    rename_profile,
    set_vehicle_snapshot,
    snapshot_matches_vehicle,
    update_profile,
)


class ConfigurationModelTests(unittest.TestCase):
    """Protect units, ranges and atomic persistence."""

    def test_conservative_profile_maps_to_verified_ardusub_values(self):
        values = profile_to_raw_parameters(default_profile())
        self.assertEqual(
            values,
            {
                'MOT_PWM_MIN': 1350.0,
                'MOT_PWM_MAX': 1650.0,
                'WPNAV_SPEED_UP': 10.0,
                'WPNAV_SPEED_DN': 10.0,
                'WPNAV_ACCEL_Z': 50.0,
            },
        )
        vehicle = raw_parameters_to_vehicle(values)
        self.assertEqual(vehicle['propulsion_signal_limit_pct'], 37.5)
        self.assertEqual(vehicle['max_vertical_accel_mps2'], 0.50)
        self.assertEqual(default_profile()['display']['sonar_source'], 'filtered')

    def test_profile_rejects_inconsistent_recovery_limits(self):
        profile = default_profile()
        profile['controller']['recovery_min_speed_mps'] = 0.09
        with self.assertRaises(ValueError):
            normalize_profile(profile)

    def test_raw_comparison_rejects_asymmetric_pwm(self):
        desired = profile_to_raw_parameters(default_profile())
        asymmetric = dict(desired, MOT_PWM_MAX=1640.0)
        self.assertFalse(raw_parameters_match(desired, asymmetric))

    def test_store_reopens_active_profile_and_snapshot(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'profiles.json'
            store = ConfigurationStore(path)
            document = update_profile(default_document(), 'Laguna', default_profile())
            snapshot = {
                'MOT_PWM_MIN': 1300.0,
                'MOT_PWM_MAX': 1700.0,
                'WPNAV_SPEED_UP': 18.0,
                'WPNAV_SPEED_DN': 18.0,
                'WPNAV_ACCEL_Z': 50.0,
            }
            vehicle = {
                'system_id': 1,
                'component_id': 1,
                'firmware': 'ArduSub 4.5.7',
            }
            document = set_vehicle_snapshot(
                document,
                snapshot,
                vehicle=vehicle,
                captured_at='2026-10-08T14:00:00+00:00',
            )
            store.save(document)
            loaded = store.load()
            self.assertEqual(loaded['active_profile'], 'Laguna')
            self.assertEqual(loaded['last_vehicle_snapshot']['values'], snapshot)
            self.assertTrue(
                snapshot_matches_vehicle(loaded['last_vehicle_snapshot'], vehicle)
            )
            with path.open('r', encoding='utf-8') as stream:
                self.assertEqual(json.load(stream)['version'], CONFIG_VERSION)

    def test_v1_profile_is_migrated_to_documented_acceleration_floor(self):
        old = default_document()
        old['version'] = 1
        old['profiles']['Piscina conservador']['vehicle'][
            'max_vertical_accel_mps2'
        ] = 0.20
        migrated = normalize_document(old)
        self.assertEqual(
            migrated['profiles']['Piscina conservador']['vehicle'][
                'max_vertical_accel_mps2'
            ],
            0.50,
        )

    def test_profiles_can_be_renamed_and_deleted_without_silent_overwrite(self):
        document = update_profile(default_document(), 'Laguna', default_profile())
        document = rename_profile(document, 'Laguna', 'Laguna tranquila')
        self.assertIn('Laguna tranquila', document['profiles'])
        with self.assertRaises(ValueError):
            rename_profile(document, 'Laguna tranquila', 'Piscina conservador')
        document = delete_profile(document, 'Laguna tranquila')
        self.assertEqual(document['active_profile'], 'Piscina conservador')

    def test_restore_requires_exact_confirmed_vehicle_identity(self):
        snapshot = set_vehicle_snapshot(
            default_document(),
            profile_to_raw_parameters(default_profile()),
            vehicle={
                'system_id': 1,
                'component_id': 1,
                'firmware': 'ArduSub 4.5.7',
            },
        )['last_vehicle_snapshot']
        self.assertTrue(
            snapshot_matches_vehicle(
                snapshot,
                {'system_id': 1, 'component_id': 1, 'firmware': 'ArduSub 4.5.7'},
            )
        )
        self.assertFalse(
            snapshot_matches_vehicle(
                snapshot,
                {'system_id': 1, 'component_id': 1, 'firmware': 'ArduSub 4.5.8'},
            )
        )


if __name__ == '__main__':
    unittest.main()
