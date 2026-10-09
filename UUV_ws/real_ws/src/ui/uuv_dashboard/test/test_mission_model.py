"""Tests for reusable mission storage and fast plan editing."""

import json
from pathlib import Path
import tempfile
import unittest

from uuv_dashboard.mission_model import (
    delete_mission,
    duplicate_step,
    empty_document,
    mission_metrics,
    MissionStore,
    rectangle_waypoints,
    rename_mission,
    save_mission,
)


def relative_mission():
    return {
        'plan_mode': 'relative',
        'relative_steps': [
            {'kind': 'advance', 'value': 0.5, 'speed': 0.1},
            {'kind': 'wait', 'duration': 1.0},
        ],
        'absolute_steps': [],
    }


class TestMissionEditing(unittest.TestCase):

    def test_duplicate_inserts_an_independent_copy(self):
        original = relative_mission()['relative_steps']
        result, selected = duplicate_step(original, 0)
        self.assertEqual(selected, 1)
        self.assertEqual(result[0], result[1])
        result[1]['value'] = 0.8
        self.assertEqual(original[0]['value'], 0.5)

    def test_named_missions_rename_and_delete_without_overwrite(self):
        document = save_mission(empty_document(), 'Cuadrado', relative_mission())
        document = rename_mission(document, 'Cuadrado', 'Cuadrado 50 cm')
        self.assertIn('Cuadrado 50 cm', document['missions'])
        with self.assertRaises(ValueError):
            rename_mission(
                save_mission(document, 'Otra', relative_mission()),
                'Otra',
                'Cuadrado 50 cm',
            )
        document = delete_mission(document, 'Cuadrado 50 cm')
        self.assertEqual(document['missions'], {})

    def test_metrics_help_detect_long_plans(self):
        metrics = mission_metrics(relative_mission())
        self.assertEqual(metrics['steps'], 2)
        self.assertAlmostEqual(metrics['horizontal_distance_m'], 0.5)
        self.assertAlmostEqual(metrics['minimum_time_s'], 6.0)

    def test_rectangle_template_closes_at_origin(self):
        template = {
            'kind': 'absolute_waypoint',
            'x': 0.0,
            'y': 0.0,
            'depth': 0.3,
            'yaw_deg': 0.0,
            'speed': 0.1,
            'vertical_speed': 0.08,
            'yaw_speed_deg': 10.0,
            'tolerance_x': 0.04,
            'tolerance_y': 0.04,
            'tolerance_depth': 0.02,
            'tolerance_yaw_deg': 5.0,
            'dwell': 1.0,
        }
        points = rectangle_waypoints(template, 0.5, -0.4)
        self.assertEqual([(p['x'], p['y']) for p in points], [
            (0.5, 0.0), (0.5, -0.4), (0.0, -0.4), (0.0, 0.0),
        ])


class TestMissionStore(unittest.TestCase):

    def test_round_trip_is_atomic_and_private(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'missions.json'
            store = MissionStore(path)
            document = save_mission(store.load(), 'Prueba', relative_mission())
            stored = store.save(document)
            self.assertEqual(store.load(), stored)
            self.assertEqual(path.stat().st_mode & 0o777, 0o600)
            json.loads(path.read_text(encoding='utf-8'))

    def test_rejects_mixed_absolute_and_relative_lists(self):
        mission = relative_mission()
        mission['absolute_steps'] = [{'kind': 'wait', 'duration': 1.0}]
        with self.assertRaises(ValueError):
            save_mission(empty_document(), 'Inválida', mission)


if __name__ == '__main__':
    unittest.main()
