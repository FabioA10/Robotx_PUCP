"""Unit tests for UAV geofence derivation, QGC .plan parsing, and RoboCommand validation."""

import json
import tempfile
import unittest

from robocommand.geofence import (
    derive_geofence_from_course,
    load_geofence_from_qgc_plan,
    validate_uav_geofence,
)

# Official Course Alpha & Garuda Transit A coordinates from robonation/robocommand test_server.py
COURSE_ALPHA = [
    (1.2809652, 103.8548424),
    (1.2808457, 103.8550331),
    (1.2810752, 103.8551770),
    (1.2811947, 103.8549863),
    (1.2809652, 103.8548424),
]
TRANSIT_A = [
    (1.281294119, 103.855048646),
    (1.281174598, 103.855239379),
    (1.2810752, 103.8551770),
    (1.2811947, 103.8549863),
    (1.281294119, 103.855048646),
]


class TestUavGeofence(unittest.TestCase):
    def test_auto_derive_merged_course_and_transit_passes_robocommand_validation(self):
        fence = derive_geofence_from_course(
            course_corners=COURSE_ALPHA,
            transit_corners=TRANSIT_A,
            include_transit=True,
            inset_m=0.5,
        )
        self.assertGreaterEqual(len(fence), 5)
        self.assertEqual(fence[0], fence[-1])

        problems = validate_uav_geofence(
            geofence=fence,
            max_altitude_hae_m=40.0,
            course_corners=COURSE_ALPHA,
            transit_corners=TRANSIT_A,
        )
        self.assertEqual(problems, [])

    def test_outside_geofence_or_invalid_ceiling_is_rejected(self):
        bad_fence = [
            (1.2800000, 103.8540000),
            (1.2800000, 103.8550000),
            (1.2810000, 103.8550000),
            (1.2800000, 103.8540000),
        ]
        problems = validate_uav_geofence(
            geofence=bad_fence,
            max_altitude_hae_m=60.0,
            course_corners=COURSE_ALPHA,
            transit_corners=TRANSIT_A,
        )
        self.assertEqual(len(problems), 2)
        self.assertIn("leaves the course and transit area", problems[0])
        self.assertIn("uav_geofence_max_altitude_hae_m", problems[1])

    def test_qgc_plan_loader_closes_ring_and_validates(self):
        qgc_plan = {
            "fileType": "Plan",
            "geoFence": {
                "polygons": [
                    {
                        "inclusion": True,
                        "polygon": [list(pt) for pt in COURSE_ALPHA[:-1]],
                        "version": 1,
                    }
                ],
                "version": 2,
            },
        }
        with tempfile.NamedTemporaryFile("w", suffix=".plan", delete=False) as f:
            json.dump(qgc_plan, f)
            plan_path = f.name

        fence = load_geofence_from_qgc_plan(plan_path)
        self.assertEqual(len(fence), 5)
        self.assertEqual(fence[0], fence[-1])

        problems = validate_uav_geofence(
            geofence=fence,
            max_altitude_hae_m=40.0,
            course_corners=COURSE_ALPHA,
            transit_corners=TRANSIT_A,
        )
        self.assertEqual(problems, [])


if __name__ == "__main__":
    unittest.main()
