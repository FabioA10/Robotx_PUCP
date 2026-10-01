import unittest

import numpy as np
from ping360_filter.filter_core import (
    Component,
    filter_polar_grid,
    mask_to_laserscan_ranges,
    points_to_polar_grid,
    rotation_warmup_complete,
    select_contour_components,
    select_target_candidate,
    TemporalPointTracker,
)


def test_filter_preserves_compact_echo_outside_dead_zone():
    grid = np.full((100, 400), 40.0, dtype=np.float32)
    grid[:, :60] = 255.0
    grid[20:23, 155:160] = 230.0

    _, _, mask, components = filter_polar_grid(
        grid=grid,
        angle_step_deg=3.6,
        range_step_m=0.0125,
        min_range_m=0.75,
        max_range_m=4.5,
        minimum_intensity=100.0,
        residual_threshold=40.0,
        background_window_bins=31,
        min_component_pixels=4,
    )

    assert not mask[:, :60].any()
    assert mask[20:23, 155:160].any()
    assert len(components) == 1


def _component(x, y, bearing, intensity=220.0):
    return Component(
        indices=np.array([[0, 0], [0, 1], [1, 0], [1, 1]]),
        point_count=8,
        centroid_x=x,
        centroid_y=y,
        range_m=float(np.hypot(x, y)),
        bearing_deg=bearing,
        mean_intensity=intensity,
        max_intensity=intensity,
        angular_span_deg=7.2,
        radial_span_m=0.12,
        extent_m=0.18,
    )


def test_target_selection_respects_front_search_sector():
    front = _component(2.0, 0.2, 5.7, intensity=245.0)
    rear = _component(-1.0, 0.0, -180.0, intensity=255.0)

    selected, confidence = select_target_candidate(
        [rear, front],
        min_range_m=0.8,
        max_range_m=4.0,
        search_center_deg=0.0,
        search_half_width_deg=70.0,
        min_points=4,
        max_points=180,
        max_angular_span_deg=25.0,
        max_radial_span_m=0.5,
        max_extent_m=0.55,
    )

    assert selected is front
    assert confidence > 0.5


def test_mask_converts_to_nearest_range_per_bearing():
    mask = np.zeros((3, 5), dtype=bool)
    intensity = np.zeros((3, 5), dtype=np.float32)
    mask[0, 3] = True
    mask[0, 2] = True
    intensity[0, 2] = 200.0

    ranges, values = mask_to_laserscan_ranges(mask, intensity, 0.1)

    assert ranges[0] == np.float32(0.2)
    assert np.isinf(ranges[1])
    assert values[0] == np.float32(200.0 / 255.0)


def test_complete_cloud_grid_keeps_strongest_echo_and_ignores_invalid_points():
    ranges = np.array([1.0, 1.0, 2.0, np.nan], dtype=np.float32)
    angles = np.deg2rad(np.array([0.0, 0.1, 3.6, 180.0]))
    x = ranges * np.cos(angles)
    y = ranges * np.sin(angles)
    intensity = np.array([75.0, 210.0, 80.0, 255.0], dtype=np.float32)

    grid = points_to_polar_grid(
        x,
        y,
        intensity,
        angle_count=100,
        angle_step_deg=3.6,
        range_bin_count=400,
        range_step_m=0.01,
    )

    assert grid[0, 100] == np.float32(210.0)
    assert grid[1, 200] == np.float32(80.0)
    assert np.count_nonzero(grid) == 2


def test_contour_filter_ignores_returns_beyond_two_meter_radius():
    grid = np.full((100, 400), 45.0, dtype=np.float32)
    grid[10:14, 118:122] = 230.0  # 1.5 m: inside the configured radius.
    grid[50:54, 198:202] = 250.0  # 2.5 m: outside the configured radius.

    _, _, mask, _ = filter_polar_grid(
        grid=grid,
        angle_step_deg=3.6,
        range_step_m=0.0125,
        min_range_m=0.70,
        max_range_m=2.0,
        minimum_intensity=90.0,
        residual_threshold=45.0,
        background_window_bins=41,
        min_component_pixels=4,
    )

    assert mask[10:14, 118:122].any()
    assert not mask[50:54, 198:202].any()


def test_contour_warmup_uses_source_timestamp_span():
    assert not rotation_warmup_complete(10.0, 24.99, 15.0, 1)
    assert rotation_warmup_complete(10.0, 25.0, 15.0, 1)
    assert not rotation_warmup_complete(10.0, 25.0, 15.0, 2)
    assert not rotation_warmup_complete(float('nan'), 25.0, 15.0, 1)


def test_temporal_tracker_requires_independent_confirmations():
    tracker = TemporalPointTracker(0.3, confirmation_hits=3, maximum_misses=1)
    first = tracker.update((1.0, 0.2), quality=0.9)
    second = tracker.update((1.05, 0.18), quality=0.9)
    third = tracker.update((1.03, 0.20), quality=0.9)

    assert not first.valid
    assert not second.valid
    assert third.valid
    assert third.confidence > 0.8


def test_contour_component_selection_keeps_long_echo_arcs_only():
    angle_step = 3.6
    range_step = 0.0125
    grid = np.full((100, 400), 40.0, dtype=np.float32)
    for angle_index in range(100):
        angle = np.deg2rad(angle_index * angle_step)
        ux, uy = np.cos(angle), np.sin(angle)
        intersections = []
        if ux > 1e-9:
            intersections.append(2.0 / ux)
        elif ux < -1e-9:
            intersections.append(-2.0 / ux)
        if uy > 1e-9:
            intersections.append(1.0 / uy)
        elif uy < -1e-9:
            intersections.append(-1.0 / uy)
        wall_range = min(value for value in intersections if value > 0.0)
        radial_bin = int(round(wall_range / range_step))
        grid[angle_index, radial_bin - 1: radial_bin + 2] = 220.0

    # An isolated object and echoes beyond the maximum pool diagonal.
    grid[50:52, 250:252] = 255.0
    grid[:, int(round(4.8 / range_step))] = 255.0

    _, _, mask, components = filter_polar_grid(
        grid=grid,
        angle_step_deg=3.6,
        range_step_m=range_step,
        min_range_m=0.7,
        max_range_m=4.62,
        minimum_intensity=90.0,
        residual_threshold=45.0,
        background_window_bins=41,
        min_component_pixels=4,
    )
    selected_mask, selected = select_contour_components(
        mask, components, min_points=12, min_angular_span_deg=10.0
    )
    assert len(selected) >= 2
    assert np.count_nonzero(selected_mask) > 50
    assert all(c.point_count >= 12 and c.angular_span_deg >= 10.0 for c in selected)
    assert np.count_nonzero(selected_mask[:, int(round(4.8 / range_step))]) == 0


class TestFilterCore(unittest.TestCase):
    """Expose the checks to colcon's unittest-based Python test runner."""

    def test_filter_preserves_compact_echo_outside_dead_zone(self):
        test_filter_preserves_compact_echo_outside_dead_zone()

    def test_target_selection_respects_front_search_sector(self):
        test_target_selection_respects_front_search_sector()

    def test_mask_converts_to_nearest_range_per_bearing(self):
        test_mask_converts_to_nearest_range_per_bearing()

    def test_complete_cloud_grid_keeps_strongest_echo_and_ignores_invalid_points(self):
        test_complete_cloud_grid_keeps_strongest_echo_and_ignores_invalid_points()

    def test_contour_filter_ignores_returns_beyond_two_meter_radius(self):
        test_contour_filter_ignores_returns_beyond_two_meter_radius()

    def test_contour_warmup_uses_source_timestamp_span(self):
        test_contour_warmup_uses_source_timestamp_span()

    def test_temporal_tracker_requires_independent_confirmations(self):
        test_temporal_tracker_requires_independent_confirmations()

    def test_contour_component_selection_keeps_long_echo_arcs_only(self):
        test_contour_component_selection_keeps_long_echo_arcs_only()
