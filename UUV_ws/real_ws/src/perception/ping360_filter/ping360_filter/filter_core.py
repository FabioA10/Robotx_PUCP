"""NumPy-only Ping360 filtering and compact-target tracking functions."""

from collections import deque
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

import numpy as np


@dataclass
class Component:
    """Properties of one connected echo region in the polar image."""

    indices: np.ndarray
    point_count: int
    centroid_x: float
    centroid_y: float
    range_m: float
    bearing_deg: float
    mean_intensity: float
    max_intensity: float
    angular_span_deg: float
    radial_span_m: float
    extent_m: float


@dataclass
class TrackedPoint:
    """Target state after temporal association."""

    x: float
    y: float
    confidence: float
    hits: int
    misses: int
    valid: bool


def wrap_degrees(angle_deg: float) -> float:
    """Wrap an angle to [-180, 180)."""
    return float((angle_deg + 180.0) % 360.0 - 180.0)


def points_to_polar_grid(
    x: np.ndarray,
    y: np.ndarray,
    intensity: np.ndarray,
    angle_count: int,
    angle_step_deg: float,
    range_bin_count: int,
    range_step_m: float,
) -> np.ndarray:
    """Rasterize a complete XY point cloud, retaining the strongest echo per bin."""
    x = np.asarray(x, dtype=np.float32)
    y = np.asarray(y, dtype=np.float32)
    intensity = np.asarray(intensity, dtype=np.float32)
    ranges = np.hypot(x, y)
    finite = np.isfinite(x) & np.isfinite(y) & np.isfinite(intensity)
    valid = finite & (ranges > float(range_step_m) * 0.5)

    grid = np.zeros((int(angle_count), int(range_bin_count)), dtype=np.float32)
    if not np.any(valid):
        return grid

    angle_indices = np.rint(
        np.degrees(np.arctan2(y[valid], x[valid])) % 360.0 / float(angle_step_deg)
    ).astype(np.int32) % int(angle_count)
    range_indices = np.rint(ranges[valid] / float(range_step_m)).astype(np.int32)
    in_grid = (range_indices >= 0) & (range_indices < int(range_bin_count))
    np.maximum.at(
        grid,
        (angle_indices[in_grid], range_indices[in_grid]),
        intensity[valid][in_grid],
    )
    return grid


def rotation_warmup_complete(
    first_stamp_sec: float,
    current_stamp_sec: float,
    rotation_period_sec: float,
    warmup_rotations: int,
) -> bool:
    """Check whether source timestamps span the configured warmup rotations."""
    first_stamp_sec = float(first_stamp_sec)
    current_stamp_sec = float(current_stamp_sec)
    required_sec = max(0, int(warmup_rotations)) * max(0.0, float(rotation_period_sec))
    if not np.isfinite(first_stamp_sec) or not np.isfinite(current_stamp_sec):
        return False
    return current_stamp_sec >= first_stamp_sec + required_sec


def polar_median_3x3(grid: np.ndarray) -> np.ndarray:
    """Apply a 3x3 median, wrapping angle and clamping range."""
    samples = []
    for angle_shift in (-1, 0, 1):
        angular = np.roll(grid, angle_shift, axis=0)
        padded = np.pad(angular, ((0, 0), (1, 1)), mode='edge')
        for radial_offset in (0, 1, 2):
            samples.append(padded[:, radial_offset: radial_offset + grid.shape[1]])
    return np.median(np.stack(samples, axis=0), axis=0).astype(np.float32)


def radial_median_background(grid: np.ndarray, window_bins: int) -> np.ndarray:
    """Estimate slow radial background independently along every ray."""
    window_bins = max(3, int(window_bins))
    if window_bins % 2 == 0:
        window_bins += 1
    radius = window_bins // 2
    padded = np.pad(grid, ((0, 0), (radius, radius)), mode='edge')
    windows = np.lib.stride_tricks.sliding_window_view(padded, window_shape=window_bins, axis=1)
    return np.median(windows, axis=-1).astype(np.float32)


def connected_components_wrap(mask: np.ndarray, min_pixels: int):
    """Find 8-connected components with angular wrap at 0/360 degrees."""
    angle_count, range_count = mask.shape
    visited = np.zeros_like(mask, dtype=bool)
    components = []

    for start_angle, start_range in np.argwhere(mask):
        if visited[start_angle, start_range]:
            continue
        queue = deque([(int(start_angle), int(start_range))])
        visited[start_angle, start_range] = True
        indices = []
        while queue:
            angle_index, range_index = queue.popleft()
            indices.append((angle_index, range_index))
            for angle_delta in (-1, 0, 1):
                for range_delta in (-1, 0, 1):
                    if angle_delta == 0 and range_delta == 0:
                        continue
                    neighbour_angle = (angle_index + angle_delta) % angle_count
                    neighbour_range = range_index + range_delta
                    if not 0 <= neighbour_range < range_count:
                        continue
                    if (
                        mask[neighbour_angle, neighbour_range]
                        and not visited[neighbour_angle, neighbour_range]
                    ):
                        visited[neighbour_angle, neighbour_range] = True
                        queue.append((neighbour_angle, neighbour_range))
        if len(indices) >= min_pixels:
            components.append(np.asarray(indices, dtype=np.int32))
    return components


def _circular_span_degrees(angle_indices: np.ndarray, angle_step_deg: float) -> float:
    unique = np.unique(angle_indices)
    if unique.size <= 1:
        return angle_step_deg
    angle_count = int(round(360.0 / angle_step_deg))
    ordered = np.sort(unique)
    gaps = np.diff(np.concatenate([ordered, ordered[:1] + angle_count]))
    covered_steps = angle_count - int(np.max(gaps)) + 1
    return float(covered_steps * angle_step_deg)


def describe_component(
    indices: np.ndarray,
    intensity_grid: np.ndarray,
    angle_step_deg: float,
    range_step_m: float,
) -> Component:
    angle_indices = indices[:, 0]
    range_indices = indices[:, 1]
    angles = np.deg2rad(angle_indices * angle_step_deg)
    ranges = range_indices * range_step_m
    intensities = intensity_grid[angle_indices, range_indices]
    x = ranges * np.cos(angles)
    y = ranges * np.sin(angles)
    weights = np.maximum(intensities, 1.0)
    centroid_x = float(np.average(x, weights=weights))
    centroid_y = float(np.average(y, weights=weights))
    return Component(
        indices=indices,
        point_count=int(indices.shape[0]),
        centroid_x=centroid_x,
        centroid_y=centroid_y,
        range_m=float(np.hypot(centroid_x, centroid_y)),
        bearing_deg=wrap_degrees(float(np.degrees(np.arctan2(centroid_y, centroid_x)))),
        mean_intensity=float(np.mean(intensities)),
        max_intensity=float(np.max(intensities)),
        angular_span_deg=_circular_span_degrees(angle_indices, angle_step_deg),
        radial_span_m=float(np.ptp(range_indices) * range_step_m),
        extent_m=float(np.hypot(np.ptp(x), np.ptp(y))),
    )


def filter_polar_grid(
    grid: np.ndarray,
    angle_step_deg: float,
    range_step_m: float,
    min_range_m: float,
    max_range_m: float,
    minimum_intensity: float,
    residual_threshold: float,
    background_window_bins: int,
    min_component_pixels: int,
):
    """Return smoothed intensity, residual, mask and echo components."""
    smoothed = polar_median_3x3(grid)
    background = radial_median_background(smoothed, background_window_bins)
    residual = smoothed - background
    ranges = np.arange(grid.shape[1], dtype=np.float32) * range_step_m
    valid_range = (ranges >= min_range_m) & (ranges <= max_range_m)
    candidate_mask = (
        valid_range[np.newaxis, :]
        & (smoothed >= minimum_intensity)
        & (residual >= residual_threshold)
    )
    raw_components = connected_components_wrap(candidate_mask, min_component_pixels)
    final_mask = np.zeros_like(candidate_mask)
    components = []
    for indices in raw_components:
        final_mask[indices[:, 0], indices[:, 1]] = True
        components.append(describe_component(indices, smoothed, angle_step_deg, range_step_m))
    components.sort(key=lambda component: component.mean_intensity, reverse=True)
    return smoothed, residual, final_mask, components


def select_target_candidate(
    components: Sequence[Component],
    min_range_m: float,
    max_range_m: float,
    search_center_deg: float,
    search_half_width_deg: float,
    min_points: int,
    max_points: int,
    max_angular_span_deg: float,
    max_radial_span_m: float,
    max_extent_m: float,
):
    """Select a compact, bright and unique echo inside the search sector."""
    compatible = []
    for component in components:
        if not min_range_m <= component.range_m <= max_range_m:
            continue
        bearing_error = abs(wrap_degrees(component.bearing_deg - search_center_deg))
        if bearing_error > search_half_width_deg:
            continue
        if not min_points <= component.point_count <= max_points:
            continue
        if component.angular_span_deg > max_angular_span_deg:
            continue
        if component.radial_span_m > max_radial_span_m:
            continue
        if component.extent_m > max_extent_m:
            continue

        compactness = component.point_count / max(component.extent_m, 0.05)
        centrality = 1.0 - bearing_error / max(search_half_width_deg, 1.0)
        score = (
            component.mean_intensity / 255.0
            + 0.12 * np.log1p(compactness)
            + 0.05 * np.log1p(component.point_count)
            + 0.08 * centrality
        )
        compatible.append((float(score), component))

    if not compatible:
        return None, 0.0
    compatible.sort(key=lambda item: item[0], reverse=True)
    best_score, best_component = compatible[0]
    second_score = compatible[1][0] if len(compatible) > 1 else best_score - 0.20
    strength = float(np.clip((best_score - 1.10) / 0.60, 0.0, 1.0))
    uniqueness = float(np.clip((best_score - second_score) / 0.15, 0.0, 1.0))
    confidence = strength * uniqueness
    return best_component, confidence


def select_contour_components(
    mask: np.ndarray,
    components: Sequence[Component],
    min_points: int,
    min_angular_span_deg: float,
):
    """Keep broad, connected echo arcs as local pool-contour candidates."""
    selected = [
        component
        for component in components
        if component.point_count >= int(min_points)
        and component.angular_span_deg >= float(min_angular_span_deg)
    ]
    selected_mask = np.zeros_like(mask, dtype=bool)
    for component in selected:
        selected_mask[component.indices[:, 0], component.indices[:, 1]] = True
    return selected_mask, selected


def mask_to_laserscan_ranges(
    mask: np.ndarray,
    intensity_grid: np.ndarray,
    range_step_m: float,
) -> Tuple[np.ndarray, np.ndarray]:
    """Return the nearest filtered echo and its intensity for every bearing."""
    ranges = np.full(mask.shape[0], np.inf, dtype=np.float32)
    intensities = np.zeros(mask.shape[0], dtype=np.float32)
    for angle_index in range(mask.shape[0]):
        radial_indices = np.flatnonzero(mask[angle_index])
        if radial_indices.size == 0:
            continue
        radial_index = int(radial_indices[0])
        ranges[angle_index] = radial_index * range_step_m
        intensities[angle_index] = intensity_grid[angle_index, radial_index] / 255.0
    return ranges, intensities


class TemporalPointTracker:
    """Confirm a point only after independent scans agree."""

    def __init__(
        self,
        association_distance_m: float,
        confirmation_hits: int,
        maximum_misses: int,
        smoothing_alpha: float = 0.45,
        minimum_valid_confidence: float = 0.55,
    ):
        self.association_distance_m = float(association_distance_m)
        self.confirmation_hits = max(1, int(confirmation_hits))
        self.maximum_misses = max(0, int(maximum_misses))
        self.smoothing_alpha = float(np.clip(smoothing_alpha, 0.0, 1.0))
        self.minimum_valid_confidence = float(np.clip(minimum_valid_confidence, 0.0, 1.0))
        self.reset()

    def reset(self):
        self._active = False
        self._x = 0.0
        self._y = 0.0
        self._quality = 0.0
        self._hits = 0
        self._misses = 0

    def update(
        self,
        observation: Optional[Tuple[float, float]],
        quality: float = 0.0,
    ):
        if observation is None:
            if self._active:
                self._misses += 1
                if self._misses > self.maximum_misses:
                    self.reset()
            return self.state()

        observed_x, observed_y = map(float, observation)
        quality = float(np.clip(quality, 0.0, 1.0))
        if not self._active:
            self._x = observed_x
            self._y = observed_y
            self._quality = quality
            self._hits = 1
            self._misses = 0
            self._active = True
            return self.state()

        distance = float(np.hypot(observed_x - self._x, observed_y - self._y))
        if distance > self.association_distance_m:
            self._x = observed_x
            self._y = observed_y
            self._quality = quality
            self._hits = 1
            self._misses = 0
            return self.state()

        alpha = self.smoothing_alpha
        self._x = (1.0 - alpha) * self._x + alpha * observed_x
        self._y = (1.0 - alpha) * self._y + alpha * observed_y
        self._quality = (1.0 - alpha) * self._quality + alpha * quality
        self._hits += 1
        self._misses = 0
        return self.state()

    def state(self) -> Optional[TrackedPoint]:
        if not self._active:
            return None
        hit_factor = min(1.0, self._hits / self.confirmation_hits)
        confidence = float(np.clip(self._quality * hit_factor, 0.0, 1.0))
        return TrackedPoint(
            x=self._x,
            y=self._y,
            confidence=confidence,
            hits=self._hits,
            misses=self._misses,
            valid=(
                self._hits >= self.confirmation_hits
                and self._misses == 0
                and confidence >= self.minimum_valid_confidence
            ),
        )
