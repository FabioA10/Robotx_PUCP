#!/usr/bin/env python3
"""Generate reviewable YOLO-OBB gate annotation candidates.

This module intentionally implements a conservative classical-vision
pre-annotator.  Its output is *not* ground truth: every label must be reviewed
before it is admitted to a training split.  The detector looks for a bright,
roughly rectangular gate body containing one or more dark rectangular
openings.  That internal-evidence requirement rejects most water glare and
white background objects.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import json
from pathlib import Path
from typing import Iterable, Sequence

import cv2
import numpy as np


CLASS_NAMES = ('gate', 'opening')
IMAGE_SUFFIXES = {'.jpg', '.jpeg', '.png', '.bmp', '.tif', '.tiff'}


@dataclass(frozen=True)
class AnnotatorConfig:
    """Thresholds for conservative gate and opening candidate generation."""

    white_max_saturation: int = 95
    white_min_value: int = 135
    dark_max_value: int = 125
    min_gate_area_ratio: float = 0.00020
    max_gate_area_ratio: float = 0.30
    min_gate_aspect: float = 0.38
    max_gate_aspect: float = 2.60
    min_gate_fill_ratio: float = 0.40
    min_opening_area_ratio: float = 0.006
    max_opening_area_ratio: float = 0.20
    min_opening_aspect: float = 0.28
    max_opening_aspect: float = 1.45
    min_opening_fill_ratio: float = 0.48
    minimum_openings: int = 1
    maximum_openings: int = 4
    processing_max_dimension: int = 1920
    min_opening_image_area_ratio: float = 0.000015
    max_opening_image_area_ratio: float = 0.030
    minimum_ring_contrast: float = 22.0
    min_pair_horizontal_ratio: float = 0.55
    max_pair_horizontal_ratio: float = 2.80
    min_pair_vertical_ratio: float = 0.15
    max_pair_vertical_ratio: float = 1.80


@dataclass(frozen=True)
class Detection:
    """One oriented-box candidate plus review diagnostics."""

    class_id: int
    class_name: str
    points_px: tuple[tuple[float, float], ...]
    confidence: float
    diagnostics: dict[str, float | int | str]


@dataclass(frozen=True)
class ImageAnnotations:
    """All candidates and image-level diagnostics for one source image."""

    image_path: str
    width: int
    height: int
    detections: tuple[Detection, ...]
    gate_candidates_before_evidence: int


@dataclass(frozen=True)
class _OpeningCandidate:
    """Internal dark-rectangle candidate used to form a two-opening gate."""

    rect: object
    points: np.ndarray
    center: np.ndarray
    vertical_axis: np.ndarray
    long_size: float
    short_size: float
    fill_ratio: float
    ring_contrast: float


def _ordered_box_points(rect) -> np.ndarray:
    """Return four min-area-rectangle corners in clockwise image order."""
    points = cv2.boxPoints(rect).astype(np.float32)
    center = points.mean(axis=0)
    angles = np.arctan2(points[:, 1] - center[1], points[:, 0] - center[0])
    ordered = points[np.argsort(angles)]
    start = int(np.argmin(ordered[:, 0] + ordered[:, 1]))
    return np.roll(ordered, -start, axis=0)


def _valid_box(rect, min_aspect: float, max_aspect: float) -> bool:
    width, height = rect[1]
    if width <= 1.0 or height <= 1.0:
        return False
    aspect = min(width, height) / max(width, height)
    return min_aspect <= aspect <= min(1.0, max_aspect)


def _rect_aspect(rect) -> float:
    width, height = rect[1]
    return min(width, height) / max(width, height)


def _box_tuple(rect) -> tuple[tuple[float, float], ...]:
    return tuple(tuple(float(value) for value in point)
                 for point in _ordered_box_points(rect))


def _long_axis(points: np.ndarray) -> tuple[np.ndarray, float, float]:
    """Return a downward-pointing long-edge unit vector and both box sizes."""
    edges = np.roll(points, -1, axis=0) - points
    lengths = np.linalg.norm(edges, axis=1)
    index = int(np.argmax(lengths))
    axis = edges[index] / max(lengths[index], 1.0e-6)
    if axis[1] < 0.0:
        axis = -axis
    return axis, float(lengths[index]), float(lengths[(index + 1) % 4])


def _global_opening_candidates(
    gray: np.ndarray,
    config: AnnotatorConfig,
) -> list[_OpeningCandidate]:
    """Find dark rectangles surrounded by a substantially brighter ring."""
    height, width = gray.shape
    image_area = float(width * height)
    dark = cv2.inRange(gray, 0, config.dark_max_value)
    dark = cv2.morphologyEx(
        dark, cv2.MORPH_CLOSE, np.ones((3, 3), dtype=np.uint8)
    )
    # RETR_LIST is intentional: an opening is a dark island nested inside the
    # bright facade, which itself is surrounded by a dark water background.
    contours, _ = cv2.findContours(
        dark, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE
    )
    candidates = []
    for contour in contours:
        contour_area = float(cv2.contourArea(contour))
        area_ratio = contour_area / image_area
        if not (
            config.min_opening_image_area_ratio
            <= area_ratio
            <= config.max_opening_image_area_ratio
        ):
            continue
        rect = cv2.minAreaRect(contour)
        if not _valid_box(
            rect, config.min_opening_aspect, config.max_opening_aspect
        ):
            continue
        rect_area = float(rect[1][0] * rect[1][1])
        fill_ratio = contour_area / max(rect_area, 1.0)
        if fill_ratio < config.min_opening_fill_ratio:
            continue

        contour_mask = np.zeros(gray.shape, dtype=np.uint8)
        cv2.drawContours(contour_mask, [contour], -1, 255, -1)
        ring_width = max(5, int(round(max(rect[1]) * 0.16)))
        if ring_width % 2 == 0:
            ring_width += 1
        ring = cv2.subtract(
            cv2.dilate(
                contour_mask,
                np.ones((ring_width, ring_width), dtype=np.uint8),
            ),
            contour_mask,
        )
        inside_value = float(cv2.mean(gray, mask=contour_mask)[0])
        ring_value = float(cv2.mean(gray, mask=ring)[0])
        contrast = ring_value - inside_value
        if contrast < config.minimum_ring_contrast:
            continue

        points = _ordered_box_points(rect)
        vertical_axis, long_size, short_size = _long_axis(points)
        candidates.append(_OpeningCandidate(
            rect=rect,
            points=points,
            center=np.asarray(rect[0], dtype=np.float32),
            vertical_axis=vertical_axis,
            long_size=long_size,
            short_size=short_size,
            fill_ratio=fill_ratio,
            ring_contrast=contrast,
        ))
    return candidates


def _pair_openings(
    candidates: list[_OpeningCandidate],
    config: AnnotatorConfig,
) -> list[tuple[_OpeningCandidate, _OpeningCandidate, float]]:
    """Greedily form disjoint upper-left/lower-right gate opening pairs."""
    proposals = []
    for first_index, first in enumerate(candidates):
        for second_index in range(first_index + 1, len(candidates)):
            second = candidates[second_index]
            left, right = sorted((first, second), key=lambda item: item.center[0])
            mean_long = 0.5 * (left.long_size + right.long_size)
            size_ratio = min(left.long_size, right.long_size) / max(
                left.long_size, right.long_size
            )
            if size_ratio < 0.58:
                continue
            axis_similarity = abs(float(np.dot(
                left.vertical_axis, right.vertical_axis
            )))
            if axis_similarity < 0.92:
                continue
            axis = left.vertical_axis + right.vertical_axis
            axis /= max(float(np.linalg.norm(axis)), 1.0e-6)
            if axis[1] < 0.0:
                axis = -axis
            horizontal = np.asarray((axis[1], -axis[0]), dtype=np.float32)
            delta = right.center - left.center
            horizontal_ratio = float(np.dot(delta, horizontal)) / mean_long
            vertical_ratio = float(np.dot(delta, axis)) / mean_long
            if not (
                config.min_pair_horizontal_ratio
                <= horizontal_ratio
                <= config.max_pair_horizontal_ratio
            ):
                continue
            if not (
                config.min_pair_vertical_ratio
                <= vertical_ratio
                <= config.max_pair_vertical_ratio
            ):
                continue
            geometry_score = 1.0 - min(1.0, abs(horizontal_ratio - 1.30) / 1.30)
            contrast_score = min(
                1.0,
                (left.ring_contrast + right.ring_contrast) / 100.0,
            )
            score = (
                0.35 * size_ratio
                + 0.25 * axis_similarity
                + 0.20 * geometry_score
                + 0.20 * contrast_score
            )
            proposals.append((score, first_index, second_index, left, right))

    selected = []
    used = set()
    for score, first_index, second_index, left, right in sorted(
        proposals, reverse=True, key=lambda item: item[0]
    ):
        if first_index in used or second_index in used:
            continue
        used.update((first_index, second_index))
        selected.append((left, right, score))
    selected.sort(key=lambda item: item[0].center[0])
    return selected


def _gate_points_from_pair(
    left: _OpeningCandidate,
    right: _OpeningCandidate,
) -> np.ndarray:
    """Infer a gate facade OBB from two aligned rectangular openings."""
    vertical = left.vertical_axis + right.vertical_axis
    vertical /= max(float(np.linalg.norm(vertical)), 1.0e-6)
    if vertical[1] < 0.0:
        vertical = -vertical
    horizontal = np.asarray((vertical[1], -vertical[0]), dtype=np.float32)
    points = np.vstack((left.points, right.points))
    horizontal_values = points @ horizontal
    vertical_values = points @ vertical
    horizontal_span = float(np.ptp(horizontal_values))
    vertical_span = float(np.ptp(vertical_values))
    horizontal_min = float(horizontal_values.min() - 0.18 * horizontal_span)
    horizontal_max = float(horizontal_values.max() + 0.18 * horizontal_span)
    vertical_min = float(vertical_values.min() - 0.16 * vertical_span)
    vertical_max = float(vertical_values.max() + 0.55 * vertical_span)
    return np.asarray([
        horizontal * horizontal_min + vertical * vertical_min,
        horizontal * horizontal_max + vertical * vertical_min,
        horizontal * horizontal_max + vertical * vertical_max,
        horizontal * horizontal_min + vertical * vertical_max,
    ], dtype=np.float32)


def annotate_image(
    image: np.ndarray,
    config: AnnotatorConfig = AnnotatorConfig(),
    image_path: str = '<memory>',
) -> ImageAnnotations:
    """Return conservative gate/opening OBB candidates for one BGR image."""
    if image is None or image.ndim != 3 or image.shape[2] != 3:
        raise ValueError('image must be a non-empty BGR array')
    height, width = image.shape[:2]
    if height < 16 or width < 16:
        raise ValueError('image is too small to annotate')

    scale = min(1.0, config.processing_max_dimension / max(width, height))
    if scale < 1.0:
        working = cv2.resize(
            image,
            (int(round(width * scale)), int(round(height * scale))),
            interpolation=cv2.INTER_AREA,
        )
    else:
        working = image
    gray = cv2.cvtColor(working, cv2.COLOR_BGR2GRAY)
    opening_candidates = _global_opening_candidates(gray, config)
    gate_pairs = _pair_openings(opening_candidates, config)
    detections: list[Detection] = []
    inverse_scale = 1.0 / scale
    for left, right, pair_score in gate_pairs:
        gate_points = _gate_points_from_pair(left, right) * inverse_scale
        detections.append(Detection(
            class_id=0,
            class_name='gate',
            points_px=tuple(tuple(float(value) for value in point)
                            for point in gate_points),
            confidence=float(np.clip(pair_score, 0.0, 0.99)),
            diagnostics={
                'opening_evidence': 2,
                'pair_score': pair_score,
                'review_status': 'candidate_only',
            },
        ))
        for opening in (left, right):
            opening_confidence = float(np.clip(
                0.35
                + 0.35 * opening.fill_ratio
                + 0.30 * min(1.0, opening.ring_contrast / 50.0),
                0.0,
                0.99,
            ))
            detections.append(Detection(
                class_id=1,
                class_name='opening',
                points_px=tuple(
                    tuple(float(value) for value in point)
                    for point in opening.points * inverse_scale
                ),
                confidence=opening_confidence,
                diagnostics={
                    'fill_ratio': opening.fill_ratio,
                    'aspect_ratio': (
                        opening.short_size / opening.long_size
                    ),
                    'ring_contrast': opening.ring_contrast,
                    'review_status': 'candidate_only',
                },
            ))

    detections.sort(key=lambda item: (
        item.class_id,
        sum(point[0] for point in item.points_px) / 4.0,
    ))
    return ImageAnnotations(
        image_path=image_path,
        width=width,
        height=height,
        detections=tuple(detections),
        gate_candidates_before_evidence=len(opening_candidates),
    )


def read_and_annotate(
    image_path: Path,
    config: AnnotatorConfig = AnnotatorConfig(),
) -> tuple[np.ndarray, ImageAnnotations]:
    """Load and annotate an image, raising a clear error if decoding fails."""
    image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError(f'cannot decode image: {image_path}')
    return image, annotate_image(image, config, str(image_path))


def yolo_obb_lines(annotations: ImageAnnotations) -> list[str]:
    """Serialize detections as Ultralytics YOLO-OBB normalized polygons."""
    lines = []
    for detection in annotations.detections:
        values = []
        for x_value, y_value in detection.points_px:
            x_normalized = float(np.clip(x_value / annotations.width, 0.0, 1.0))
            y_normalized = float(np.clip(y_value / annotations.height, 0.0, 1.0))
            values.extend((x_normalized, y_normalized))
        if len(values) != 8:
            raise ValueError('an oriented box must contain exactly four points')
        lines.append(
            str(detection.class_id)
            + ' '
            + ' '.join(f'{value:.8f}' for value in values)
        )
    return lines


def draw_overlay(image: np.ndarray, annotations: ImageAnnotations) -> np.ndarray:
    """Draw auditable class, confidence and OBB candidate overlays."""
    overlay = image.copy()
    colors = ((30, 220, 30), (30, 170, 255))
    for detection in annotations.detections:
        points = np.rint(detection.points_px).astype(np.int32)
        color = colors[detection.class_id]
        cv2.polylines(overlay, [points], True, color, 3, cv2.LINE_AA)
        x_value, y_value = points[0]
        label = f'{detection.class_name} {detection.confidence:.2f} REVIEW'
        cv2.putText(
            overlay,
            label,
            (max(0, x_value), max(20, y_value - 5)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.60,
            color,
            2,
            cv2.LINE_AA,
        )
    return overlay


def _input_images(input_path: Path) -> Iterable[Path]:
    if input_path.is_file():
        if input_path.suffix.lower() not in IMAGE_SUFFIXES:
            raise ValueError(f'unsupported image suffix: {input_path.suffix}')
        yield input_path
        return
    if not input_path.is_dir():
        raise ValueError(f'input does not exist: {input_path}')
    for path in sorted(input_path.iterdir()):
        if path.is_file() and path.suffix.lower() in IMAGE_SUFFIXES:
            yield path


def _diagnostic_record(annotations: ImageAnnotations) -> dict:
    record = asdict(annotations)
    record['detections'] = [asdict(item) for item in annotations.detections]
    record['review_required'] = True
    record['schema'] = 'robotx.gate_candidates.v1'
    return record


def process_paths(
    input_path: Path,
    output_path: Path,
    config: AnnotatorConfig,
    write_overlays: bool,
) -> dict[str, int]:
    """Pre-annotate one image or directory into a review-only artifact tree."""
    labels_path = output_path / 'labels_candidates'
    diagnostics_path = output_path / 'diagnostics'
    overlays_path = output_path / 'overlays'
    labels_path.mkdir(parents=True, exist_ok=True)
    diagnostics_path.mkdir(parents=True, exist_ok=True)
    if write_overlays:
        overlays_path.mkdir(parents=True, exist_ok=True)

    counts = {'images': 0, 'gates': 0, 'openings': 0, 'empty': 0}
    for image_path in _input_images(input_path):
        image, annotations = read_and_annotate(image_path, config)
        label_file = labels_path / f'{image_path.stem}.txt'
        label_file.write_text(
            '\n'.join(yolo_obb_lines(annotations))
            + ('\n' if annotations.detections else ''),
            encoding='utf-8',
        )
        diagnostic_file = diagnostics_path / f'{image_path.stem}.json'
        diagnostic_file.write_text(
            json.dumps(_diagnostic_record(annotations), indent=2),
            encoding='utf-8',
        )
        if write_overlays:
            cv2.imwrite(
                str(overlays_path / f'{image_path.stem}.jpg'),
                draw_overlay(image, annotations),
                [cv2.IMWRITE_JPEG_QUALITY, 92],
            )
        counts['images'] += 1
        counts['gates'] += sum(
            detection.class_id == 0 for detection in annotations.detections
        )
        counts['openings'] += sum(
            detection.class_id == 1 for detection in annotations.detections
        )
        counts['empty'] += not annotations.detections
    return counts


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """Parse the pre-annotation command line."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=Path, help='Image file or flat image directory')
    parser.add_argument('output', type=Path, help='New review-artifact directory')
    parser.add_argument(
        '--overlays', action='store_true', help='Write REVIEW-marked JPEG overlays'
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    """Run candidate generation and print a machine-readable summary."""
    args = parse_args(argv)
    counts = process_paths(
        args.input,
        args.output,
        AnnotatorConfig(),
        args.overlays,
    )
    print(json.dumps(counts, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
