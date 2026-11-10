"""Tests for the review-only classical gate pre-annotator."""

from pathlib import Path

import cv2
import numpy as np

try:
    from .auto_annotate import (
        AnnotatorConfig,
        annotate_image,
        process_paths,
        yolo_obb_lines,
    )
except ImportError:
    from auto_annotate import (
        AnnotatorConfig,
        annotate_image,
        process_paths,
        yolo_obb_lines,
    )


def _synthetic_gate(angle_degrees=8.0):
    image = np.full((480, 720, 3), (80, 60, 40), dtype=np.uint8)
    gate_rect = ((360.0, 245.0), (250.0, 310.0), angle_degrees)
    cv2.fillConvexPoly(
        image,
        np.rint(cv2.boxPoints(gate_rect)).astype(np.int32),
        (235, 235, 235),
    )
    for center, size in (((320.0, 190.0), (58.0, 82.0)),
                         ((400.0, 285.0), (55.0, 105.0))):
        opening = (center, size, angle_degrees)
        cv2.fillConvexPoly(
            image,
            np.rint(cv2.boxPoints(opening)).astype(np.int32),
            (25, 25, 25),
        )
    return image


def test_rotated_gate_and_two_openings_generate_obb_candidates():
    annotations = annotate_image(_synthetic_gate())

    classes = [item.class_name for item in annotations.detections]
    assert classes.count('gate') == 1
    assert classes.count('opening') == 2
    assert all(item.diagnostics['review_status'] == 'candidate_only'
               for item in annotations.detections)


def test_yolo_obb_has_class_and_four_normalized_points():
    annotations = annotate_image(_synthetic_gate(angle_degrees=-12.0))
    lines = yolo_obb_lines(annotations)

    assert len(lines) == 3
    for line in lines:
        fields = line.split()
        assert len(fields) == 9
        assert fields[0] in ('0', '1')
        assert all(0.0 <= float(value) <= 1.0 for value in fields[1:])


def test_white_object_without_dark_openings_is_rejected():
    image = np.zeros((300, 400, 3), dtype=np.uint8)
    cv2.rectangle(image, (120, 70), (280, 230), (240, 240, 240), -1)

    annotations = annotate_image(image)

    assert annotations.detections == ()


def test_process_tree_marks_outputs_as_candidates(tmp_path: Path):
    input_path = tmp_path / 'input'
    output_path = tmp_path / 'output'
    input_path.mkdir()
    cv2.imwrite(str(input_path / 'frame.jpg'), _synthetic_gate())

    counts = process_paths(
        input_path,
        output_path,
        AnnotatorConfig(),
        write_overlays=True,
    )

    assert counts == {'images': 1, 'gates': 1, 'openings': 2, 'empty': 0}
    assert (output_path / 'labels_candidates' / 'frame.txt').is_file()
    diagnostic = (output_path / 'diagnostics' / 'frame.json').read_text()
    assert '"review_required": true' in diagnostic
    assert (output_path / 'overlays' / 'frame.jpg').is_file()
