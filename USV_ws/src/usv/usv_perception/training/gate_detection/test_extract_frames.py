"""Tests for deterministic, source-preserving gate frame extraction."""

import importlib.util
import json
from pathlib import Path
import subprocess
import sys

import cv2
import numpy as np
import pytest

_MODULE_PATH = Path(__file__).with_name('extract_frames.py')
_SPEC = importlib.util.spec_from_file_location('gate_extract_frames', _MODULE_PATH)
_MODULE = importlib.util.module_from_spec(_SPEC)
sys.modules[_SPEC.name] = _MODULE
_SPEC.loader.exec_module(_MODULE)

blur_variance = _MODULE.blur_variance
build_plan = _MODULE.build_plan
difference_hash = _MODULE.difference_hash
extract_dataset = _MODULE.extract_dataset
hamming_distance = _MODULE.hamming_distance
sample_timestamps = _MODULE.sample_timestamps
SourceMedia = _MODULE.SourceMedia


def test_sample_timestamps_never_reaches_video_end():
    assert sample_timestamps(5.1, 2.5) == (0.0, 2.5, 5.0)
    assert sample_timestamps(5.0, 2.5) == (0.0, 2.5)


def test_plan_has_collision_resistant_source_and_time_names(tmp_path):
    first = SourceMedia(
        tmp_path / 'a' / 'clip.mov', 'a/clip.mov', 'video', 3.0, 0
    )
    second = SourceMedia(
        tmp_path / 'b' / 'clip.mov', 'b/clip.mov', 'video', 3.0, 0
    )

    plan = build_plan((first, second), interval_s=2.5)

    assert len(plan) == 4
    assert len({entry.output_name for entry in plan}) == 4
    assert all('t000000000ms' in entry.output_name for entry in plan[::2])


def test_hash_and_blur_metrics_are_deterministic():
    sharp = np.zeros((64, 64, 3), dtype=np.uint8)
    sharp[:, 32:] = 255
    flat = np.full((64, 64, 3), 127, dtype=np.uint8)

    assert difference_hash(sharp) == difference_hash(sharp.copy())
    assert hamming_distance(difference_hash(sharp), difference_hash(sharp)) == 0
    assert blur_variance(sharp) > blur_variance(flat)


def _write_test_video(path: Path) -> None:
    subprocess.run(
        [
            'ffmpeg', '-v', 'error', '-y',
            '-f', 'lavfi', '-i', 'testsrc=size=96x64:rate=2:duration=2',
            '-pix_fmt', 'yuv420p', str(path),
        ],
        check=True,
    )


def test_temporary_end_to_end_extraction_writes_auditable_manifest(tmp_path):
    source = tmp_path / 'source'
    source.mkdir()
    _write_test_video(source / 'clip.mp4')
    image = np.zeros((64, 96, 3), dtype=np.uint8)
    image[:, 48:] = 255
    assert cv2.imwrite(str(source / 'still.jpg'), image)
    output = tmp_path / 'processed'

    summary = extract_dataset(
        source,
        output,
        interval_s=1.0,
        max_dhash_distance=0,
    )

    manifest = json.loads((output / 'manifest.json').read_text())
    assert summary['source_videos'] == 1
    assert summary['source_images'] == 1
    assert manifest['schema'] == 'robotx.gate_frames.v1'
    assert len(manifest['frames']) == summary['planned_candidates']
    assert manifest['summary']['accepted'] == summary['accepted']
    accepted = [row for row in manifest['frames'] if row['accepted']]
    assert accepted
    assert all(row['source_file'] for row in accepted)
    assert all((output / row['output_file']).is_file() for row in accepted)


def test_dry_run_creates_no_output(tmp_path):
    source = tmp_path / 'source'
    source.mkdir()
    image = np.zeros((32, 32, 3), dtype=np.uint8)
    assert cv2.imwrite(str(source / 'still.jpg'), image)
    output = tmp_path / 'processed'

    summary = extract_dataset(source, output, dry_run=True)

    assert summary['dry_run'] is True
    assert not output.exists()


@pytest.mark.parametrize('duration,interval', [(0.0, 1.0), (1.0, 0.0)])
def test_invalid_sampling_configuration_is_rejected(duration, interval):
    with pytest.raises(ValueError):
        sample_timestamps(duration, interval)
