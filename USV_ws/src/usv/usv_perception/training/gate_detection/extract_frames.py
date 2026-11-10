#!/usr/bin/env python3
"""Extract a deterministic, auditable set of gate-detection frames."""

from __future__ import annotations

import argparse
import csv
from dataclasses import asdict, dataclass
import hashlib
import json
import math
from pathlib import Path
import shutil
import subprocess
import tempfile
from typing import Iterable, Optional, Sequence

import cv2
import numpy as np


IMAGE_EXTENSIONS = {'.jpg', '.jpeg', '.png'}
VIDEO_EXTENSIONS = {'.mov', '.mp4', '.m4v'}
MANIFEST_FIELDS = (
    'source_file',
    'source_kind',
    'source_rotation_deg',
    'timestamp_s',
    'output_file',
    'width',
    'height',
    'blur_variance',
    'dhash_hex',
    'accepted',
    'rejection_reason',
)


@dataclass(frozen=True)
class SourceMedia:
    """Metadata needed to schedule samples from one source file."""

    path: Path
    relative_path: str
    kind: str
    duration_s: Optional[float]
    rotation_deg: int


@dataclass(frozen=True)
class FramePlan:
    """One source frame requested at an explicit media timestamp."""

    source: SourceMedia
    timestamp_s: float
    output_name: str


@dataclass(frozen=True)
class ManifestRow:
    """Selection evidence for one extracted candidate frame."""

    source_file: str
    source_kind: str
    source_rotation_deg: int
    timestamp_s: float
    output_file: str
    width: int
    height: int
    blur_variance: float
    dhash_hex: str
    accepted: bool
    rejection_reason: str


def run_command(command: Sequence[str]) -> subprocess.CompletedProcess:
    """Run one external media command and surface its diagnostic output."""
    try:
        return subprocess.run(
            command,
            check=True,
            capture_output=True,
            text=True,
        )
    except FileNotFoundError as error:
        raise RuntimeError(f'Required command not found: {command[0]}') from error
    except subprocess.CalledProcessError as error:
        details = error.stderr.strip() or error.stdout.strip()
        raise RuntimeError(
            f'Command failed ({command[0]}): {details}'
        ) from error


def discover_sources(source_dir: Path) -> list[Path]:
    """Return supported media paths in stable relative-path order."""
    supported = IMAGE_EXTENSIONS | VIDEO_EXTENSIONS
    paths = [
        path
        for path in source_dir.rglob('*')
        if path.is_file() and path.suffix.lower() in supported
    ]
    return sorted(paths, key=lambda path: path.relative_to(source_dir).as_posix())


def _rotation_from_stream(stream: dict) -> int:
    """Read display rotation from either legacy tags or side data."""
    rotations = []
    tags = stream.get('tags', {})
    if 'rotate' in tags:
        rotations.append(tags['rotate'])
    for entry in stream.get('side_data_list', []):
        if 'rotation' in entry:
            rotations.append(entry['rotation'])
    for value in rotations:
        try:
            return int(round(float(value))) % 360
        except (TypeError, ValueError):
            continue
    return 0


def probe_source(path: Path, source_dir: Path) -> SourceMedia:
    """Probe duration and rotation without decoding semantic image content."""
    suffix = path.suffix.lower()
    kind = 'image' if suffix in IMAGE_EXTENSIONS else 'video'
    command = [
        'ffprobe',
        '-v',
        'error',
        '-select_streams',
        'v:0',
        '-show_entries',
        'stream=width,height:stream_tags=rotate:stream_side_data=rotation:'
        'format=duration',
        '-of',
        'json',
        str(path),
    ]
    result = run_command(command)
    payload = json.loads(result.stdout)
    streams = payload.get('streams', [])
    if not streams:
        raise ValueError(f'No video stream found in {path}')

    duration_s = None
    if kind == 'video':
        raw_duration = payload.get('format', {}).get('duration')
        try:
            duration_s = float(raw_duration)
        except (TypeError, ValueError) as error:
            raise ValueError(f'Invalid video duration for {path}') from error
        if not math.isfinite(duration_s) or duration_s <= 0.0:
            raise ValueError(f'Invalid video duration for {path}: {duration_s}')

    return SourceMedia(
        path=path,
        relative_path=path.relative_to(source_dir).as_posix(),
        kind=kind,
        duration_s=duration_s,
        rotation_deg=_rotation_from_stream(streams[0]),
    )


def sample_timestamps(duration_s: float, interval_s: float) -> tuple[float, ...]:
    """Generate stable timestamps strictly before the end of a video."""
    if not math.isfinite(duration_s) or duration_s <= 0.0:
        raise ValueError('duration_s must be finite and positive')
    if not math.isfinite(interval_s) or interval_s <= 0.0:
        raise ValueError('interval_s must be finite and positive')
    count = int(math.ceil(duration_s / interval_s))
    return tuple(
        round(index * interval_s, 6)
        for index in range(count)
        if index * interval_s < duration_s
    )


def _output_name(source: SourceMedia, timestamp_s: float) -> str:
    """Build a collision-resistant name retaining source and timestamp."""
    safe_stem = ''.join(
        character if character.isalnum() or character in ('-', '_') else '_'
        for character in source.path.stem
    )
    path_id = hashlib.sha1(
        source.relative_path.encode('utf-8')
    ).hexdigest()[:8]
    timestamp_ms = int(round(timestamp_s * 1000.0))
    return f'{safe_stem}__{path_id}__t{timestamp_ms:09d}ms.jpg'


def build_plan(
    sources: Iterable[SourceMedia], interval_s: float
) -> list[FramePlan]:
    """Build a deterministic extraction plan for images and videos."""
    plan = []
    for source in sources:
        timestamps = (
            (0.0,)
            if source.kind == 'image'
            else sample_timestamps(float(source.duration_s), interval_s)
        )
        plan.extend(
            FramePlan(source, timestamp, _output_name(source, timestamp))
            for timestamp in timestamps
        )
    return plan


def extract_candidate(plan: FramePlan, output_path: Path) -> None:
    """Decode one frame with ffmpeg display rotation enabled by default."""
    command = ['ffmpeg', '-v', 'error', '-y']
    if plan.source.kind == 'video':
        command.extend(['-ss', f'{plan.timestamp_s:.6f}'])
    command.extend([
        '-i',
        str(plan.source.path),
        '-map_metadata',
        '-1',
        '-frames:v',
        '1',
        '-q:v',
        '2',
        str(output_path),
    ])
    run_command(command)
    if not output_path.is_file() or output_path.stat().st_size == 0:
        raise RuntimeError(f'ffmpeg did not produce {output_path}')


def load_image(path: Path) -> np.ndarray:
    """Load an extracted image and reject undecodable or empty output."""
    image = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if image is None or image.size == 0:
        raise ValueError(f'Unable to decode extracted frame: {path}')
    return image


def blur_variance(image: np.ndarray) -> float:
    """Return Laplacian variance as a reproducible sharpness indicator."""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


def difference_hash(image: np.ndarray) -> int:
    """Return a 64-bit dHash used only for conservative deduplication."""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    reduced = cv2.resize(gray, (9, 8), interpolation=cv2.INTER_AREA)
    comparisons = reduced[:, 1:] > reduced[:, :-1]
    value = 0
    for bit in comparisons.flatten():
        value = (value << 1) | int(bit)
    return value


def hamming_distance(first: int, second: int) -> int:
    """Return the bit difference between two fixed-width hashes."""
    return (first ^ second).bit_count()


def select_rejection(
    sharpness: float,
    image_hash: int,
    accepted_hashes: Sequence[int],
    min_blur_variance: float,
    max_dhash_distance: int,
) -> str:
    """Return the deterministic rejection reason or an empty string."""
    if sharpness < min_blur_variance:
        return 'blur_below_threshold'
    if any(
        hamming_distance(image_hash, previous) <= max_dhash_distance
        for previous in accepted_hashes
    ):
        return 'perceptual_duplicate'
    return ''


def _validate_paths(source_dir: Path, output_dir: Path) -> None:
    """Reject unsafe or ambiguous source/output layouts."""
    if not source_dir.is_dir():
        raise ValueError(f'Source directory does not exist: {source_dir}')
    source_resolved = source_dir.resolve()
    output_resolved = output_dir.resolve()
    if output_resolved == source_resolved or source_resolved in output_resolved.parents:
        raise ValueError('Output directory must be outside the source directory')
    if output_dir.exists() and any(output_dir.iterdir()):
        raise ValueError(f'Output directory is not empty: {output_dir}')


def _write_manifests(
    output_dir: Path,
    rows: Sequence[ManifestRow],
    interval_s: float,
    min_blur_variance: float,
    max_dhash_distance: int,
) -> None:
    """Write equivalent CSV and versioned JSON audit manifests."""
    csv_path = output_dir / 'manifest.csv'
    with csv_path.open('w', encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=MANIFEST_FIELDS)
        writer.writeheader()
        writer.writerows(asdict(row) for row in rows)

    accepted_count = sum(row.accepted for row in rows)
    payload = {
        'schema': 'robotx.gate_frames.v1',
        'config': {
            'interval_s': interval_s,
            'min_blur_variance': min_blur_variance,
            'max_dhash_distance': max_dhash_distance,
            'orientation': 'ffmpeg_display_rotation',
        },
        'summary': {
            'candidates': len(rows),
            'accepted': accepted_count,
            'rejected': len(rows) - accepted_count,
        },
        'frames': [asdict(row) for row in rows],
    }
    with (output_dir / 'manifest.json').open('w', encoding='utf-8') as stream:
        json.dump(payload, stream, indent=2, sort_keys=True)
        stream.write('\n')


def extract_dataset(
    source_dir: Path,
    output_dir: Path,
    interval_s: float = 2.5,
    min_blur_variance: float = 0.0,
    max_dhash_distance: int = 2,
    dry_run: bool = False,
) -> dict:
    """Probe, plan and optionally extract the selected dataset frames."""
    if min_blur_variance < 0.0 or not math.isfinite(min_blur_variance):
        raise ValueError('min_blur_variance must be finite and non-negative')
    if not 0 <= max_dhash_distance <= 64:
        raise ValueError('max_dhash_distance must be in [0, 64]')
    _validate_paths(source_dir, output_dir)

    paths = discover_sources(source_dir)
    if not paths:
        raise ValueError(f'No supported media found in {source_dir}')
    sources = [probe_source(path, source_dir) for path in paths]
    plan = build_plan(sources, interval_s)
    summary = {
        'source_files': len(sources),
        'source_images': sum(source.kind == 'image' for source in sources),
        'source_videos': sum(source.kind == 'video' for source in sources),
        'planned_candidates': len(plan),
        'dry_run': dry_run,
    }
    if dry_run:
        return summary

    output_dir.mkdir(parents=True, exist_ok=True)
    images_dir = output_dir / 'images'
    images_dir.mkdir()
    rows = []
    # Keep independent source groups available for leakage-free train/val/test
    # splits. Near-identical frames are rejected only within the same media
    # file; similarity across two source files remains measurable evidence.
    accepted_hashes_by_source: dict[str, list[int]] = {}
    with tempfile.TemporaryDirectory(
        prefix='gate_frame_candidates.', dir=output_dir
    ) as temporary:
        temporary_dir = Path(temporary)
        for item in plan:
            candidate_path = temporary_dir / item.output_name
            extract_candidate(item, candidate_path)
            image = load_image(candidate_path)
            sharpness = blur_variance(image)
            image_hash = difference_hash(image)
            accepted_hashes = accepted_hashes_by_source.setdefault(
                item.source.relative_path, []
            )
            rejection = select_rejection(
                sharpness,
                image_hash,
                accepted_hashes,
                min_blur_variance,
                max_dhash_distance,
            )
            accepted = not rejection
            output_file = ''
            if accepted:
                destination = images_dir / item.output_name
                shutil.move(str(candidate_path), destination)
                accepted_hashes.append(image_hash)
                output_file = destination.relative_to(output_dir).as_posix()
            height, width = image.shape[:2]
            rows.append(ManifestRow(
                source_file=item.source.relative_path,
                source_kind=item.source.kind,
                source_rotation_deg=item.source.rotation_deg,
                timestamp_s=item.timestamp_s,
                output_file=output_file,
                width=width,
                height=height,
                blur_variance=round(sharpness, 6),
                dhash_hex=f'{image_hash:016x}',
                accepted=accepted,
                rejection_reason=rejection,
            ))

    _write_manifests(
        output_dir,
        rows,
        interval_s,
        min_blur_variance,
        max_dhash_distance,
    )
    summary.update({
        'accepted': sum(row.accepted for row in rows),
        'rejected': sum(not row.accepted for row in rows),
        'manifest_csv': str(output_dir / 'manifest.csv'),
        'manifest_json': str(output_dir / 'manifest.json'),
    })
    return summary


def build_argument_parser() -> argparse.ArgumentParser:
    """Build the command-line interface without side effects."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source_dir', type=Path)
    parser.add_argument('output_dir', type=Path)
    parser.add_argument('--interval-s', type=float, default=2.5)
    parser.add_argument('--min-blur-variance', type=float, default=0.0)
    parser.add_argument('--max-dhash-distance', type=int, default=2)
    parser.add_argument(
        '--dry-run',
        action='store_true',
        help='Probe and report the plan without creating output files.',
    )
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    """Run extraction and print a machine-readable execution summary."""
    arguments = build_argument_parser().parse_args(argv)
    try:
        summary = extract_dataset(
            arguments.source_dir,
            arguments.output_dir,
            interval_s=arguments.interval_s,
            min_blur_variance=arguments.min_blur_variance,
            max_dhash_distance=arguments.max_dhash_distance,
            dry_run=arguments.dry_run,
        )
    except (RuntimeError, ValueError, OSError, json.JSONDecodeError) as error:
        print(json.dumps({'error': str(error)}, sort_keys=True))
        return 2
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
