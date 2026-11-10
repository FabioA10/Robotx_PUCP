#!/usr/bin/env python3
"""Validate and split a leak-free YOLO-OBB gate dataset."""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass, field
import json
import math
from pathlib import Path
import random
import sys
from typing import Iterable, Mapping, Sequence


CLASS_NAMES = ('gate', 'opening')
IMAGE_SUFFIXES = {'.jpg', '.jpeg', '.png', '.bmp', '.webp'}
SPLIT_NAMES = ('train', 'val', 'test')


@dataclass(frozen=True)
class DatasetIssue:
    """One actionable dataset problem."""

    severity: str
    code: str
    path: str
    message: str

    def as_dict(self) -> dict[str, str]:
        """Return a JSON-serializable representation."""
        return {
            'severity': self.severity,
            'code': self.code,
            'path': self.path,
            'message': self.message,
        }


@dataclass
class ValidationReport:
    """Validation counters, issues and split statistics."""

    image_count: int = 0
    label_file_count: int = 0
    instance_counts: dict[str, int] = field(
        default_factory=lambda: {name: 0 for name in CLASS_NAMES}
    )
    split_image_counts: dict[str, int] = field(default_factory=dict)
    split_source_counts: dict[str, int] = field(default_factory=dict)
    issues: list[DatasetIssue] = field(default_factory=list)

    @property
    def error_count(self) -> int:
        """Return the number of blocking errors."""
        return sum(issue.severity == 'error' for issue in self.issues)

    @property
    def warning_count(self) -> int:
        """Return the number of non-blocking warnings."""
        return sum(issue.severity == 'warning' for issue in self.issues)

    def as_dict(self) -> dict[str, object]:
        """Return a stable JSON report."""
        return {
            'valid': self.error_count == 0,
            'errors': self.error_count,
            'warnings': self.warning_count,
            'image_count': self.image_count,
            'label_file_count': self.label_file_count,
            'instance_counts': self.instance_counts,
            'split_image_counts': self.split_image_counts,
            'split_source_counts': self.split_source_counts,
            'issues': [issue.as_dict() for issue in self.issues],
        }


def load_manifest(manifest_path: Path) -> list[dict[str, str]]:
    """Load a CSV manifest and normalize the source group column."""
    with manifest_path.open(newline='', encoding='utf-8') as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None or not (
            {'image_path', 'output_file'} & set(reader.fieldnames)
        ):
            raise ValueError(
                'manifest must contain image_path or extractor output_file'
            )
        rows = []
        for row_number, raw in enumerate(reader, start=2):
            row = {key: (value or '').strip() for key, value in raw.items()}
            accepted = row.get('accepted', '').lower()
            if accepted in ('false', '0', 'no'):
                continue
            image_path = row.get('image_path') or row.get('output_file')
            if not image_path:
                if accepted:
                    raise ValueError(
                        f'manifest row {row_number} is accepted but has no output file'
                    )
                continue
            row['image_path'] = image_path
            source = (
                row.get('source_media')
                or row.get('source_video')
                or row.get('source_file')
            )
            if not source:
                raise ValueError(
                    f'manifest row {row_number} has no source_media/source_video'
                )
            row['source_media'] = source
            rows.append(row)
    if not rows:
        raise ValueError('manifest contains no image rows')
    return rows


def _normalized_ratios(ratios: Sequence[float]) -> tuple[float, float, float]:
    if len(ratios) != 3:
        raise ValueError('exactly three split ratios are required')
    if not all(math.isfinite(value) and value > 0.0 for value in ratios):
        raise ValueError('split ratios must be finite and positive')
    total = sum(ratios)
    return tuple(value / total for value in ratios)


def assign_group_splits(
    rows: Sequence[Mapping[str, str]],
    ratios: Sequence[float] = (0.70, 0.15, 0.15),
    seed: int = 2026,
) -> list[dict[str, str]]:
    """Assign whole source-media groups to train/val/test deterministically."""
    normalized = _normalized_ratios(ratios)
    groups: dict[str, list[int]] = {}
    for index, row in enumerate(rows):
        source = str(row.get('source_media', '')).strip()
        if not source:
            raise ValueError(f'row {index + 1} has no source_media')
        groups.setdefault(source, []).append(index)
    if len(groups) < 3:
        raise ValueError('at least three independent source groups are required')

    generator = random.Random(seed)
    tie_breakers = {source: generator.random() for source in groups}
    ordered = sorted(
        groups,
        key=lambda source: (-len(groups[source]), tie_breakers[source], source),
    )
    targets = {
        name: normalized[index] * len(rows)
        for index, name in enumerate(SPLIT_NAMES)
    }
    assigned_counts = {name: 0 for name in SPLIT_NAMES}
    source_split: dict[str, str] = {}

    # Seed every split with one source group before balancing the remainder.
    for name, source in zip(SPLIT_NAMES, ordered[:3]):
        source_split[source] = name
        assigned_counts[name] += len(groups[source])
    for source in ordered[3:]:
        split = max(
            SPLIT_NAMES,
            key=lambda name: (
                targets[name] - assigned_counts[name],
                -assigned_counts[name],
            ),
        )
        source_split[source] = split
        assigned_counts[split] += len(groups[source])

    output = []
    for row in rows:
        copied = dict(row)
        copied['split'] = source_split[copied['source_media']]
        output.append(copied)
    return output


def write_manifest(rows: Sequence[Mapping[str, str]], output_path: Path) -> None:
    """Write normalized manifest rows without dropping extractor metadata."""
    fieldnames = []
    for preferred in ('image_path', 'label_path', 'source_media', 'split'):
        if any(preferred in row for row in rows):
            fieldnames.append(preferred)
    for row in rows:
        for key in row:
            if key not in fieldnames and key != 'source_video':
                fieldnames.append(key)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(
            {key: row.get(key, '') for key in fieldnames} for row in rows
        )


def _safe_dataset_path(root: Path, relative_path: str) -> Path:
    candidate = (root / relative_path).resolve()
    try:
        candidate.relative_to(root.resolve())
    except ValueError as error:
        raise ValueError('path escapes the dataset root') from error
    return candidate


def _default_label_path(image_path: str) -> str:
    path = Path(image_path)
    parts = list(path.parts)
    if 'images' not in parts:
        raise ValueError('image_path must be under an images directory')
    parts[parts.index('images')] = 'labels'
    return str(Path(*parts).with_suffix('.txt'))


def _polygon_area(points: Sequence[tuple[float, float]]) -> float:
    return 0.5 * abs(sum(
        points[index][0] * points[(index + 1) % 4][1]
        - points[(index + 1) % 4][0] * points[index][1]
        for index in range(4)
    ))


def _orientation(a, b, c) -> float:
    return (b[0] - a[0]) * (c[1] - a[1]) - (
        b[1] - a[1]
    ) * (c[0] - a[0])


def _proper_intersection(a, b, c, d) -> bool:
    return (
        _orientation(a, b, c) * _orientation(a, b, d) < 0.0
        and _orientation(c, d, a) * _orientation(c, d, b) < 0.0
    )


def _validate_label_line(
    line: str,
    label_path: str,
    line_number: int,
) -> tuple[int | None, list[DatasetIssue]]:
    issues = []
    tokens = line.split()
    location = f'{label_path}:{line_number}'
    if len(tokens) != 9:
        return None, [DatasetIssue(
            'error',
            'label_token_count',
            location,
            'YOLO-OBB rows require class_id followed by four x/y corners',
        )]
    try:
        class_value = float(tokens[0])
        values = [float(token) for token in tokens[1:]]
    except ValueError:
        return None, [DatasetIssue(
            'error', 'label_not_numeric', location, 'label contains non-numeric data'
        )]
    if not class_value.is_integer() or int(class_value) not in range(len(CLASS_NAMES)):
        issues.append(DatasetIssue(
            'error', 'class_id', location, 'class_id must be 0 (gate) or 1 (opening)'
        ))
        class_id = None
    else:
        class_id = int(class_value)
    if not all(math.isfinite(value) for value in values):
        issues.append(DatasetIssue(
            'error', 'coordinate_not_finite', location, 'coordinates must be finite'
        ))
        return class_id, issues
    if not all(0.0 <= value <= 1.0 for value in values):
        issues.append(DatasetIssue(
            'error', 'coordinate_range', location, 'coordinates must be in [0, 1]'
        ))
        return class_id, issues
    points = list(zip(values[0::2], values[1::2]))
    if _polygon_area(points) <= 1.0e-6:
        issues.append(DatasetIssue(
            'error', 'degenerate_obb', location, 'quadrilateral area is zero or negligible'
        ))
    if (
        _proper_intersection(points[0], points[1], points[2], points[3])
        or _proper_intersection(points[1], points[2], points[3], points[0])
    ):
        issues.append(DatasetIssue(
            'error', 'self_intersecting_obb', location, 'corner order self-intersects'
        ))
    turns = [
        _orientation(
            points[index],
            points[(index + 1) % 4],
            points[(index + 2) % 4],
        )
        for index in range(4)
    ]
    if not (
        all(turn > 1.0e-9 for turn in turns)
        or all(turn < -1.0e-9 for turn in turns)
    ):
        issues.append(DatasetIssue(
            'error',
            'non_convex_obb',
            location,
            'corners must form one consistently ordered convex quadrilateral',
        ))
    return class_id, issues


def validate_dataset(
    dataset_root: Path,
    rows: Sequence[Mapping[str, str]],
) -> ValidationReport:
    """Validate manifest paths, labels, source splits and orphaned labels."""
    root = dataset_root.resolve()
    report = ValidationReport(image_count=len(rows))
    seen_images = set()
    referenced_labels = set()
    source_splits: dict[str, set[str]] = {}
    split_sources: dict[str, set[str]] = {name: set() for name in SPLIT_NAMES}

    for row_number, row in enumerate(rows, start=2):
        image_relative = str(row.get('image_path', '')).strip()
        source = str(row.get('source_media', '')).strip()
        split = str(row.get('split', '')).strip()
        if not image_relative:
            report.issues.append(DatasetIssue(
                'error', 'image_path_missing', f'manifest:{row_number}', 'missing image_path'
            ))
            continue
        if image_relative in seen_images:
            report.issues.append(DatasetIssue(
                'error', 'duplicate_image', image_relative, 'image appears more than once'
            ))
        seen_images.add(image_relative)
        try:
            image_path = _safe_dataset_path(root, image_relative)
        except ValueError as error:
            report.issues.append(DatasetIssue(
                'error', 'unsafe_image_path', image_relative, str(error)
            ))
            continue
        if image_path.suffix.lower() not in IMAGE_SUFFIXES:
            report.issues.append(DatasetIssue(
                'error', 'image_suffix', image_relative, 'unsupported image extension'
            ))
        if not image_path.is_file() or image_path.stat().st_size == 0:
            report.issues.append(DatasetIssue(
                'error', 'image_missing', image_relative, 'image is absent or empty'
            ))

        label_relative = str(row.get('label_path', '')).strip()
        if not label_relative:
            try:
                label_relative = _default_label_path(image_relative)
            except ValueError as error:
                report.issues.append(DatasetIssue(
                    'error', 'label_path_missing', image_relative, str(error)
                ))
                continue
        try:
            label_path = _safe_dataset_path(root, label_relative)
        except ValueError as error:
            report.issues.append(DatasetIssue(
                'error', 'unsafe_label_path', label_relative, str(error)
            ))
            continue
        referenced_labels.add(label_path)
        if not label_path.is_file():
            report.issues.append(DatasetIssue(
                'error',
                'label_missing',
                label_relative,
                'missing label; use an empty file for an intentional negative image',
            ))
        else:
            report.label_file_count += 1
            raw_lines = label_path.read_text(encoding='utf-8').splitlines()
            nonempty = [line.strip() for line in raw_lines if line.strip()]
            if len(nonempty) != len(set(nonempty)):
                report.issues.append(DatasetIssue(
                    'warning', 'duplicate_instance', label_relative, 'duplicate label rows found'
                ))
            for line_number, line in enumerate(raw_lines, start=1):
                if not line.strip():
                    continue
                class_id, issues = _validate_label_line(
                    line.strip(), label_relative, line_number
                )
                report.issues.extend(issues)
                if class_id is not None and not any(
                    issue.severity == 'error' for issue in issues
                ):
                    report.instance_counts[CLASS_NAMES[class_id]] += 1

        if split not in SPLIT_NAMES:
            report.issues.append(DatasetIssue(
                'error', 'split_missing', f'manifest:{row_number}', 'split must be train/val/test'
            ))
        else:
            report.split_image_counts[split] = (
                report.split_image_counts.get(split, 0) + 1
            )
            split_sources[split].add(source)
            source_splits.setdefault(source, set()).add(split)

    for source, splits in source_splits.items():
        if len(splits) > 1:
            report.issues.append(DatasetIssue(
                'error',
                'source_leakage',
                source,
                'source media appears in multiple splits: ' + ', '.join(sorted(splits)),
            ))
    report.split_source_counts = {
        name: len(split_sources[name]) for name in SPLIT_NAMES
    }
    for split in SPLIT_NAMES:
        if report.split_image_counts.get(split, 0) == 0:
            report.issues.append(DatasetIssue(
                'error', 'empty_split', split, f'{split} contains no images'
            ))

    labels_root = root / 'labels'
    if labels_root.is_dir():
        for label_path in labels_root.rglob('*.txt'):
            if label_path.resolve() not in referenced_labels:
                report.issues.append(DatasetIssue(
                    'error',
                    'orphan_label',
                    str(label_path.relative_to(root)),
                    'label has no manifest image',
                ))
    return report


def write_split_lists(
    dataset_root: Path,
    rows: Sequence[Mapping[str, str]],
) -> None:
    """Write deterministic image lists consumed by Ultralytics."""
    split_directory = dataset_root / 'splits'
    split_directory.mkdir(parents=True, exist_ok=True)
    for split in SPLIT_NAMES:
        images = sorted(
            str(row['image_path']) for row in rows if row.get('split') == split
        )
        (split_directory / f'{split}.txt').write_text(
            ''.join(f'../{image}\n' for image in images),
            encoding='utf-8',
        )


def write_data_yaml(dataset_root: Path, output_path: Path | None = None) -> Path:
    """Write a portable two-class Ultralytics dataset configuration."""
    root = dataset_root.resolve()
    output = output_path or root / 'data.yaml'
    content = '\n'.join([
        f'path: {json.dumps(str(root))}',
        'train: splits/train.txt',
        'val: splits/val.txt',
        'test: splits/test.txt',
        'names:',
        '  0: gate',
        '  1: opening',
        '',
    ])
    output.write_text(content, encoding='utf-8')
    return output


def build_parser() -> argparse.ArgumentParser:
    """Create the command-line parser."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('dataset_root', type=Path)
    parser.add_argument('--manifest', type=Path, default=Path('manifest.csv'))
    parser.add_argument('--assign-splits', action='store_true')
    parser.add_argument(
        '--output-manifest',
        type=Path,
        default=Path('manifest_split.csv'),
        help='normalized split manifest; the extractor manifest is never overwritten',
    )
    parser.add_argument('--train-ratio', type=float, default=0.70)
    parser.add_argument('--val-ratio', type=float, default=0.15)
    parser.add_argument('--test-ratio', type=float, default=0.15)
    parser.add_argument('--seed', type=int, default=2026)
    parser.add_argument('--write-config', action='store_true')
    parser.add_argument('--report', type=Path)
    return parser


def main(argv: Iterable[str] | None = None) -> int:
    """Validate a dataset and return zero only when it is training-safe."""
    arguments = build_parser().parse_args(argv)
    root = arguments.dataset_root.resolve()
    manifest_path = arguments.manifest
    if not manifest_path.is_absolute():
        manifest_path = root / manifest_path
    try:
        rows = load_manifest(manifest_path)
        if arguments.assign_splits:
            rows = assign_group_splits(
                rows,
                (
                    arguments.train_ratio,
                    arguments.val_ratio,
                    arguments.test_ratio,
                ),
                arguments.seed,
            )
            output_manifest = arguments.output_manifest
            if not output_manifest.is_absolute():
                output_manifest = root / output_manifest
            write_manifest(rows, output_manifest)
        report = validate_dataset(root, rows)
        if arguments.write_config and report.error_count == 0:
            write_split_lists(root, rows)
            write_data_yaml(root)
    except (OSError, ValueError) as error:
        print(json.dumps({'valid': False, 'fatal_error': str(error)}))
        return 2

    serialized = json.dumps(report.as_dict(), indent=2, sort_keys=True)
    print(serialized)
    if arguments.report:
        arguments.report.write_text(serialized + '\n', encoding='utf-8')
    return 0 if report.error_count == 0 else 2


if __name__ == '__main__':
    sys.exit(main())
