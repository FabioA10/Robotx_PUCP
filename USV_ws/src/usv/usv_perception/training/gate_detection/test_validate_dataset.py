"""Tests for leak-free gate dataset validation."""

import csv
from pathlib import Path

try:
    from .validate_dataset import (
        assign_group_splits,
        load_manifest,
        validate_dataset,
        write_data_yaml,
        write_manifest,
        write_split_lists,
    )
except ImportError:
    from validate_dataset import (
        assign_group_splits,
        load_manifest,
        validate_dataset,
        write_data_yaml,
        write_manifest,
        write_split_lists,
    )


VALID_GATE = '0 0.1 0.1 0.4 0.1 0.4 0.5 0.1 0.5\n'
VALID_OPENING = '1 0.2 0.2 0.3 0.2 0.3 0.4 0.2 0.4\n'


def _dataset(tmp_path: Path, source_count: int = 5):
    rows = []
    for index in range(source_count):
        for frame in range(index + 1):
            name = f'source_{index}_{frame}.jpg'
            image = tmp_path / 'images' / name
            label = tmp_path / 'labels' / name.replace('.jpg', '.txt')
            image.parent.mkdir(parents=True, exist_ok=True)
            label.parent.mkdir(parents=True, exist_ok=True)
            image.write_bytes(b'not-decoded-by-validator')
            label.write_text(VALID_GATE + VALID_OPENING, encoding='utf-8')
            rows.append({
                'image_path': f'images/{name}',
                'source_media': f'source_{index}.MOV',
            })
    return rows


def test_group_split_is_deterministic_and_has_no_source_leakage(tmp_path):
    rows = _dataset(tmp_path)
    first = assign_group_splits(rows, seed=9)
    second = assign_group_splits(rows, seed=9)
    assert first == second
    source_splits = {}
    for row in first:
        source_splits.setdefault(row['source_media'], set()).add(row['split'])
    assert all(len(splits) == 1 for splits in source_splits.values())
    assert {row['split'] for row in first} == {'train', 'val', 'test'}


def test_valid_dataset_writes_ultralytics_files(tmp_path):
    rows = assign_group_splits(_dataset(tmp_path))
    report = validate_dataset(tmp_path, rows)
    assert report.error_count == 0
    assert report.instance_counts == {'gate': 15, 'opening': 15}

    write_split_lists(tmp_path, rows)
    config = write_data_yaml(tmp_path)
    assert config.is_file()
    assert '0: gate' in config.read_text(encoding='utf-8')
    assert all((tmp_path / 'splits' / f'{name}.txt').is_file()
               for name in ('train', 'val', 'test'))


def test_validator_reports_missing_invalid_and_orphan_labels(tmp_path):
    rows = assign_group_splits(_dataset(tmp_path, source_count=3))
    first_label = tmp_path / rows[0]['image_path'].replace('images/', 'labels/')
    first_label = first_label.with_suffix('.txt')
    first_label.write_text(
        '0 1.2 0.1 0.4 0.1 0.4 0.5 0.1 0.5\n', encoding='utf-8'
    )
    second_label = tmp_path / rows[1]['image_path'].replace('images/', 'labels/')
    second_label.with_suffix('.txt').unlink()
    (tmp_path / 'labels' / 'orphan.txt').write_text(VALID_GATE, encoding='utf-8')

    report = validate_dataset(tmp_path, rows)
    codes = {issue.code for issue in report.issues}
    assert {'coordinate_range', 'label_missing', 'orphan_label'} <= codes
    assert report.error_count >= 3


def test_manifest_loader_accepts_source_video_alias(tmp_path):
    path = tmp_path / 'manifest.csv'
    with path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=['image_path', 'source_video'])
        writer.writeheader()
        writer.writerow({'image_path': 'images/a.jpg', 'source_video': 'a.MOV'})
    rows = load_manifest(path)
    assert rows[0]['source_media'] == 'a.MOV'


def test_manifest_loader_consumes_extractor_schema(tmp_path):
    path = tmp_path / 'manifest.csv'
    with path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=['output_file', 'source_file', 'accepted'],
        )
        writer.writeheader()
        writer.writerow({
            'output_file': 'images/a.jpg',
            'source_file': 'a.MOV',
            'accepted': 'true',
        })
        writer.writerow({
            'output_file': '',
            'source_file': 'a.MOV',
            'accepted': 'false',
        })
    rows = load_manifest(path)
    assert len(rows) == 1
    assert rows[0]['image_path'] == 'images/a.jpg'
    assert rows[0]['source_media'] == 'a.MOV'


def test_manifest_round_trip_preserves_metadata(tmp_path):
    rows = [{
        'image_path': 'images/a.jpg',
        'source_media': 'a.MOV',
        'split': 'train',
        'timestamp_s': '2.5',
    }]
    path = tmp_path / 'manifest.csv'
    write_manifest(rows, path)
    loaded = load_manifest(path)
    assert loaded[0]['timestamp_s'] == '2.5'
