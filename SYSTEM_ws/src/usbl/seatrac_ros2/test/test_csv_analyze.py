"""Tests for the offline USBL analyzer command-line interface."""

import json

from seatrac_ros2.csv_analyze import main


def write_csv(tmp_path, rows):
    """Write a minimal normalized recording for command tests."""
    path = tmp_path / 'fixes.csv'
    path.write_text(
        'timestamp_s,body_forward_m,body_left_m,beacon_id\n' + rows,
        encoding='utf-8',
    )
    return str(path)


def test_json_go_report_and_exit_status(tmp_path, capsys):
    """Return zero and valid JSON when all configured gates pass."""
    path = write_csv(
        tmp_path,
        '0.0,1.00,0.00,2\n'
        '0.5,1.01,0.00,2\n'
        '1.0,1.02,0.00,2\n',
    )
    assert main([
        path,
        '--expected-beacon-id',
        '2',
        '--minimum-duration-s',
        '1.0',
        '--json',
    ]) == 0
    report = json.loads(capsys.readouterr().out)
    assert report['go']
    assert report['accepted_count'] == 3


def test_stationary_no_go_exit_status(tmp_path, capsys):
    """Return two when a valid capture fails an acceptance criterion."""
    path = write_csv(
        tmp_path,
        '0.0,1.0,0.0,2\n'
        '0.5,1.0,0.0,2\n'
        '1.0,1.3,0.0,2\n',
    )
    assert main([path, '--stationary']) == 2
    assert 'verdict: NO-GO' in capsys.readouterr().out


def test_bad_input_exit_status(tmp_path, capsys):
    """Return one and explain an input error without a traceback."""
    missing = str(tmp_path / 'missing.csv')
    assert main([missing]) == 1
    assert 'error:' in capsys.readouterr().err
