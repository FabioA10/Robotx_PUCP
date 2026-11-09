"""Tests for normalized USBL CSV recordings."""

import pytest

from seatrac_ros2.csv_fixes import CsvFormatError, load_csv_fixes


def write_csv(tmp_path, contents):
    path = tmp_path / "fixes.csv"
    path.write_text(contents, encoding="utf-8")
    return str(path)


def test_loads_comments_aliases_and_optional_fields(tmp_path):
    path = write_csv(
        tmp_path,
        "# seatrac_csv_version=1\n"
        "stamp_s,body_forward_m,body_left_m,src_id,valid,fit_error\n"
        "0.0,1.2,-0.3,2,true,0.4\n",
    )
    samples = load_csv_fixes(path)
    assert len(samples) == 1
    assert samples[0].timestamp_s == 0.0
    assert samples[0].z_m == 0.0
    assert samples[0].beacon_id == 2
    assert samples[0].position_valid


def test_rejects_missing_required_column(tmp_path):
    path = write_csv(
        tmp_path, "timestamp_s,body_forward_m\n0.0,1.0\n"
    )
    with pytest.raises(CsvFormatError, match="body_left_m"):
        load_csv_fixes(path)


def test_reports_bad_row_line(tmp_path):
    path = write_csv(
        tmp_path,
        "timestamp_s,body_forward_m,body_left_m,position_valid\n"
        "0.0,1.0,2.0,perhaps\n",
    )
    with pytest.raises(CsvFormatError, match="line 2: invalid boolean"):
        load_csv_fixes(path)


def test_rejects_header_only_file(tmp_path):
    path = write_csv(
        tmp_path, "timestamp_s,body_forward_m,body_left_m\n"
    )
    with pytest.raises(CsvFormatError, match="no data rows"):
        load_csv_fixes(path)


def test_rejects_ambiguous_native_or_xyz_columns(tmp_path):
    path = write_csv(
        tmp_path,
        "timestamp_s,x_m,y_m,easting_m,northing_m\n"
        "0.0,1.0,2.0,3.0,4.0\n",
    )
    with pytest.raises(CsvFormatError, match="body_forward_m"):
        load_csv_fixes(path)
