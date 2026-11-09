"""Tests for the ROS-independent USBL fix validator."""

import math

import pytest

from seatrac_ros2.fix_validation import (
    FixValidator,
    UsblFix,
    ValidationConfig,
    pose_covariance,
)


def fix(timestamp_s=1.0, **changes):
    values = {
        "timestamp_s": timestamp_s,
        "x_m": 1.0,
        "y_m": 0.0,
        "z_m": -0.2,
        "beacon_id": 2,
        "range_m": 1.1,
        "fit_error": 0.5,
    }
    values.update(changes)
    return UsblFix(**values)


@pytest.mark.parametrize(
    "changes, reason",
    [
        ({"x_m": math.nan}, "non_finite_position_or_timestamp"),
        ({"range_m": math.inf}, "non_finite_quality_value"),
        ({"position_valid": False}, "position_not_valid"),
        ({"position_filter_error": True}, "position_filter_error"),
        ({"beacon_id": 3}, "unexpected_beacon_id"),
        ({"range_m": 1001.0}, "range_out_of_bounds"),
        ({"fit_error": 3.0}, "fit_error_too_high"),
    ],
)
def test_rejects_bad_samples(changes, reason):
    validator = FixValidator(ValidationConfig(expected_beacon_id=2))
    result = validator.validate(fix(**changes))
    assert not result.accepted
    assert result.reason == reason


def test_warn_fit_boundary_is_accepted_but_degraded():
    validator = FixValidator(ValidationConfig(expected_beacon_id=2))
    result = validator.validate(fix(fit_error=2.0))
    assert result.accepted
    assert result.degraded


def test_rejection_does_not_replace_last_accepted_sample():
    validator = FixValidator(
        ValidationConfig(
            expected_beacon_id=2,
            max_horizontal_speed_mps=2.0,
        )
    )
    assert validator.validate(fix(timestamp_s=1.0)).accepted
    assert not validator.validate(
        fix(timestamp_s=2.0, x_m=100.0)
    ).accepted
    result = validator.validate(fix(timestamp_s=2.0, x_m=2.0))
    assert result.accepted
    assert result.horizontal_speed_mps == pytest.approx(1.0)


def test_duplicate_timestamp_is_rejected():
    validator = FixValidator(ValidationConfig(expected_beacon_id=2))
    assert validator.validate(fix(timestamp_s=1.0)).accepted
    result = validator.validate(fix(timestamp_s=1.0))
    assert result.reason == "non_monotonic_timestamp"


def test_vertical_speed_gate():
    validator = FixValidator(
        ValidationConfig(max_vertical_speed_mps=0.5)
    )
    assert validator.validate(fix(timestamp_s=1.0)).accepted
    result = validator.validate(fix(timestamp_s=2.0, z_m=1.0))
    assert result.reason == "vertical_speed_too_high"


def test_covariance_grows_with_range_and_fit_error():
    close = pose_covariance(fix(range_m=1.0, fit_error=0.5))
    far = pose_covariance(fix(range_m=10.0, fit_error=2.5))
    assert far[0] > close[0]
    assert far[7] == far[0]
    assert far[21] == 1.0e6
    assert far[28] == 1.0e6
    assert far[35] == 1.0e6
