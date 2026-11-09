"""Tests for offline USBL recording analysis."""

import pytest

from seatrac_ros2.csv_analysis import AnalysisConfig, analyze_recording
from seatrac_ros2.fix_validation import UsblFix, ValidationConfig


def fix(timestamp_s, x_m=1.0, y_m=0.0, **changes):
    """Create a valid normalized sample with concise overrides."""
    values = {
        'timestamp_s': timestamp_s,
        'x_m': x_m,
        'y_m': y_m,
        'z_m': -0.2,
        'beacon_id': 2,
        'range_m': 1.1,
        'fit_error': 0.5,
    }
    values.update(changes)
    return UsblFix(**values)


def test_clean_stationary_recording_is_go():
    """Accept a stable recording with sufficient rate and low dispersion."""
    samples = [
        fix(0.0, 1.00, 0.00),
        fix(0.5, 1.01, -0.01),
        fix(1.0, 0.99, 0.01),
        fix(1.5, 1.02, 0.00),
        fix(2.0, 1.00, -0.01),
    ]
    result = analyze_recording(
        samples,
        AnalysisConfig(
            validation=ValidationConfig(expected_beacon_id=2),
            stationary=True,
            minimum_duration_s=2.0,
        ),
    )
    assert result.go
    assert result.accepted_count == 5
    assert result.effective_rate_hz == pytest.approx(2.0)
    assert result.maximum_interval_s == pytest.approx(0.5)
    assert result.stationary_p95_error_m < 0.03


def test_gap_at_threshold_is_no_go():
    """Treat an interval equal to the stale timeout as an availability gap."""
    result = analyze_recording(
        [fix(0.0), fix(0.5), fix(3.0)],
        AnalysisConfig(
            maximum_rejected_fraction=1.0,
            minimum_duration_s=0.0,
        ),
    )
    assert not result.go
    assert result.gap_count == 1
    assert result.failed_criteria == ('gap_threshold_exceeded',)


def test_reports_validation_rejections_and_fraction():
    """Retain rejection reasons and apply the configured rejection budget."""
    samples = [
        fix(0.0),
        fix(0.5, fit_error=3.0),
        fix(1.0),
        fix(1.5),
    ]
    result = analyze_recording(
        samples,
        AnalysisConfig(
            validation=ValidationConfig(expected_beacon_id=2),
            maximum_rejected_fraction=0.20,
            minimum_duration_s=0.0,
        ),
    )
    assert not result.go
    assert result.rejected_reasons == {'fit_error_too_high': 1}
    assert result.rejected_fraction == pytest.approx(0.25)
    assert 'rejected_fraction_too_high' in result.failed_criteria


def test_stationary_dispersion_can_fail_independently():
    """Reject an allegedly stationary capture with excessive spread."""
    samples = [
        fix(0.0, 1.0, 0.0),
        fix(0.5, 1.0, 0.0),
        fix(1.0, 1.3, 0.0),
    ]
    result = analyze_recording(
        samples,
        AnalysisConfig(
            stationary=True,
            maximum_stationary_p95_error_m=0.10,
            minimum_duration_s=0.0,
        ),
    )
    assert not result.go
    assert result.stationary_p95_error_m == pytest.approx(0.30)
    assert 'stationary_p95_error_too_high' in result.failed_criteria


@pytest.mark.parametrize(
    'config, message',
    [
        (AnalysisConfig(gap_threshold_s=0.0), 'gap_threshold_s'),
        (AnalysisConfig(minimum_accepted_fixes=0), 'minimum_accepted_fixes'),
        (AnalysisConfig(minimum_duration_s=-1.0), 'minimum_duration_s'),
        (
            AnalysisConfig(maximum_rejected_fraction=1.1),
            'maximum_rejected_fraction',
        ),
    ],
)
def test_rejects_invalid_analysis_configuration(config, message):
    """Reject thresholds which could produce misleading verdicts."""
    with pytest.raises(ValueError, match=message):
        analyze_recording([fix(0.0)], config)
