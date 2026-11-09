"""Offline quality analysis for normalized USBL recordings."""

import collections
import dataclasses
import math
import statistics
import typing

from seatrac_ros2.fix_validation import (
    FixValidator,
    UsblFix,
    ValidationConfig,
)


@dataclasses.dataclass(frozen=True)
class AnalysisConfig:
    """Acceptance criteria for one normalized recording."""

    validation: ValidationConfig = ValidationConfig()
    gap_threshold_s: float = 2.5
    minimum_accepted_fixes: int = 3
    minimum_duration_s: float = 60.0
    maximum_rejected_fraction: float = 0.05
    stationary: bool = False
    maximum_stationary_p95_error_m: float = 0.10


@dataclasses.dataclass(frozen=True)
class RecordingAnalysis:
    """Metrics and conservative verdict produced for a recording."""

    total_count: int
    accepted_count: int
    rejected_count: int
    degraded_count: int
    rejected_reasons: typing.Dict[str, int]
    duration_s: typing.Optional[float]
    effective_rate_hz: typing.Optional[float]
    median_interval_s: typing.Optional[float]
    p95_interval_s: typing.Optional[float]
    maximum_interval_s: typing.Optional[float]
    gap_count: int
    center_x_m: typing.Optional[float]
    center_y_m: typing.Optional[float]
    stationary_p95_error_m: typing.Optional[float]
    go: bool
    failed_criteria: typing.Tuple[str, ...]

    @property
    def rejected_fraction(self) -> float:
        """Return the rejected fraction, including an empty-input guard."""
        if self.total_count == 0:
            return 0.0
        return self.rejected_count / self.total_count


def analyze_recording(
    samples: typing.Sequence[UsblFix],
    config: AnalysisConfig = AnalysisConfig(),
) -> RecordingAnalysis:
    """Calculate availability and noise metrics for samples in order."""
    _validate_analysis_config(config)
    validator = FixValidator(config.validation)
    accepted: typing.List[UsblFix] = []
    rejected_reasons: collections.Counter[str] = collections.Counter()
    degraded_count = 0

    for sample in samples:
        result = validator.validate(sample)
        if result.accepted:
            accepted.append(sample)
            degraded_count += int(result.degraded)
        else:
            rejected_reasons[result.reason] += 1

    intervals = [
        right.timestamp_s - left.timestamp_s
        for left, right in zip(accepted, accepted[1:])
    ]
    duration_s = None
    effective_rate_hz = None
    if len(accepted) >= 2:
        duration_s = accepted[-1].timestamp_s - accepted[0].timestamp_s
        if duration_s > 0.0:
            effective_rate_hz = (len(accepted) - 1) / duration_s

    maximum_interval_s = max(intervals) if intervals else None
    gap_count = sum(
        interval >= config.gap_threshold_s for interval in intervals
    )

    center_x_m = None
    center_y_m = None
    stationary_p95_error_m = None
    if accepted:
        center_x_m = statistics.median(sample.x_m for sample in accepted)
        center_y_m = statistics.median(sample.y_m for sample in accepted)
        if config.stationary:
            radial_errors = [
                math.hypot(
                    sample.x_m - center_x_m,
                    sample.y_m - center_y_m,
                )
                for sample in accepted
            ]
            stationary_p95_error_m = _nearest_rank(radial_errors, 0.95)

    rejected_count = sum(rejected_reasons.values())
    total_count = len(samples)
    failed_criteria = []
    if len(accepted) < config.minimum_accepted_fixes:
        failed_criteria.append('insufficient_accepted_fixes')
    if duration_s is None or duration_s < config.minimum_duration_s:
        failed_criteria.append('recording_duration_too_short')
    rejected_fraction = rejected_count / total_count if total_count else 0.0
    if rejected_fraction > config.maximum_rejected_fraction:
        failed_criteria.append('rejected_fraction_too_high')
    if gap_count:
        failed_criteria.append('gap_threshold_exceeded')
    if (
        config.stationary
        and stationary_p95_error_m is not None
        and stationary_p95_error_m
        > config.maximum_stationary_p95_error_m
    ):
        failed_criteria.append('stationary_p95_error_too_high')

    return RecordingAnalysis(
        total_count=total_count,
        accepted_count=len(accepted),
        rejected_count=rejected_count,
        degraded_count=degraded_count,
        rejected_reasons=dict(sorted(rejected_reasons.items())),
        duration_s=duration_s,
        effective_rate_hz=effective_rate_hz,
        median_interval_s=(
            statistics.median(intervals) if intervals else None
        ),
        p95_interval_s=_nearest_rank(intervals, 0.95),
        maximum_interval_s=maximum_interval_s,
        gap_count=gap_count,
        center_x_m=center_x_m,
        center_y_m=center_y_m,
        stationary_p95_error_m=stationary_p95_error_m,
        go=not failed_criteria,
        failed_criteria=tuple(failed_criteria),
    )


def _validate_analysis_config(config: AnalysisConfig) -> None:
    if (
        not math.isfinite(config.gap_threshold_s)
        or config.gap_threshold_s <= 0
    ):
        raise ValueError('gap_threshold_s must be finite and positive')
    if config.minimum_accepted_fixes < 1:
        raise ValueError('minimum_accepted_fixes must be positive')
    if (
        not math.isfinite(config.minimum_duration_s)
        or config.minimum_duration_s < 0.0
    ):
        raise ValueError('minimum_duration_s must be finite and non-negative')
    if not 0.0 <= config.maximum_rejected_fraction <= 1.0:
        raise ValueError('maximum_rejected_fraction must be between 0 and 1')
    if (
        not math.isfinite(config.maximum_stationary_p95_error_m)
        or config.maximum_stationary_p95_error_m <= 0.0
    ):
        raise ValueError(
            'maximum_stationary_p95_error_m must be finite and positive'
        )


def _nearest_rank(
    values: typing.Sequence[float], quantile: float
) -> typing.Optional[float]:
    if not values:
        return None
    ordered = sorted(values)
    index = max(0, math.ceil(quantile * len(ordered)) - 1)
    return ordered[index]
