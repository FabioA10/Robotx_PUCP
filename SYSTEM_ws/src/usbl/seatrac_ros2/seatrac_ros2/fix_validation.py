"""Hardware-independent USBL fix model, validation and covariance."""

from dataclasses import dataclass
import math
from typing import Optional, Tuple


@dataclass(frozen=True)
class UsblFix:
    """One normalized sample in the provisional ROS relative frame."""

    timestamp_s: float
    x_m: float
    y_m: float
    z_m: float
    beacon_id: Optional[int] = None
    range_m: Optional[float] = None
    fit_error: Optional[float] = None
    rssi_db: Optional[float] = None
    position_valid: bool = True
    position_filter_error: bool = False


@dataclass(frozen=True)
class ValidationConfig:
    """Conservative gates which can later be tuned from captured data."""

    expected_beacon_id: Optional[int] = None
    min_range_m: float = 0.0
    max_range_m: float = 1000.0
    warn_fit_error: float = 2.0
    reject_fit_error: float = 3.0
    max_horizontal_speed_mps: float = 3.0
    max_vertical_speed_mps: float = 1.5
    reject_position_filter_error: bool = True


@dataclass(frozen=True)
class ValidationResult:
    """Result returned without ROS dependencies."""

    accepted: bool
    degraded: bool
    reason: str
    horizontal_speed_mps: Optional[float] = None
    vertical_speed_mps: Optional[float] = None


class FixValidator:
    """Stateful validator; state changes only after an accepted sample."""

    def __init__(self, config: ValidationConfig) -> None:
        self.config = config
        self.last_accepted: Optional[UsblFix] = None

    def reset(self) -> None:
        """Forget the previous accepted sample, for a new replay run."""
        self.last_accepted = None

    def validate(self, sample: UsblFix) -> ValidationResult:
        """Validate one sample and remember it only when accepted."""
        scalar_values = (
            sample.timestamp_s,
            sample.x_m,
            sample.y_m,
            sample.z_m,
        )
        optional_values = (
            sample.range_m,
            sample.fit_error,
            sample.rssi_db,
        )
        if not all(math.isfinite(value) for value in scalar_values):
            return self._reject("non_finite_position_or_timestamp")
        if not all(
            value is None or math.isfinite(value)
            for value in optional_values
        ):
            return self._reject("non_finite_quality_value")
        if not sample.position_valid:
            return self._reject("position_not_valid")
        if (
            self.config.reject_position_filter_error
            and sample.position_filter_error
        ):
            return self._reject("position_filter_error")
        if (
            self.config.expected_beacon_id is not None
            and sample.beacon_id != self.config.expected_beacon_id
        ):
            return self._reject("unexpected_beacon_id")
        if sample.range_m is not None and not (
            self.config.min_range_m
            <= sample.range_m
            <= self.config.max_range_m
        ):
            return self._reject("range_out_of_bounds")
        if (
            sample.fit_error is not None
            and sample.fit_error >= self.config.reject_fit_error
        ):
            return self._reject("fit_error_too_high")

        horizontal_speed = None
        vertical_speed = None
        if self.last_accepted is not None:
            delta_t = sample.timestamp_s - self.last_accepted.timestamp_s
            if delta_t <= 0.0:
                return self._reject("non_monotonic_timestamp")
            delta_x = sample.x_m - self.last_accepted.x_m
            delta_y = sample.y_m - self.last_accepted.y_m
            delta_z = sample.z_m - self.last_accepted.z_m
            horizontal_speed = math.hypot(delta_x, delta_y) / delta_t
            vertical_speed = abs(delta_z) / delta_t
            if horizontal_speed > self.config.max_horizontal_speed_mps:
                return self._reject("horizontal_speed_too_high")
            if vertical_speed > self.config.max_vertical_speed_mps:
                return self._reject("vertical_speed_too_high")

        degraded = (
            sample.fit_error is not None
            and sample.fit_error >= self.config.warn_fit_error
        )
        self.last_accepted = sample
        return ValidationResult(
            accepted=True,
            degraded=degraded,
            reason="fit_error_warning" if degraded else "ok",
            horizontal_speed_mps=horizontal_speed,
            vertical_speed_mps=vertical_speed,
        )

    @staticmethod
    def _reject(reason: str) -> ValidationResult:
        return ValidationResult(False, False, reason)


def pose_covariance(
    sample: UsblFix,
    minimum_horizontal_sigma_m: float = 0.15,
    angle_sigma_deg: float = 5.0,
    vertical_sigma_m: float = 0.25,
) -> Tuple[float, ...]:
    """Build a conservative ROS 6x6 covariance for a position-only fix."""
    range_m = sample.range_m
    if range_m is None:
        range_m = math.sqrt(
            sample.x_m ** 2 + sample.y_m ** 2 + sample.z_m ** 2
        )
    angular_error_m = range_m * math.tan(math.radians(angle_sigma_deg))
    horizontal_sigma = max(
        minimum_horizontal_sigma_m,
        math.hypot(0.10, angular_error_m),
    )
    inflation = 1.0
    if sample.fit_error is not None:
        inflation += max(0.0, sample.fit_error - 2.0)

    covariance = [0.0] * 36
    covariance[0] = (horizontal_sigma * inflation) ** 2
    covariance[7] = (horizontal_sigma * inflation) ** 2
    covariance[14] = (vertical_sigma_m * inflation) ** 2
    covariance[21] = 1.0e6
    covariance[28] = 1.0e6
    covariance[35] = 1.0e6
    return tuple(covariance)
