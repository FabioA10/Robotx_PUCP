"""Pure, fail-safe controller for following a relative UUV position."""

from dataclasses import dataclass
from enum import Enum
import math
from numbers import Real
from typing import Optional


class FollowState(str, Enum):
    """Observable states of the follower."""

    DISABLED = "disabled"
    WAITING_FIX = "waiting_fix"
    TRACKING = "tracking"
    HOLDING = "holding"
    LOST_TARGET = "lost_target"
    REJECTED_FIX = "rejected_fix"


@dataclass(frozen=True)
class RelativeFix:
    """UUV position in the USBL frame: x forward and y left, in metres."""

    x_m: float
    y_m: float
    timestamp_s: float
    valid: bool = True
    quality: Optional[float] = None
    frame_id: str = "base_link"
    variance_x_m2: Optional[float] = None
    variance_y_m2: Optional[float] = None
    covariance_xy_m2: Optional[float] = None


@dataclass(frozen=True)
class FollowConfig:
    """Conservative limits suitable for initial pool tests."""

    follow_distance_m: float = 1.0
    distance_deadband_m: float = 0.20
    kp_distance: float = 0.50
    kp_bearing: float = 0.80
    max_linear_speed_mps: float = 0.20
    max_angular_speed_radps: float = 0.30
    pivot_threshold_rad: float = 0.60
    fix_timeout_s: float = 2.50
    acquisition_fixes: int = 3
    min_range_m: float = 0.05
    max_range_m: float = 20.0
    max_relative_speed_mps: float = 3.0
    min_quality: float = 0.0
    future_tolerance_s: float = 0.10
    expected_frame: str = "base_link"
    require_covariance: bool = True
    max_horizontal_stddev_m: float = 0.75
    covariance_tolerance_m4: float = 1.0e-9


@dataclass(frozen=True)
class FollowDecision:
    """A bounded preview command plus enough data for diagnostics."""

    state: FollowState
    linear_x_mps: float
    angular_z_radps: float
    reason: str
    age_s: Optional[float] = None
    range_m: Optional[float] = None
    bearing_rad: Optional[float] = None


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def _is_finite_number(value) -> bool:
    """Accept real finite scalars, but never Python booleans or strings."""
    return (
        isinstance(value, Real)
        and not isinstance(value, bool)
        and math.isfinite(float(value))
    )


class UuvFollowController:
    """
    Validate relative USBL fixes and calculate bounded preview commands.

    The controller intentionally never commands reverse motion. Any invalid,
    stale or unconfirmed input produces a zero command.
    """

    def __init__(self, config: FollowConfig = FollowConfig()) -> None:
        self.config = config
        self._validate_config()
        self._accepted_fixes = 0
        self._last_fix: Optional[RelativeFix] = None

    def reset(self) -> None:
        """Forget target acquisition history."""
        self._accepted_fixes = 0
        self._last_fix = None

    def evaluate(
        self,
        fix: Optional[RelativeFix],
        now_s: float,
        enabled: bool,
    ) -> FollowDecision:
        """Validate one fix and calculate a safe preview command."""
        if enabled is not True:
            self.reset()
            return self._stop(FollowState.DISABLED, "follow is disabled")

        if fix is None:
            self.reset()
            return self._stop(FollowState.WAITING_FIX, "no USBL fix received")

        rejection = self._basic_rejection(fix, now_s)
        if rejection is not None:
            self._accepted_fixes = 0
            return self._stop(FollowState.REJECTED_FIX, rejection)

        age_s = max(0.0, now_s - fix.timestamp_s)
        range_m = math.hypot(fix.x_m, fix.y_m)
        bearing_rad = math.atan2(fix.y_m, fix.x_m)

        if age_s > self.config.fix_timeout_s:
            self.reset()
            return self._stop(
                FollowState.LOST_TARGET,
                "USBL fix timed out",
                age_s,
                range_m,
                bearing_rad,
            )

        if not (self.config.min_range_m <= range_m <= self.config.max_range_m):
            self.reset()
            return self._stop(
                FollowState.REJECTED_FIX,
                "range is outside configured limits",
                age_s,
                range_m,
                bearing_rad,
            )

        if self._last_fix is None or fix.timestamp_s > self._last_fix.timestamp_s:
            jump_reason = self._jump_rejection(fix)
            if jump_reason is not None:
                # Keep the last trusted sample: otherwise one repeated outlier
                # would become the new baseline after its first rejection.
                self._accepted_fixes = 0
                return self._stop(
                    FollowState.REJECTED_FIX,
                    jump_reason,
                    age_s,
                    range_m,
                    bearing_rad,
                )
            self._last_fix = fix
            self._accepted_fixes += 1
        elif fix.timestamp_s < self._last_fix.timestamp_s:
            self._accepted_fixes = 0
            return self._stop(
                FollowState.REJECTED_FIX,
                "USBL timestamp moved backwards",
                age_s,
                range_m,
                bearing_rad,
            )
        elif not self._same_sample(fix, self._last_fix):
            self._accepted_fixes = 0
            return self._stop(
                FollowState.REJECTED_FIX,
                "USBL sample changed without a new timestamp",
                age_s,
                range_m,
                bearing_rad,
            )

        if self._accepted_fixes < self.config.acquisition_fixes:
            return self._stop(
                FollowState.WAITING_FIX,
                "acquiring consecutive USBL fixes",
                age_s,
                range_m,
                bearing_rad,
            )

        distance_error = range_m - self.config.follow_distance_m
        if distance_error <= self.config.distance_deadband_m:
            return self._stop(
                FollowState.HOLDING,
                "inside stand-off distance",
                age_s,
                range_m,
                bearing_rad,
            )

        angular_z = _clamp(
            self.config.kp_bearing * bearing_rad,
            -self.config.max_angular_speed_radps,
            self.config.max_angular_speed_radps,
        )

        if abs(bearing_rad) > self.config.pivot_threshold_rad:
            linear_x = 0.0
            reason = "pivoting toward target"
        else:
            requested = self.config.kp_distance * distance_error
            linear_x = _clamp(
                requested * max(0.0, math.cos(bearing_rad)),
                0.0,
                self.config.max_linear_speed_mps,
            )
            reason = "following target"

        return FollowDecision(
            state=FollowState.TRACKING,
            linear_x_mps=linear_x,
            angular_z_radps=angular_z,
            reason=reason,
            age_s=age_s,
            range_m=range_m,
            bearing_rad=bearing_rad,
        )

    def _basic_rejection(self, fix: RelativeFix, now_s: float) -> Optional[str]:
        values = (fix.x_m, fix.y_m, fix.timestamp_s, now_s)
        if not all(_is_finite_number(value) for value in values):
            return "fix contains a non-finite or non-numeric value"
        if not isinstance(fix.valid, bool) or not fix.valid:
            return "USBL marked the fix invalid"
        if fix.timestamp_s > now_s + self.config.future_tolerance_s:
            return "USBL timestamp is in the future"
        if fix.frame_id != self.config.expected_frame:
            return "USBL fix is not expressed in the command frame"
        if fix.quality is not None:
            if not _is_finite_number(fix.quality):
                return "fix quality is not finite or numeric"
            if not 0.0 <= fix.quality <= 1.0:
                return "fix quality is outside [0, 1]"
            if fix.quality < self.config.min_quality:
                return "fix quality is below threshold"
        covariance_rejection = self._covariance_rejection(fix)
        if covariance_rejection is not None:
            return covariance_rejection
        return None

    def _covariance_rejection(self, fix: RelativeFix) -> Optional[str]:
        covariance = (
            fix.variance_x_m2,
            fix.variance_y_m2,
            fix.covariance_xy_m2,
        )
        if any(value is None for value in covariance):
            if self.config.require_covariance:
                return "USBL horizontal covariance is missing"
            return None
        if not all(_is_finite_number(value) for value in covariance):
            return "USBL horizontal covariance is not finite"
        variance_x, variance_y, covariance_xy = covariance
        if variance_x < 0.0 or variance_y < 0.0:
            return "USBL horizontal variance is negative"
        determinant = variance_x * variance_y - covariance_xy * covariance_xy
        if determinant < -self.config.covariance_tolerance_m4:
            return "USBL horizontal covariance is not positive semidefinite"
        # Bound the worst horizontal direction, not only the X/Y axes.  A
        # strongly correlated covariance can have acceptable diagonal terms
        # while its principal-axis uncertainty exceeds the configured limit.
        trace = variance_x + variance_y
        discriminant = math.hypot(
            variance_x - variance_y,
            2.0 * covariance_xy,
        )
        maximum_eigenvalue = 0.5 * (trace + discriminant)
        variance_limit = self.config.max_horizontal_stddev_m ** 2
        if maximum_eigenvalue > variance_limit:
            return "USBL horizontal uncertainty exceeds the configured limit"
        return None

    @staticmethod
    def _same_sample(first: RelativeFix, second: RelativeFix) -> bool:
        """Detect mutated data carrying an already consumed timestamp."""
        return (
            first.x_m == second.x_m
            and first.y_m == second.y_m
            and first.valid == second.valid
            and first.quality == second.quality
            and first.frame_id == second.frame_id
            and first.variance_x_m2 == second.variance_x_m2
            and first.variance_y_m2 == second.variance_y_m2
            and first.covariance_xy_m2 == second.covariance_xy_m2
        )

    def _jump_rejection(self, fix: RelativeFix) -> Optional[str]:
        if self._last_fix is None:
            return None
        elapsed = fix.timestamp_s - self._last_fix.timestamp_s
        if elapsed <= 0.0:
            return None
        displacement = math.hypot(
            fix.x_m - self._last_fix.x_m,
            fix.y_m - self._last_fix.y_m,
        )
        if displacement / elapsed > self.config.max_relative_speed_mps:
            return "relative-position jump exceeds speed limit"
        return None

    def _stop(
        self,
        state: FollowState,
        reason: str,
        age_s: Optional[float] = None,
        range_m: Optional[float] = None,
        bearing_rad: Optional[float] = None,
    ) -> FollowDecision:
        return FollowDecision(
            state=state,
            linear_x_mps=0.0,
            angular_z_radps=0.0,
            reason=reason,
            age_s=age_s,
            range_m=range_m,
            bearing_rad=bearing_rad,
        )

    def _validate_config(self) -> None:
        if (
            not isinstance(self.config.acquisition_fixes, int)
            or isinstance(self.config.acquisition_fixes, bool)
            or self.config.acquisition_fixes < 1
        ):
            raise ValueError("acquisition_fixes must be at least one")
        positive = (
            self.config.follow_distance_m,
            self.config.fix_timeout_s,
            self.config.max_range_m,
            self.config.max_relative_speed_mps,
            self.config.max_horizontal_stddev_m,
        )
        if not all(_is_finite_number(value) and value > 0.0 for value in positive):
            raise ValueError("distance, timeout, range and speed must be positive")
        if self.config.min_range_m >= self.config.max_range_m:
            raise ValueError("min_range_m must be less than max_range_m")
        nonnegative = (
            self.config.distance_deadband_m,
            self.config.kp_distance,
            self.config.kp_bearing,
            self.config.max_linear_speed_mps,
            self.config.max_angular_speed_radps,
            self.config.min_range_m,
            self.config.future_tolerance_s,
            self.config.covariance_tolerance_m4,
        )
        if not all(
            _is_finite_number(value) and value >= 0.0
            for value in nonnegative
        ):
            raise ValueError("controller gains and limits must be non-negative")
        if not _is_finite_number(self.config.pivot_threshold_rad) or not (
            0.0 < self.config.pivot_threshold_rad <= math.pi
        ):
            raise ValueError("pivot_threshold_rad must be in (0, pi]")
        if not _is_finite_number(self.config.min_quality) or not (
            0.0 <= self.config.min_quality <= 1.0
        ):
            raise ValueError("min_quality must be in [0, 1]")
        if not self.config.expected_frame:
            raise ValueError("expected_frame must not be empty")
        if not isinstance(self.config.require_covariance, bool):
            raise ValueError("require_covariance must be boolean")
