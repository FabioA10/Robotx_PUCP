"""Robust preview controller for maintaining a radial band around a UUV."""

from dataclasses import dataclass
from enum import Enum
import math
from numbers import Real
from typing import Optional, Tuple

from .uuv_follow_model import RelativeFix


class RobustFollowState(str, Enum):
    """Observable states of the second-generation follower."""

    DISABLED = 'disabled'
    ACQUIRING = 'acquiring'
    ALIGNING = 'aligning'
    APPROACHING = 'approaching'
    RETREATING = 'retreating'
    HOLDING_BAND = 'holding_band'
    REVERSAL_NEUTRAL = 'reversal_neutral'
    HARD_STOP = 'hard_stop'
    LOST_TARGET = 'lost_target'
    REJECTED_FIX = 'rejected_fix'


class DriveDirection(str, Enum):
    """Hull direction selected to realize the desired horizontal motion."""

    FORWARD = 'forward'
    REVERSE = 'reverse'


@dataclass(frozen=True)
class RobustFollowConfig:
    """Conservative defaults intended for simulation and preview only."""

    inner_radius_m: float = 1.0
    outer_radius_m: float = 2.0
    target_radius_m: Optional[float] = None
    hard_stop_radius_m: float = 0.35
    radial_hysteresis_m: float = 0.15
    kp_radial: float = 0.80
    kd_radial: float = 0.35
    radial_velocity_alpha: float = 0.35
    kp_bearing: float = 1.20
    max_forward_speed_mps: float = 0.60
    max_reverse_speed_mps: float = 0.40
    max_angular_speed_radps: float = 0.60
    max_linear_accel_mps2: float = 0.35
    max_linear_decel_mps2: float = 0.60
    max_angular_accel_radps2: float = 0.80
    max_linear_jerk_mps3: float = 2.0
    max_angular_jerk_radps3: float = 4.0
    pivot_threshold_rad: float = math.radians(60.0)
    reverse_exit_angle_rad: float = math.radians(75.0)
    reverse_enter_angle_rad: float = math.radians(105.0)
    reversal_neutral_s: float = 0.40
    command_epsilon: float = 1.0e-6
    fix_timeout_s: float = 1.0
    acquisition_fixes: int = 3
    max_range_m: float = 20.0
    max_relative_speed_mps: float = 3.0
    min_quality: float = 0.0
    future_tolerance_s: float = 0.10
    expected_frame: str = 'base_link'
    require_covariance: bool = True
    max_horizontal_stddev_m: float = 0.35
    covariance_tolerance_m4: float = 1.0e-9
    initial_control_dt_s: float = 0.10
    max_control_dt_s: float = 0.25


@dataclass(frozen=True)
class RobustFollowDecision:
    """A bounded candidate command plus diagnostics for offline evaluation."""

    state: RobustFollowState
    linear_x_mps: float
    angular_z_radps: float
    reason: str
    direction: DriveDirection
    age_s: Optional[float] = None
    range_m: Optional[float] = None
    bearing_rad: Optional[float] = None
    radial_velocity_mps: Optional[float] = None
    linear_acceleration_mps2: float = 0.0
    angular_acceleration_radps2: float = 0.0


def _is_finite_real(value) -> bool:
    return (
        isinstance(value, Real)
        and not isinstance(value, bool)
        and math.isfinite(float(value))
    )


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def _wrap_to_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _move_toward(current: float, target: float, maximum_delta: float) -> float:
    if target > current:
        return min(target, current + maximum_delta)
    return max(target, current - maximum_delta)


def _sign(value: float, epsilon: float) -> int:
    if value > epsilon:
        return 1
    if value < -epsilon:
        return -1
    return 0


class RobustUuvFollowController:
    """Maintain a horizontal band using bounded forward or reverse motion.

    This class has no ROS or actuator dependencies. Invalid, stale, lost, or
    unconfirmed input always bypasses the rate limiter and returns exact zero.
    """

    def __init__(
        self,
        config: RobustFollowConfig = RobustFollowConfig(),
    ) -> None:
        """Create an inactive controller with validated immutable limits."""
        self.config = config
        self._validate_config()
        self._accepted_fixes = 0
        self._last_fix: Optional[RelativeFix] = None
        self._last_range_m: Optional[float] = None
        self._radial_velocity_mps = 0.0
        self._radial_state = RobustFollowState.HOLDING_BAND
        self._direction = DriveDirection.FORWARD
        self._linear_command_mps = 0.0
        self._angular_command_radps = 0.0
        self._linear_acceleration_mps2 = 0.0
        self._angular_acceleration_radps2 = 0.0
        self._last_control_time_s: Optional[float] = None
        self._pending_reversal_sign = 0
        self._neutral_until_s: Optional[float] = None

    def reset(self, now_s: Optional[float] = None) -> None:
        """Forget target history and return the internal command to neutral."""
        self._accepted_fixes = 0
        self._last_fix = None
        self._last_range_m = None
        self._radial_velocity_mps = 0.0
        self._radial_state = RobustFollowState.HOLDING_BAND
        self._direction = DriveDirection.FORWARD
        self._emergency_zero(now_s)

    def evaluate(
        self,
        fix: Optional[RelativeFix],
        now_s: float,
        enabled: bool,
    ) -> RobustFollowDecision:
        """Validate a fix and calculate a deterministic preview command."""
        if not _is_finite_real(now_s):
            self.reset()
            return self._stop(
                RobustFollowState.REJECTED_FIX,
                'control time is not finite',
            )

        if enabled is not True:
            self.reset(float(now_s))
            return self._stop(
                RobustFollowState.DISABLED,
                'follow is disabled',
            )

        if (
            self._last_control_time_s is not None
            and now_s + self.config.future_tolerance_s
            < self._last_control_time_s
        ):
            self.reset(float(now_s))
            return self._stop(
                RobustFollowState.REJECTED_FIX,
                'control clock moved backwards',
            )

        if fix is None:
            self.reset(float(now_s))
            return self._stop(
                RobustFollowState.LOST_TARGET,
                'no USBL fix received',
            )

        rejection = self._basic_rejection(fix, float(now_s))
        if rejection is not None:
            self._reject_acquisition(float(now_s))
            return self._stop(RobustFollowState.REJECTED_FIX, rejection)

        age_s = max(0.0, float(now_s) - float(fix.timestamp_s))
        range_m = math.hypot(float(fix.x_m), float(fix.y_m))
        bearing_rad = math.atan2(float(fix.y_m), float(fix.x_m))

        if age_s > self.config.fix_timeout_s:
            self.reset(float(now_s))
            return self._stop(
                RobustFollowState.LOST_TARGET,
                'USBL fix timed out',
                age_s,
                range_m,
                bearing_rad,
            )

        if range_m > self.config.max_range_m:
            self._reject_acquisition(float(now_s))
            return self._stop(
                RobustFollowState.REJECTED_FIX,
                'range is outside configured limits',
                age_s,
                range_m,
                bearing_rad,
            )

        is_new, sample_rejection = self._consume_fix(fix, range_m)
        if sample_rejection is not None:
            self._reject_acquisition(float(now_s))
            return self._stop(
                RobustFollowState.REJECTED_FIX,
                sample_rejection,
                age_s,
                range_m,
                bearing_rad,
            )

        if range_m <= self.config.hard_stop_radius_m:
            self._accepted_fixes = 0
            self._emergency_zero(float(now_s))
            return self._stop(
                RobustFollowState.HARD_STOP,
                'target is inside the hard-stop radius',
                age_s,
                range_m,
                bearing_rad,
            )

        if self._accepted_fixes < self.config.acquisition_fixes:
            self._emergency_zero(float(now_s))
            return self._stop(
                RobustFollowState.ACQUIRING,
                'acquiring consecutive USBL fixes',
                age_s,
                range_m,
                bearing_rad,
            )

        if not is_new and self._last_fix is None:
            self._emergency_zero(float(now_s))
            return self._stop(
                RobustFollowState.ACQUIRING,
                'waiting for a new USBL fix',
                age_s,
                range_m,
                bearing_rad,
            )

        self._update_radial_state(range_m)
        if self._radial_state == RobustFollowState.HOLDING_BAND:
            linear, angular, reversing = self._apply_limits(
                0.0,
                0.0,
                float(now_s),
            )
            state = (
                RobustFollowState.REVERSAL_NEUTRAL
                if reversing
                else RobustFollowState.HOLDING_BAND
            )
            return self._decision(
                state,
                linear,
                angular,
                'inside the radial holding band',
                age_s,
                range_m,
                bearing_rad,
            )

        radial_error = range_m - self._target_radius_m()
        closure_command = (
            self.config.kp_radial * radial_error
            + self.config.kd_radial * self._radial_velocity_mps
        )
        motion_angle = (
            bearing_rad
            if self._radial_state == RobustFollowState.APPROACHING
            else _wrap_to_pi(bearing_rad + math.pi)
        )
        self._select_drive_direction(motion_angle)
        steering_error = (
            motion_angle
            if self._direction == DriveDirection.FORWARD
            else _wrap_to_pi(motion_angle - math.pi)
        )
        angular_target = _clamp(
            self.config.kp_bearing * steering_error,
            -self.config.max_angular_speed_radps,
            self.config.max_angular_speed_radps,
        )

        speed_limit = (
            self.config.max_forward_speed_mps
            if self._direction == DriveDirection.FORWARD
            else self.config.max_reverse_speed_mps
        )
        closure_speed = (
            max(0.0, closure_command)
            if self._radial_state == RobustFollowState.APPROACHING
            else max(0.0, -closure_command)
        )
        speed_target = min(closure_speed, speed_limit)
        alignment = max(0.0, math.cos(steering_error))
        linear_target = (
            self._direction_sign() * speed_target * alignment * alignment
        )

        aligning = abs(steering_error) > self.config.pivot_threshold_rad
        if aligning:
            linear_target = 0.0

        linear, angular, reversing = self._apply_limits(
            linear_target,
            angular_target,
            float(now_s),
        )
        if reversing:
            state = RobustFollowState.REVERSAL_NEUTRAL
            reason = 'braking and holding neutral before reversing'
        elif aligning:
            state = RobustFollowState.ALIGNING
            reason = 'aligning hull with desired motion'
        else:
            state = self._radial_state
            reason = (
                'closing excessive horizontal range'
                if state == RobustFollowState.APPROACHING
                else 'increasing insufficient horizontal range'
            )
        return self._decision(
            state,
            linear,
            angular,
            reason,
            age_s,
            range_m,
            bearing_rad,
        )

    def _consume_fix(
        self,
        fix: RelativeFix,
        range_m: float,
    ) -> Tuple[bool, Optional[str]]:
        if self._last_fix is None:
            self._last_fix = fix
            self._last_range_m = range_m
            self._radial_velocity_mps = 0.0
            self._accepted_fixes = 1
            return True, None

        if fix.timestamp_s < self._last_fix.timestamp_s:
            return False, 'USBL timestamp moved backwards'
        if fix.timestamp_s == self._last_fix.timestamp_s:
            if not self._same_sample(fix, self._last_fix):
                return False, 'USBL sample changed without a new timestamp'
            return False, None

        elapsed_s = float(fix.timestamp_s - self._last_fix.timestamp_s)
        displacement_m = math.hypot(
            float(fix.x_m - self._last_fix.x_m),
            float(fix.y_m - self._last_fix.y_m),
        )
        if displacement_m / elapsed_s > self.config.max_relative_speed_mps:
            return False, 'relative-position jump exceeds speed limit'

        previous_range = self._last_range_m
        if self._accepted_fixes == 0 or previous_range is None:
            self._radial_velocity_mps = 0.0
        else:
            raw_velocity = (range_m - previous_range) / elapsed_s
            alpha = self.config.radial_velocity_alpha
            self._radial_velocity_mps = (
                alpha * raw_velocity
                + (1.0 - alpha) * self._radial_velocity_mps
            )
        self._last_fix = fix
        self._last_range_m = range_m
        self._accepted_fixes += 1
        return True, None

    def _update_radial_state(self, range_m: float) -> None:
        if self._radial_state == RobustFollowState.APPROACHING:
            if (
                range_m
                <= self.config.outer_radius_m
                - self.config.radial_hysteresis_m
            ):
                self._radial_state = RobustFollowState.HOLDING_BAND
            return
        if self._radial_state == RobustFollowState.RETREATING:
            if (
                range_m
                >= self.config.inner_radius_m
                + self.config.radial_hysteresis_m
            ):
                self._radial_state = RobustFollowState.HOLDING_BAND
            return
        if range_m > self.config.outer_radius_m:
            self._radial_state = RobustFollowState.APPROACHING
        elif range_m < self.config.inner_radius_m:
            self._radial_state = RobustFollowState.RETREATING

    def _target_radius_m(self) -> float:
        if self.config.target_radius_m is not None:
            return self.config.target_radius_m
        return 0.5 * (
            self.config.inner_radius_m + self.config.outer_radius_m
        )

    def _select_drive_direction(self, motion_angle: float) -> None:
        angle = abs(_wrap_to_pi(motion_angle))
        if (
            self._direction == DriveDirection.FORWARD
            and angle >= self.config.reverse_enter_angle_rad
        ):
            self._direction = DriveDirection.REVERSE
        elif (
            self._direction == DriveDirection.REVERSE
            and angle <= self.config.reverse_exit_angle_rad
        ):
            self._direction = DriveDirection.FORWARD

    def _apply_limits(
        self,
        linear_target: float,
        angular_target: float,
        now_s: float,
    ) -> Tuple[float, float, bool]:
        dt_s = self._control_dt(now_s)
        requested_sign = _sign(linear_target, self.config.command_epsilon)
        current_sign = _sign(
            self._linear_command_mps,
            self.config.command_epsilon,
        )

        if requested_sign and current_sign and requested_sign != current_sign:
            self._pending_reversal_sign = requested_sign
            self._neutral_until_s = None
        elif (
            self._pending_reversal_sign
            and requested_sign == current_sign
        ):
            self._pending_reversal_sign = 0
            self._neutral_until_s = None
        elif requested_sign == 0:
            self._pending_reversal_sign = 0
            self._neutral_until_s = None

        reversing = self._pending_reversal_sign != 0
        if reversing:
            (
                self._linear_command_mps,
                self._linear_acceleration_mps2,
            ) = self._slew_axis(
                self._linear_command_mps,
                self._linear_acceleration_mps2,
                0.0,
                self.config.max_linear_decel_mps2,
                self.config.max_linear_jerk_mps3,
                dt_s,
            )
            if (
                abs(self._linear_command_mps)
                <= self.config.command_epsilon
            ):
                self._linear_command_mps = 0.0
                if self._neutral_until_s is None:
                    self._neutral_until_s = (
                        now_s + self.config.reversal_neutral_s
                    )
                if now_s >= self._neutral_until_s:
                    self._pending_reversal_sign = 0
                    self._neutral_until_s = None
                    reversing = False
        else:
            same_sign = requested_sign == current_sign or current_sign == 0
            accelerating = (
                same_sign
                and abs(linear_target) > abs(self._linear_command_mps)
            )
            acceleration_limit = (
                self.config.max_linear_accel_mps2
                if accelerating
                else self.config.max_linear_decel_mps2
            )
            (
                self._linear_command_mps,
                self._linear_acceleration_mps2,
            ) = self._slew_axis(
                self._linear_command_mps,
                self._linear_acceleration_mps2,
                linear_target,
                acceleration_limit,
                self.config.max_linear_jerk_mps3,
                dt_s,
            )

        (
            self._angular_command_radps,
            self._angular_acceleration_radps2,
        ) = self._slew_axis(
            self._angular_command_radps,
            self._angular_acceleration_radps2,
            angular_target,
            self.config.max_angular_accel_radps2,
            self.config.max_angular_jerk_radps3,
            dt_s,
        )
        return (
            self._linear_command_mps,
            self._angular_command_radps,
            reversing,
        )

    @staticmethod
    def _slew_axis(
        current: float,
        current_rate: float,
        target: float,
        rate_limit: float,
        jerk_limit: float,
        dt_s: float,
    ) -> Tuple[float, float]:
        """Move one command axis with bounded first and second derivative."""
        if dt_s <= 0.0:
            return current, current_rate
        error = target - current
        if abs(error) <= 1.0e-15:
            return target, 0.0
        desired_rate = math.copysign(rate_limit, error)
        next_rate = _move_toward(
            current_rate,
            desired_rate,
            jerk_limit * dt_s,
        )
        next_value = current + next_rate * dt_s
        if error * (target - next_value) <= 0.0:
            return target, 0.0
        return next_value, next_rate

    def _control_dt(self, now_s: float) -> float:
        if self._last_control_time_s is None:
            dt_s = self.config.initial_control_dt_s
        else:
            dt_s = max(0.0, now_s - self._last_control_time_s)
            dt_s = min(dt_s, self.config.max_control_dt_s)
        self._last_control_time_s = now_s
        return dt_s

    def _basic_rejection(
        self,
        fix: RelativeFix,
        now_s: float,
    ) -> Optional[str]:
        values = (fix.x_m, fix.y_m, fix.timestamp_s)
        if not all(_is_finite_real(value) for value in values):
            return 'fix contains a non-finite or non-numeric value'
        if not isinstance(fix.valid, bool) or not fix.valid:
            return 'USBL marked the fix invalid'
        if fix.timestamp_s > now_s + self.config.future_tolerance_s:
            return 'USBL timestamp is in the future'
        if fix.frame_id != self.config.expected_frame:
            return 'USBL fix is not expressed in the command frame'
        if fix.quality is not None:
            if not _is_finite_real(fix.quality):
                return 'fix quality is not finite or numeric'
            if not 0.0 <= fix.quality <= 1.0:
                return 'fix quality is outside [0, 1]'
            if fix.quality < self.config.min_quality:
                return 'fix quality is below threshold'
        return self._covariance_rejection(fix)

    def _covariance_rejection(self, fix: RelativeFix) -> Optional[str]:
        covariance = (
            fix.variance_x_m2,
            fix.variance_y_m2,
            fix.covariance_xy_m2,
        )
        if any(value is None for value in covariance):
            if self.config.require_covariance:
                return 'USBL horizontal covariance is missing'
            return None
        if not all(_is_finite_real(value) for value in covariance):
            return 'USBL horizontal covariance is not finite'
        variance_x, variance_y, covariance_xy = covariance
        if variance_x < 0.0 or variance_y < 0.0:
            return 'USBL horizontal variance is negative'
        determinant = variance_x * variance_y - covariance_xy * covariance_xy
        if determinant < -self.config.covariance_tolerance_m4:
            return 'USBL horizontal covariance is not positive semidefinite'
        variance_limit = self.config.max_horizontal_stddev_m ** 2
        if max(variance_x, variance_y) > variance_limit:
            return 'USBL horizontal uncertainty exceeds the configured limit'
        return None

    @staticmethod
    def _same_sample(first: RelativeFix, second: RelativeFix) -> bool:
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

    def _direction_sign(self) -> float:
        return 1.0 if self._direction == DriveDirection.FORWARD else -1.0

    def _reject_acquisition(self, now_s: float) -> None:
        self._accepted_fixes = 0
        self._radial_velocity_mps = 0.0
        self._radial_state = RobustFollowState.HOLDING_BAND
        self._emergency_zero(now_s)

    def _emergency_zero(self, now_s: Optional[float]) -> None:
        self._linear_command_mps = 0.0
        self._angular_command_radps = 0.0
        self._linear_acceleration_mps2 = 0.0
        self._angular_acceleration_radps2 = 0.0
        self._pending_reversal_sign = 0
        self._neutral_until_s = None
        self._last_control_time_s = now_s

    def _decision(
        self,
        state: RobustFollowState,
        linear_x_mps: float,
        angular_z_radps: float,
        reason: str,
        age_s: Optional[float],
        range_m: Optional[float],
        bearing_rad: Optional[float],
    ) -> RobustFollowDecision:
        return RobustFollowDecision(
            state=state,
            linear_x_mps=linear_x_mps,
            angular_z_radps=angular_z_radps,
            reason=reason,
            direction=self._direction,
            age_s=age_s,
            range_m=range_m,
            bearing_rad=bearing_rad,
            radial_velocity_mps=self._radial_velocity_mps,
            linear_acceleration_mps2=self._linear_acceleration_mps2,
            angular_acceleration_radps2=self._angular_acceleration_radps2,
        )

    def _stop(
        self,
        state: RobustFollowState,
        reason: str,
        age_s: Optional[float] = None,
        range_m: Optional[float] = None,
        bearing_rad: Optional[float] = None,
    ) -> RobustFollowDecision:
        return self._decision(
            state,
            0.0,
            0.0,
            reason,
            age_s,
            range_m,
            bearing_rad,
        )

    def _validate_config(self) -> None:
        positive = (
            self.config.inner_radius_m,
            self.config.outer_radius_m,
            self.config.hard_stop_radius_m,
            self.config.max_forward_speed_mps,
            self.config.max_reverse_speed_mps,
            self.config.max_angular_speed_radps,
            self.config.max_linear_accel_mps2,
            self.config.max_linear_decel_mps2,
            self.config.max_angular_accel_radps2,
            self.config.max_linear_jerk_mps3,
            self.config.max_angular_jerk_radps3,
            self.config.fix_timeout_s,
            self.config.max_range_m,
            self.config.max_relative_speed_mps,
            self.config.max_horizontal_stddev_m,
            self.config.initial_control_dt_s,
            self.config.max_control_dt_s,
        )
        if not all(_is_finite_real(value) and value > 0.0 for value in positive):
            raise ValueError('positive limits must be finite and greater than zero')
        if not (
            self.config.hard_stop_radius_m
            < self.config.inner_radius_m
            < self.config.outer_radius_m
            < self.config.max_range_m
        ):
            raise ValueError('radii must satisfy hard < inner < outer < max')
        target_radius = self._target_radius_m()
        if not _is_finite_real(target_radius) or not (
            self.config.inner_radius_m
            < target_radius
            < self.config.outer_radius_m
        ):
            raise ValueError('target_radius_m must be strictly inside the band')
        nonnegative = (
            self.config.radial_hysteresis_m,
            self.config.kp_radial,
            self.config.kd_radial,
            self.config.kp_bearing,
            self.config.reversal_neutral_s,
            self.config.command_epsilon,
            self.config.future_tolerance_s,
            self.config.covariance_tolerance_m4,
        )
        if not all(
            _is_finite_real(value) and value >= 0.0
            for value in nonnegative
        ):
            raise ValueError('gains and tolerances must be finite and non-negative')
        if self.config.radial_hysteresis_m >= min(
            self.config.inner_radius_m - self.config.hard_stop_radius_m,
            self.config.outer_radius_m - self.config.inner_radius_m,
        ):
            raise ValueError('radial_hysteresis_m is too large for the radii')
        if not _is_finite_real(self.config.radial_velocity_alpha) or not (
            0.0 <= self.config.radial_velocity_alpha <= 1.0
        ):
            raise ValueError('radial_velocity_alpha must be in [0, 1]')
        if not (
            0.0 < self.config.reverse_exit_angle_rad < math.pi / 2.0
            and math.pi / 2.0
            < self.config.reverse_enter_angle_rad
            < math.pi
        ):
            raise ValueError('reverse angles must straddle pi/2')
        if not (
            _is_finite_real(self.config.pivot_threshold_rad)
            and 0.0 < self.config.pivot_threshold_rad <= math.pi
        ):
            raise ValueError('pivot_threshold_rad must be in (0, pi]')
        if (
            not isinstance(self.config.acquisition_fixes, int)
            or isinstance(self.config.acquisition_fixes, bool)
            or self.config.acquisition_fixes < 1
        ):
            raise ValueError('acquisition_fixes must be a positive integer')
        if not _is_finite_real(self.config.min_quality) or not (
            0.0 <= self.config.min_quality <= 1.0
        ):
            raise ValueError('min_quality must be in [0, 1]')
        if not self.config.expected_frame:
            raise ValueError('expected_frame must not be empty')
        if not isinstance(self.config.require_covariance, bool):
            raise ValueError('require_covariance must be boolean')
