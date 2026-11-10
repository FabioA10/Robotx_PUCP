"""Fail-safe command arbitration model with no ROS or actuator dependency."""

from dataclasses import dataclass
from enum import Enum
import math
from numbers import Real
from typing import Optional


class ArbiterMode(str, Enum):
    """Mutually exclusive command sources."""

    DISABLED = 'disabled'
    MANUAL = 'manual'
    MISSION = 'mission'
    UUV_FOLLOW = 'uuv_follow'


class ArbiterState(str, Enum):
    """Externally observable shadow-arbiter states."""

    DISABLED = 'disabled'
    BLOCKED = 'blocked'
    ACTIVE = 'active'
    ABORTED = 'aborted'


@dataclass(frozen=True)
class CandidateCommand:
    """A timestamped velocity request from a named source."""

    linear_x_mps: float
    angular_z_radps: float
    timestamp_s: float
    source: str = 'uuv_follow'
    frame_id: str = 'base_link'


@dataclass(frozen=True)
class ArbiterConfig:
    """Conservative limits for shadow validation."""

    command_timeout_s: float = 0.30
    safety_gate_timeout_s: float = 0.50
    future_tolerance_s: float = 0.10
    max_clock_step_s: float = 1.00
    max_linear_speed_mps: float = 0.20
    max_angular_speed_radps: float = 0.30
    max_linear_acceleration_mps2: float = 0.20
    max_angular_acceleration_radps2: float = 0.50
    expected_source: str = 'uuv_follow'
    expected_frame_id: str = 'base_link'


@dataclass(frozen=True)
class ArbiterDecision:
    """Final shadow command and audit context."""

    state: ArbiterState
    mode: ArbiterMode
    linear_x_mps: float
    angular_z_radps: float
    reason: str
    input_age_s: Optional[float] = None
    abort_latched: bool = False


def _finite(value) -> bool:
    return (
        isinstance(value, Real)
        and not isinstance(value, bool)
        and math.isfinite(float(value))
    )


def _toward(current: float, target: float, maximum_delta: float) -> float:
    if target > current:
        return min(target, current + maximum_delta)
    return max(target, current - maximum_delta)


class ShadowCommandArbiter:
    """
    Validate and rate-limit candidate commands without actuating hardware.

    A safety fault latches an abort. Recovery requires an explicit disabled
    evaluation before the operator can enable the shadow path again.
    """

    def __init__(self, config: ArbiterConfig = ArbiterConfig()) -> None:
        """Create a neutral arbiter and validate all configured limits."""
        self.config = config
        self._validate_config()
        self._abort_latched = False
        self._last_mode = ArbiterMode.DISABLED
        self._last_time_s: Optional[float] = None
        self._linear_x_mps = 0.0
        self._angular_z_radps = 0.0

    def evaluate(
        self,
        candidate: Optional[CandidateCommand],
        now_s: float,
        requested_mode: ArbiterMode,
        operator_enabled: bool,
        vehicle_connected: bool,
        kill_active: bool,
        vehicle_state_timestamp_s: Optional[float] = None,
        kill_state_timestamp_s: Optional[float] = None,
    ) -> ArbiterDecision:
        """Return a bounded shadow decision; unsafe input always returns zero."""
        if not _finite(now_s):
            return self._abort(requested_mode, 'control clock is invalid')

        now_s = float(now_s)
        if operator_enabled is not True:
            self._abort_latched = False
            self._last_mode = ArbiterMode.DISABLED
            self._neutral(now_s)
            return self._decision(
                ArbiterState.DISABLED,
                ArbiterMode.DISABLED,
                'operator enable is off',
            )

        if not isinstance(requested_mode, ArbiterMode):
            return self._abort(
                ArbiterMode.DISABLED,
                'requested mode is invalid; disable before re-enabling',
            )

        if requested_mode == ArbiterMode.DISABLED:
            self._abort_latched = False
            self._last_mode = ArbiterMode.DISABLED
            self._neutral(now_s)
            return self._decision(
                ArbiterState.DISABLED,
                ArbiterMode.DISABLED,
                'requested mode is disabled',
            )

        clock_rejection = self._clock_rejection(now_s)
        if clock_rejection is not None:
            return self._abort(requested_mode, clock_rejection)

        vehicle_rejection = self._gate_rejection(
            'vehicle connection', vehicle_state_timestamp_s, now_s
        )
        if vehicle_rejection is not None:
            return self._abort(requested_mode, vehicle_rejection)

        kill_rejection = self._gate_rejection(
            'killswitch', kill_state_timestamp_s, now_s
        )
        if kill_rejection is not None:
            return self._abort(requested_mode, kill_rejection)

        if kill_active is not False:
            return self._abort(requested_mode, 'killswitch is active')

        if vehicle_connected is not True:
            return self._abort(requested_mode, 'vehicle state is not connected')

        if self._abort_latched:
            self._neutral(now_s)
            return self._decision(
                ArbiterState.ABORTED,
                requested_mode,
                'abort is latched; disable before re-enabling',
            )

        if requested_mode != self._last_mode:
            self._last_mode = requested_mode
            self._neutral(now_s)
            return self._decision(
                ArbiterState.BLOCKED,
                requested_mode,
                'mode changed; neutral cycle required',
            )

        if requested_mode != ArbiterMode.UUV_FOLLOW:
            self._neutral(now_s)
            return self._decision(
                ArbiterState.BLOCKED,
                requested_mode,
                'selected source has no shadow input',
            )

        if candidate is None:
            self._neutral(now_s)
            return self._decision(
                ArbiterState.BLOCKED,
                requested_mode,
                'no candidate command received',
            )

        rejection = self._candidate_rejection(candidate, now_s)
        if rejection is not None:
            return self._abort(requested_mode, rejection)

        age_s = max(0.0, now_s - float(candidate.timestamp_s))
        dt_s = now_s - float(self._last_time_s)
        self._linear_x_mps = _toward(
            self._linear_x_mps,
            float(candidate.linear_x_mps),
            self.config.max_linear_acceleration_mps2 * dt_s,
        )
        self._angular_z_radps = _toward(
            self._angular_z_radps,
            float(candidate.angular_z_radps),
            self.config.max_angular_acceleration_radps2 * dt_s,
        )
        self._last_time_s = now_s
        return self._decision(
            ArbiterState.ACTIVE,
            requested_mode,
            'candidate accepted in shadow mode',
            age_s,
        )

    def _candidate_rejection(
        self, candidate: CandidateCommand, now_s: float
    ) -> Optional[str]:
        if candidate.source != self.config.expected_source:
            return 'candidate source does not match selected mode'
        if candidate.frame_id != self.config.expected_frame_id:
            return 'candidate frame does not match expected frame'
        values = (
            candidate.linear_x_mps,
            candidate.angular_z_radps,
            candidate.timestamp_s,
        )
        if not all(_finite(value) for value in values):
            return 'candidate contains a non-finite value'
        age_s = now_s - float(candidate.timestamp_s)
        if age_s < -self.config.future_tolerance_s:
            return 'candidate timestamp is in the future'
        if age_s > self.config.command_timeout_s:
            return 'candidate command timed out'
        if abs(float(candidate.linear_x_mps)) > self.config.max_linear_speed_mps:
            return 'candidate exceeds linear speed limit'
        if (
            abs(float(candidate.angular_z_radps))
            > self.config.max_angular_speed_radps
        ):
            return 'candidate exceeds angular speed limit'
        return None

    def _clock_rejection(self, now_s: float) -> Optional[str]:
        """Reject discontinuous control time before rate limiting."""
        if self._last_time_s is None:
            return None
        elapsed_s = now_s - self._last_time_s
        if elapsed_s < 0.0:
            return 'control clock moved backwards'
        if elapsed_s > self.config.max_clock_step_s:
            return 'control clock step exceeded safety limit'
        return None

    def _gate_rejection(
        self, name: str, observed_s: Optional[float], now_s: float
    ) -> Optional[str]:
        """Require a recent, finite observation for a safety gate."""
        if not _finite(observed_s):
            return f'{name} state has not been observed'
        age_s = now_s - float(observed_s)
        if age_s < -self.config.future_tolerance_s:
            return f'{name} state timestamp is in the future'
        if age_s > self.config.safety_gate_timeout_s:
            return f'{name} state observation timed out'
        return None

    def _abort(self, mode: ArbiterMode, reason: str) -> ArbiterDecision:
        if not isinstance(mode, ArbiterMode):
            mode = ArbiterMode.DISABLED
        self._abort_latched = True
        self._linear_x_mps = 0.0
        self._angular_z_radps = 0.0
        return self._decision(ArbiterState.ABORTED, mode, reason)

    def _neutral(self, now_s: float) -> None:
        self._linear_x_mps = 0.0
        self._angular_z_radps = 0.0
        self._last_time_s = now_s

    def _decision(
        self,
        state: ArbiterState,
        mode: ArbiterMode,
        reason: str,
        input_age_s: Optional[float] = None,
    ) -> ArbiterDecision:
        return ArbiterDecision(
            state=state,
            mode=mode,
            linear_x_mps=self._linear_x_mps,
            angular_z_radps=self._angular_z_radps,
            reason=reason,
            input_age_s=input_age_s,
            abort_latched=self._abort_latched,
        )

    def _validate_config(self) -> None:
        positive = (
            self.config.command_timeout_s,
            self.config.safety_gate_timeout_s,
            self.config.max_clock_step_s,
            self.config.max_linear_speed_mps,
            self.config.max_angular_speed_radps,
            self.config.max_linear_acceleration_mps2,
            self.config.max_angular_acceleration_radps2,
        )
        if not all(_finite(value) and float(value) > 0.0 for value in positive):
            raise ValueError('arbiter limits must be finite and positive')
        if (
            not _finite(self.config.future_tolerance_s)
            or self.config.future_tolerance_s < 0.0
        ):
            raise ValueError('future_tolerance_s must be finite and non-negative')
        if (
            not isinstance(self.config.expected_source, str)
            or not self.config.expected_source
        ):
            raise ValueError('expected_source must be a non-empty string')
        if (
            not isinstance(self.config.expected_frame_id, str)
            or not self.config.expected_frame_id
        ):
            raise ValueError('expected_frame_id must be a non-empty string')
