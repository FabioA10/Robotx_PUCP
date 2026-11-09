"""Deterministic adversarial simulator for relative-position following."""

from dataclasses import dataclass
import math
import random
from typing import Callable, Optional, Protocol, Sequence, Tuple


@dataclass(frozen=True)
class SimFix:
    """One noisy relative fix expressed in the simulated USV body frame."""

    x_m: float
    y_m: float
    timestamp_s: float
    variance_x_m2: float
    variance_y_m2: float
    covariance_xy_m2: float = 0.0


@dataclass(frozen=True)
class SimCommand:
    """Controller output before the simulated physical limits are applied."""

    linear_x_mps: float
    angular_z_radps: float
    state: str = 'unknown'


class TrackingController(Protocol):
    """Minimal interface implemented by a controller or test adapter."""

    def reset(self) -> None:
        """Reset all acquisition and rate-limiter state."""

    def step(
        self,
        fix: Optional[SimFix],
        now_s: float,
        dt_s: float,
    ) -> SimCommand:
        """Return one bounded candidate command."""


@dataclass(frozen=True)
class DropoutWindow:
    """Half-open interval during which no new acoustic fix is acquired."""

    start_s: float
    end_s: float

    def contains(self, timestamp_s: float) -> bool:
        """Return whether the timestamp lies inside the dropout."""
        return self.start_s <= timestamp_s < self.end_s


TargetPosition = Callable[[float], Tuple[float, float]]


@dataclass(frozen=True)
class Scenario:
    """Target trajectory and optional deterministic sensor faults."""

    name: str
    target_position: TargetPosition
    duration_s: float = 30.0
    dropouts: Tuple[DropoutWindow, ...] = ()
    outlier_times_s: Tuple[float, ...] = ()
    initial_usv_x_m: float = 0.0
    initial_usv_y_m: float = 0.0
    initial_usv_yaw_rad: float = 0.0


@dataclass(frozen=True)
class ScenarioConfig:
    """Simulation, sensor, actuator and acceptance parameters."""

    dt_s: float = 0.05
    measurement_period_s: float = 0.50
    measurement_latency_s: float = 0.10
    measurement_noise_stddev_m: float = 0.02
    outlier_offset_m: float = 4.0
    random_seed: int = 42
    radial_inner_m: float = 1.0
    radial_outer_m: float = 2.0
    warmup_s: float = 5.0
    metric_band_tolerance_m: float = 0.05
    linear_limit_mps: float = 0.70
    angular_limit_radps: float = 0.70
    linear_time_constant_s: float = 0.20
    angular_time_constant_s: float = 0.15
    neutral_linear_epsilon_mps: float = 0.01
    neutral_angular_epsilon_radps: float = 0.01


@dataclass(frozen=True)
class SimulationSample:
    """Truth, measurement and command values recorded at one control tick."""

    time_s: float
    radial_distance_m: float
    target_x_m: float
    target_y_m: float
    usv_x_m: float
    usv_y_m: float
    usv_yaw_rad: float
    requested_linear_mps: float
    requested_angular_radps: float
    applied_linear_mps: float
    applied_angular_radps: float
    actual_linear_mps: float
    actual_angular_radps: float
    fix_age_s: Optional[float]
    controller_state: str


@dataclass(frozen=True)
class SimulationMetrics:
    """Metrics used to compare controller changes and gate regressions."""

    samples: int
    time_in_band_fraction: float
    radial_error_p95_m: float
    radial_error_max_m: float
    maximum_band_violation_m: float
    direction_reversals: int
    linear_saturation_fraction: float
    angular_saturation_fraction: float
    maximum_linear_acceleration_mps2: float
    maximum_angular_acceleration_radps2: float
    maximum_linear_jerk_mps3: float
    loss_command_stop_delay_s: Optional[float]
    loss_motion_stop_delay_s: Optional[float]
    fixes_generated: int
    outliers_generated: int


@dataclass(frozen=True)
class SimulationResult:
    """Complete deterministic result for later inspection or reporting."""

    scenario_name: str
    samples: Tuple[SimulationSample, ...]
    metrics: SimulationMetrics


@dataclass
class _VehicleState:
    x_m: float = 0.0
    y_m: float = 0.0
    yaw_rad: float = 0.0
    linear_mps: float = 0.0
    angular_radps: float = 0.0


def run_scenario(
    controller: TrackingController,
    scenario: Scenario,
    config: ScenarioConfig = ScenarioConfig(),
) -> SimulationResult:
    """Run one scenario with a unicycle USV and delayed noisy fixes."""
    _validate(scenario, config)
    controller.reset()
    random_source = random.Random(config.random_seed)
    state = _VehicleState(
        x_m=scenario.initial_usv_x_m,
        y_m=scenario.initial_usv_y_m,
        yaw_rad=scenario.initial_usv_yaw_rad,
    )
    pending_fixes = []
    latest_fix = None
    next_fix_s = 0.0
    fixes_generated = 0
    outliers_generated = 0
    samples = []
    tick_count = int(math.ceil(scenario.duration_s / config.dt_s)) + 1

    for tick in range(tick_count):
        now_s = min(tick * config.dt_s, scenario.duration_s)
        target_x, target_y = scenario.target_position(now_s)

        while next_fix_s <= now_s + 1.0e-12:
            if not _in_dropout(next_fix_s, scenario.dropouts):
                fix, is_outlier = _make_fix(
                    state,
                    scenario,
                    config,
                    next_fix_s,
                    random_source,
                )
                pending_fixes.append(
                    (next_fix_s + config.measurement_latency_s, fix)
                )
                fixes_generated += 1
                outliers_generated += int(is_outlier)
            next_fix_s += config.measurement_period_s

        delivered = [item for item in pending_fixes if item[0] <= now_s]
        if delivered:
            latest_fix = delivered[-1][1]
            pending_fixes = [
                item for item in pending_fixes if item[0] > now_s
            ]

        command = controller.step(latest_fix, now_s, config.dt_s)
        requested_linear = _finite_or_zero(command.linear_x_mps)
        requested_angular = _finite_or_zero(command.angular_z_radps)
        applied_linear = _clamp(
            requested_linear,
            -config.linear_limit_mps,
            config.linear_limit_mps,
        )
        applied_angular = _clamp(
            requested_angular,
            -config.angular_limit_radps,
            config.angular_limit_radps,
        )
        state.linear_mps = _first_order(
            state.linear_mps,
            applied_linear,
            config.dt_s,
            config.linear_time_constant_s,
        )
        state.angular_radps = _first_order(
            state.angular_radps,
            applied_angular,
            config.dt_s,
            config.angular_time_constant_s,
        )
        state.yaw_rad = _wrap(
            state.yaw_rad + state.angular_radps * config.dt_s
        )
        state.x_m += (
            state.linear_mps * math.cos(state.yaw_rad) * config.dt_s
        )
        state.y_m += (
            state.linear_mps * math.sin(state.yaw_rad) * config.dt_s
        )

        delta_x = target_x - state.x_m
        delta_y = target_y - state.y_m
        samples.append(
            SimulationSample(
                time_s=now_s,
                radial_distance_m=math.hypot(delta_x, delta_y),
                target_x_m=target_x,
                target_y_m=target_y,
                usv_x_m=state.x_m,
                usv_y_m=state.y_m,
                usv_yaw_rad=state.yaw_rad,
                requested_linear_mps=requested_linear,
                requested_angular_radps=requested_angular,
                applied_linear_mps=applied_linear,
                applied_angular_radps=applied_angular,
                actual_linear_mps=state.linear_mps,
                actual_angular_radps=state.angular_radps,
                fix_age_s=(
                    None if latest_fix is None
                    else now_s - latest_fix.timestamp_s
                ),
                controller_state=command.state,
            )
        )

    metrics = _calculate_metrics(
        samples,
        scenario,
        config,
        fixes_generated,
        outliers_generated,
    )
    return SimulationResult(scenario.name, tuple(samples), metrics)


def default_scenarios() -> Tuple[Scenario, ...]:
    """Return repeatable trajectories covering common relative motions."""
    return (
        Scenario('stationary', lambda _time: (3.0, 0.0)),
        Scenario('straight', lambda time: (3.0 + 0.20 * time, 0.0)),
        Scenario('lateral', lambda time: (3.0, -2.0 + 0.15 * time)),
        Scenario(
            'zigzag',
            lambda time: (
                3.0 + 0.12 * time,
                1.2 * math.sin(0.45 * time),
            ),
        ),
        Scenario(
            'cross_behind',
            lambda time: (4.0 - 0.30 * time, 1.5 - 0.10 * time),
        ),
        Scenario(
            'dropout_outlier',
            lambda time: (3.0 + 0.10 * time, 0.8 * math.sin(0.3 * time)),
            dropouts=(DropoutWindow(8.0, 12.0),),
            outlier_times_s=(5.0, 18.0),
        ),
    )


def _make_fix(
    state: _VehicleState,
    scenario: Scenario,
    config: ScenarioConfig,
    timestamp_s: float,
    random_source: random.Random,
) -> Tuple[SimFix, bool]:
    target_x, target_y = scenario.target_position(timestamp_s)
    delta_x = target_x - state.x_m
    delta_y = target_y - state.y_m
    cosine = math.cos(state.yaw_rad)
    sine = math.sin(state.yaw_rad)
    body_x = cosine * delta_x + sine * delta_y
    body_y = -sine * delta_x + cosine * delta_y
    is_outlier = any(
        abs(timestamp_s - outlier_s)
        <= 0.5 * config.measurement_period_s
        for outlier_s in scenario.outlier_times_s
    )
    outlier_x = config.outlier_offset_m if is_outlier else 0.0
    outlier_y = -0.5 * config.outlier_offset_m if is_outlier else 0.0
    noise = config.measurement_noise_stddev_m
    variance = max(noise, 1.0e-6) ** 2
    return (
        SimFix(
            x_m=body_x + random_source.gauss(0.0, noise) + outlier_x,
            y_m=body_y + random_source.gauss(0.0, noise) + outlier_y,
            timestamp_s=timestamp_s,
            variance_x_m2=variance,
            variance_y_m2=variance,
        ),
        is_outlier,
    )


def _calculate_metrics(
    samples: Sequence[SimulationSample],
    scenario: Scenario,
    config: ScenarioConfig,
    fixes_generated: int,
    outliers_generated: int,
) -> SimulationMetrics:
    evaluated = [sample for sample in samples if sample.time_s >= config.warmup_s]
    if not evaluated:
        evaluated = list(samples)
    raw_band_errors = [
        _band_error(sample.radial_distance_m, config) for sample in evaluated
    ]
    band_errors = [
        max(0.0, error - config.metric_band_tolerance_m)
        for error in raw_band_errors
    ]
    in_band = sum(error == 0.0 for error in band_errors)
    linear_saturated = sum(
        abs(sample.requested_linear_mps) >= config.linear_limit_mps - 1.0e-9
        for sample in evaluated
    )
    angular_saturated = sum(
        abs(sample.requested_angular_radps) >= config.angular_limit_radps - 1.0e-9
        for sample in evaluated
    )
    linear_accelerations = _differences(
        [sample.applied_linear_mps for sample in samples], config.dt_s
    )
    angular_accelerations = _differences(
        [sample.applied_angular_radps for sample in samples], config.dt_s
    )
    linear_jerks = _differences(linear_accelerations, config.dt_s)

    return SimulationMetrics(
        samples=len(samples),
        time_in_band_fraction=in_band / len(evaluated),
        radial_error_p95_m=_nearest_rank(band_errors, 0.95),
        radial_error_max_m=max(band_errors, default=0.0),
        maximum_band_violation_m=max(raw_band_errors, default=0.0),
        direction_reversals=_count_reversals(
            [sample.applied_linear_mps for sample in samples],
            config.neutral_linear_epsilon_mps,
        ),
        linear_saturation_fraction=linear_saturated / len(evaluated),
        angular_saturation_fraction=angular_saturated / len(evaluated),
        maximum_linear_acceleration_mps2=max(
            (abs(value) for value in linear_accelerations), default=0.0
        ),
        maximum_angular_acceleration_radps2=max(
            (abs(value) for value in angular_accelerations), default=0.0
        ),
        maximum_linear_jerk_mps3=max(
            (abs(value) for value in linear_jerks), default=0.0
        ),
        loss_command_stop_delay_s=_loss_stop_delay(
            samples, scenario.dropouts, config, actual=False
        ),
        loss_motion_stop_delay_s=_loss_stop_delay(
            samples, scenario.dropouts, config, actual=True
        ),
        fixes_generated=fixes_generated,
        outliers_generated=outliers_generated,
    )


def _loss_stop_delay(
    samples: Sequence[SimulationSample],
    dropouts: Sequence[DropoutWindow],
    config: ScenarioConfig,
    *,
    actual: bool,
) -> Optional[float]:
    if not dropouts:
        return None
    delays = []
    for window in dropouts:
        window_samples = [
            sample
            for sample in samples
            if window.start_s <= sample.time_s < window.end_s
        ]
        stopped_at = None
        for index, sample in enumerate(window_samples):
            if _sample_is_neutral(sample, config, actual=actual) and all(
                _sample_is_neutral(item, config, actual=actual)
                for item in window_samples[index:]
            ):
                stopped_at = sample.time_s
                break
        if stopped_at is None:
            delays.append(samples[-1].time_s - window.start_s)
        else:
            delays.append(stopped_at - window.start_s)
    return max(delays, default=None)


def _validate(scenario: Scenario, config: ScenarioConfig) -> None:
    positive = (
        scenario.duration_s,
        config.dt_s,
        config.measurement_period_s,
        config.linear_limit_mps,
        config.angular_limit_radps,
    )
    if not all(math.isfinite(value) and value > 0.0 for value in positive):
        raise ValueError('duration, periods and physical limits must be positive')
    if not 0.0 <= config.radial_inner_m < config.radial_outer_m:
        raise ValueError('radial band must be ordered and non-negative')
    nonnegative = (
        config.measurement_latency_s,
        config.measurement_noise_stddev_m,
        config.warmup_s,
        config.metric_band_tolerance_m,
        config.linear_time_constant_s,
        config.angular_time_constant_s,
    )
    if not all(math.isfinite(value) and value >= 0.0 for value in nonnegative):
        raise ValueError('latency, noise, warmup and time constants must be non-negative')
    for window in scenario.dropouts:
        if window.start_s < 0.0 or window.end_s <= window.start_s:
            raise ValueError('dropout windows must be ordered and non-negative')


def _in_dropout(timestamp_s: float, windows: Sequence[DropoutWindow]) -> bool:
    return any(window.contains(timestamp_s) for window in windows)


def _sample_is_neutral(
    sample: SimulationSample,
    config: ScenarioConfig,
    *,
    actual: bool,
) -> bool:
    linear = sample.actual_linear_mps if actual else sample.applied_linear_mps
    angular = (
        sample.actual_angular_radps
        if actual else sample.applied_angular_radps
    )
    return (
        abs(linear) <= config.neutral_linear_epsilon_mps
        and abs(angular) <= config.neutral_angular_epsilon_radps
    )


def _band_error(radius_m: float, config: ScenarioConfig) -> float:
    if radius_m < config.radial_inner_m:
        return config.radial_inner_m - radius_m
    if radius_m > config.radial_outer_m:
        return radius_m - config.radial_outer_m
    return 0.0


def _first_order(current: float, target: float, dt_s: float, tau_s: float) -> float:
    if tau_s <= 0.0:
        return target
    return current + min(1.0, dt_s / tau_s) * (target - current)


def _differences(values: Sequence[float], dt_s: float) -> Sequence[float]:
    return [
        (right - left) / dt_s
        for left, right in zip(values, values[1:])
    ]


def _count_reversals(values: Sequence[float], epsilon: float) -> int:
    previous_sign = 0
    reversals = 0
    for value in values:
        sign = 1 if value > epsilon else -1 if value < -epsilon else 0
        if sign == 0:
            continue
        if previous_sign and sign != previous_sign:
            reversals += 1
        previous_sign = sign
    return reversals


def _nearest_rank(values: Sequence[float], quantile: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    index = max(0, math.ceil(quantile * len(ordered)) - 1)
    return ordered[index]


def _finite_or_zero(value: float) -> float:
    return float(value) if math.isfinite(float(value)) else 0.0


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))
