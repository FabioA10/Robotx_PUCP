"""Tests for deterministic adversarial follow simulation."""

import math

import pytest

from usv_control.core.uuv_follow_v2_model import (
    RobustFollowConfig,
    RobustUuvFollowController,
)
from usv_control.simulation import (
    default_scenarios,
    DropoutWindow,
    RobustFollowSimulationAdapter,
    run_scenario,
    Scenario,
    ScenarioConfig,
    SimCommand,
)


class NeutralController:
    """Always request neutral, useful for exact metric checks."""

    def reset(self):
        """Reset without state."""

    def step(self, _fix, _now_s, _dt_s):
        """Return a neutral candidate command."""
        return SimCommand(0.0, 0.0, 'neutral')


class ProportionalBandController:
    """Small reference controller used only to exercise the harness."""

    def __init__(self, timeout_s=0.6):
        self.timeout_s = timeout_s

    def reset(self):
        """Reset without retained state."""

    def step(self, fix, now_s, _dt_s):
        """Approach the target and stop when the fix becomes stale."""
        if fix is None or now_s - fix.timestamp_s > self.timeout_s:
            return SimCommand(0.0, 0.0, 'lost')
        radius = math.hypot(fix.x_m, fix.y_m)
        bearing = math.atan2(fix.y_m, fix.x_m)
        radial_error = max(0.0, radius - 1.5)
        return SimCommand(
            min(0.6, 0.7 * radial_error) * max(0.0, math.cos(bearing)),
            max(-0.6, min(0.6, bearing)),
            'tracking',
        )


class AlternatingController:
    """Alternate direction to validate reversal and saturation metrics."""

    def __init__(self):
        self.tick = 0

    def reset(self):
        """Reset the deterministic tick counter."""
        self.tick = 0

    def step(self, _fix, _now_s, _dt_s):
        """Switch direction every two calls and request saturation."""
        sign = 1.0 if (self.tick // 2) % 2 == 0 else -1.0
        self.tick += 1
        return SimCommand(sign * 2.0, 2.0, 'alternating')


class RecordingController(NeutralController):
    """Record delivered fixes to verify frame and latency semantics."""

    def __init__(self):
        """Create an empty delivery record."""
        self.deliveries = []

    def reset(self):
        """Clear prior deliveries."""
        self.deliveries = []

    def step(self, fix, now_s, _dt_s):
        """Record each newly observed timestamp and its delivery time."""
        if fix is not None and (
            not self.deliveries
            or fix.timestamp_s != self.deliveries[-1][1].timestamp_s
        ):
            self.deliveries.append((now_s, fix))
        return SimCommand(0.0, 0.0, 'recording')


def test_stationary_inside_band_has_exact_metrics():
    """Keep perfect truth in-band when neither vehicle moves."""
    scenario = Scenario('in_band', lambda _time: (1.5, 0.0), duration_s=2.0)
    config = ScenarioConfig(
        measurement_noise_stddev_m=0.0,
        measurement_latency_s=0.0,
        warmup_s=0.0,
    )
    result = run_scenario(NeutralController(), scenario, config)
    assert result.metrics.time_in_band_fraction == 1.0
    assert result.metrics.radial_error_p95_m == 0.0
    assert result.metrics.maximum_band_violation_m == 0.0
    assert result.metrics.direction_reversals == 0


def test_simulation_is_deterministic_for_noise_and_outliers():
    """Produce byte-for-byte equivalent records for the same seed."""
    scenario = Scenario(
        'faults',
        lambda time: (3.0 + 0.1 * time, 0.2),
        duration_s=3.0,
        outlier_times_s=(1.0,),
    )
    first = run_scenario(ProportionalBandController(), scenario)
    second = run_scenario(ProportionalBandController(), scenario)
    assert first == second
    assert first.metrics.outliers_generated == 1


def test_world_to_body_rotation_and_source_timestamp_are_preserved():
    """Rotate with R(-yaw) and never replace acquisition time by delivery."""
    controller = RecordingController()
    scenario = Scenario(
        'frame_latency',
        lambda _time: (0.0, 2.0),
        duration_s=0.5,
        initial_usv_yaw_rad=math.pi / 2.0,
    )
    config = ScenarioConfig(
        dt_s=0.05,
        measurement_period_s=0.5,
        measurement_latency_s=0.2,
        measurement_noise_stddev_m=0.0,
        warmup_s=0.0,
    )
    run_scenario(controller, scenario, config)
    delivered_at, fix = controller.deliveries[0]
    assert delivered_at == pytest.approx(0.2)
    assert fix.timestamp_s == pytest.approx(0.0)
    assert fix.x_m == pytest.approx(2.0)
    assert fix.y_m == pytest.approx(0.0, abs=1.0e-12)


def test_dropout_records_command_and_vehicle_stop_delays():
    """Measure both controller timeout and simulated hull coast-down."""
    scenario = Scenario(
        'dropout',
        lambda _time: (5.0, 0.0),
        duration_s=5.0,
        dropouts=(DropoutWindow(1.0, 4.0),),
    )
    config = ScenarioConfig(
        measurement_latency_s=0.0,
        measurement_noise_stddev_m=0.0,
        warmup_s=0.0,
    )
    result = run_scenario(ProportionalBandController(timeout_s=0.6), scenario, config)
    assert 0.05 <= result.metrics.loss_command_stop_delay_s <= 0.20
    assert result.metrics.loss_motion_stop_delay_s > result.metrics.loss_command_stop_delay_s
    assert result.metrics.loss_motion_stop_delay_s < 1.5


def test_reversal_and_saturation_metrics_detect_chatter():
    """Expose unsafe alternating commands instead of hiding them in dynamics."""
    scenario = Scenario('chatter', lambda _time: (3.0, 0.0), duration_s=1.0)
    config = ScenarioConfig(
        dt_s=0.1,
        warmup_s=0.0,
        measurement_noise_stddev_m=0.0,
    )
    result = run_scenario(AlternatingController(), scenario, config)
    assert result.metrics.direction_reversals >= 4
    assert result.metrics.linear_saturation_fraction == 1.0
    assert result.metrics.angular_saturation_fraction == 1.0
    assert result.metrics.maximum_linear_acceleration_mps2 > 0.0
    assert result.metrics.maximum_linear_jerk_mps3 > 0.0


def test_default_adversarial_suite_runs_with_finite_metrics():
    """Exercise stationary, moving, crossing, noisy and dropout trajectories."""
    for scenario in default_scenarios():
        result = run_scenario(ProportionalBandController(), scenario)
        metrics = result.metrics
        assert metrics.samples > 0
        assert 0.0 <= metrics.time_in_band_fraction <= 1.0
        assert math.isfinite(metrics.radial_error_p95_m)
        assert math.isfinite(metrics.radial_error_max_m)


def test_robust_v2_core_runs_against_all_adversarial_scenarios():
    """Integrate the production preview-v2 core with every simulator fault."""
    simulation_config = ScenarioConfig(
        radial_inner_m=1.0,
        radial_outer_m=2.0,
        linear_limit_mps=0.70,
        angular_limit_radps=0.70,
    )
    for scenario in default_scenarios():
        adapter = RobustFollowSimulationAdapter(
            RobustUuvFollowController(RobustFollowConfig())
        )
        result = run_scenario(adapter, scenario, simulation_config)
        metrics = result.metrics
        assert metrics.samples > 0
        assert metrics.linear_saturation_fraction == 0.0
        assert metrics.angular_saturation_fraction == 0.0
        assert metrics.direction_reversals <= 8
        if scenario.dropouts:
            assert metrics.loss_command_stop_delay_s is not None
            assert metrics.loss_command_stop_delay_s <= 1.1


@pytest.mark.parametrize(
    'scenario, config',
    [
        (Scenario('bad', lambda _time: (0.0, 0.0), duration_s=0.0), ScenarioConfig()),
        (
            Scenario('bad_band', lambda _time: (0.0, 0.0)),
            ScenarioConfig(radial_inner_m=2.0, radial_outer_m=1.0),
        ),
        (
            Scenario(
                'bad_dropout',
                lambda _time: (0.0, 0.0),
                dropouts=(DropoutWindow(2.0, 1.0),),
            ),
            ScenarioConfig(),
        ),
    ],
)
def test_invalid_scenario_configuration_is_rejected(scenario, config):
    """Reject setups which would produce misleading metrics."""
    with pytest.raises(ValueError):
        run_scenario(NeutralController(), scenario, config)
