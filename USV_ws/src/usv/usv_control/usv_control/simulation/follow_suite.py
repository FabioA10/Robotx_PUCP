"""Repeatable acceptance suite for the production preview-v2 controller."""

from dataclasses import dataclass
from typing import Dict, Optional, Tuple

from usv_control.core.uuv_follow_v2_model import RobustUuvFollowController
from usv_control.simulation.follow_scenarios import (
    ScenarioConfig,
    SimulationMetrics,
    default_scenarios,
    run_scenario,
)
from usv_control.simulation.robust_follow_adapter import (
    RobustFollowSimulationAdapter,
)


@dataclass(frozen=True)
class AcceptanceThresholds:
    """Explicit regression limits for one deterministic scenario."""

    minimum_time_in_band_fraction: float
    maximum_radial_error_p95_m: float
    maximum_band_violation_m: float
    maximum_direction_reversals: int
    maximum_loss_command_stop_s: Optional[float] = None
    maximum_loss_motion_stop_s: Optional[float] = None


@dataclass(frozen=True)
class ScenarioReport:
    """Metrics and failed criteria for one acceptance scenario."""

    name: str
    go: bool
    metrics: SimulationMetrics
    thresholds: AcceptanceThresholds
    failed_criteria: Tuple[str, ...]


@dataclass(frozen=True)
class SuiteReport:
    """Overall result returned by the offline acceptance suite."""

    go: bool
    scenarios: Tuple[ScenarioReport, ...]


DEFAULT_THRESHOLDS: Dict[str, AcceptanceThresholds] = {
    'stationary': AcceptanceThresholds(0.99, 0.01, 0.02, 0),
    'straight': AcceptanceThresholds(0.65, 0.15, 0.30, 1),
    'lateral': AcceptanceThresholds(0.85, 0.05, 0.15, 1),
    'zigzag': AcceptanceThresholds(0.55, 0.40, 0.50, 2),
    'cross_behind': AcceptanceThresholds(0.60, 0.35, 0.45, 3),
    'dropout_outlier': AcceptanceThresholds(
        0.70,
        0.55,
        0.65,
        1,
        maximum_loss_command_stop_s=0.60,
        maximum_loss_motion_stop_s=1.20,
    ),
}


def run_robust_acceptance_suite(
    config: ScenarioConfig = ScenarioConfig(),
) -> SuiteReport:
    """Run all standard scenarios and evaluate explicit thresholds."""
    reports = []
    for scenario in default_scenarios():
        thresholds = DEFAULT_THRESHOLDS[scenario.name]
        result = run_scenario(
            RobustFollowSimulationAdapter(RobustUuvFollowController()),
            scenario,
            config,
        )
        failures = _failed_criteria(result.metrics, thresholds)
        reports.append(
            ScenarioReport(
                name=scenario.name,
                go=not failures,
                metrics=result.metrics,
                thresholds=thresholds,
                failed_criteria=failures,
            )
        )
    return SuiteReport(
        go=all(report.go for report in reports),
        scenarios=tuple(reports),
    )


def _failed_criteria(
    metrics: SimulationMetrics,
    thresholds: AcceptanceThresholds,
) -> Tuple[str, ...]:
    failures = []
    if (
        metrics.time_in_band_fraction
        < thresholds.minimum_time_in_band_fraction
    ):
        failures.append('time_in_band_fraction')
    if metrics.radial_error_p95_m > thresholds.maximum_radial_error_p95_m:
        failures.append('radial_error_p95_m')
    if (
        metrics.maximum_band_violation_m
        > thresholds.maximum_band_violation_m
    ):
        failures.append('maximum_band_violation_m')
    if metrics.direction_reversals > thresholds.maximum_direction_reversals:
        failures.append('direction_reversals')
    if (
        thresholds.maximum_loss_command_stop_s is not None
        and (
            metrics.loss_command_stop_delay_s is None
            or metrics.loss_command_stop_delay_s
            > thresholds.maximum_loss_command_stop_s
        )
    ):
        failures.append('loss_command_stop_delay_s')
    if (
        thresholds.maximum_loss_motion_stop_s is not None
        and (
            metrics.loss_motion_stop_delay_s is None
            or metrics.loss_motion_stop_delay_s
            > thresholds.maximum_loss_motion_stop_s
        )
    ):
        failures.append('loss_motion_stop_delay_s')
    return tuple(failures)
