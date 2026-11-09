#!/usr/bin/env python3
"""Command-line runner for the deterministic preview-v2 acceptance suite."""

import argparse
import dataclasses
import json
import sys
from typing import Optional, Sequence

from usv_control.simulation.follow_scenarios import ScenarioConfig
from usv_control.simulation.follow_suite import run_robust_acceptance_suite


def build_parser() -> argparse.ArgumentParser:
    """Build the argument parser without running a simulation."""
    parser = argparse.ArgumentParser(
        description='Run deterministic adversarial UUV-follow scenarios.',
    )
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--noise-stddev-m', type=float, default=0.02)
    parser.add_argument('--latency-s', type=float, default=0.10)
    parser.add_argument('--json', action='store_true', dest='as_json')
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    """Return zero for GO, two for NO-GO and one for invalid input."""
    arguments = build_parser().parse_args(argv)
    try:
        config = ScenarioConfig(
            random_seed=arguments.seed,
            measurement_noise_stddev_m=arguments.noise_stddev_m,
            measurement_latency_s=arguments.latency_s,
        )
        report = run_robust_acceptance_suite(config)
    except ValueError as error:
        print(f'error: {error}', file=sys.stderr)
        return 1

    if arguments.as_json:
        print(json.dumps(dataclasses.asdict(report), indent=2, sort_keys=True))
    else:
        _print_table(report)
    return 0 if report.go else 2


def _print_table(report) -> None:
    print('scenario          verdict  in_band   p95_m   max_m  reversals')
    for item in report.scenarios:
        metrics = item.metrics
        verdict = 'GO' if item.go else 'NO-GO'
        print(
            f'{item.name:17} {verdict:7} '
            f'{metrics.time_in_band_fraction:7.3f} '
            f'{metrics.radial_error_p95_m:7.3f} '
            f'{metrics.maximum_band_violation_m:7.3f} '
            f'{metrics.direction_reversals:9d}'
        )
        if item.failed_criteria:
            print('  failed: ' + ', '.join(item.failed_criteria))
    print('overall: ' + ('GO' if report.go else 'NO-GO'))


if __name__ == '__main__':
    raise SystemExit(main())
