#!/usr/bin/env python3
"""Command-line analysis of normalized USBL CSV recordings."""

import argparse
import dataclasses
import json
import sys
import typing

from seatrac_ros2.csv_analysis import AnalysisConfig, analyze_recording
from seatrac_ros2.csv_fixes import CsvFormatError, load_csv_fixes
from seatrac_ros2.fix_validation import ValidationConfig


def build_parser() -> argparse.ArgumentParser:
    """Build the command-line parser without side effects."""
    parser = argparse.ArgumentParser(
        description='Analyze normalized USBL fixes without ROS or hardware.',
    )
    parser.add_argument('csv_path', help='normalized USBL CSV recording')
    parser.add_argument('--expected-beacon-id', type=int)
    parser.add_argument('--gap-threshold-s', type=float, default=2.5)
    parser.add_argument('--minimum-accepted-fixes', type=int, default=3)
    parser.add_argument('--minimum-duration-s', type=float, default=60.0)
    parser.add_argument(
        '--maximum-rejected-fraction', type=float, default=0.05
    )
    parser.add_argument(
        '--stationary',
        action='store_true',
        help='evaluate horizontal dispersion around the median position',
    )
    parser.add_argument(
        '--maximum-stationary-p95-error-m',
        type=float,
        default=0.10,
    )
    parser.add_argument('--warn-fit-error', type=float, default=2.0)
    parser.add_argument('--reject-fit-error', type=float, default=3.0)
    parser.add_argument('--max-horizontal-speed-mps', type=float, default=3.0)
    parser.add_argument('--max-vertical-speed-mps', type=float, default=1.5)
    parser.add_argument('--json', action='store_true', dest='as_json')
    return parser


def main(argv: typing.Optional[typing.Sequence[str]] = None) -> int:
    """Analyze one file and return zero only for a GO verdict."""
    arguments = build_parser().parse_args(argv)
    try:
        samples = load_csv_fixes(arguments.csv_path)
        config = AnalysisConfig(
            validation=ValidationConfig(
                expected_beacon_id=arguments.expected_beacon_id,
                warn_fit_error=arguments.warn_fit_error,
                reject_fit_error=arguments.reject_fit_error,
                max_horizontal_speed_mps=arguments.max_horizontal_speed_mps,
                max_vertical_speed_mps=arguments.max_vertical_speed_mps,
            ),
            gap_threshold_s=arguments.gap_threshold_s,
            minimum_accepted_fixes=arguments.minimum_accepted_fixes,
            minimum_duration_s=arguments.minimum_duration_s,
            maximum_rejected_fraction=arguments.maximum_rejected_fraction,
            stationary=arguments.stationary,
            maximum_stationary_p95_error_m=(
                arguments.maximum_stationary_p95_error_m
            ),
        )
        analysis = analyze_recording(samples, config)
    except (CsvFormatError, OSError, ValueError) as error:
        print(f'error: {error}', file=sys.stderr)
        return 1

    if arguments.as_json:
        print(
            json.dumps(
                dataclasses.asdict(analysis), indent=2, sort_keys=True
            )
        )
    else:
        _print_human_report(analysis)
    return 0 if analysis.go else 2


def _print_human_report(analysis) -> None:
    verdict = 'GO' if analysis.go else 'NO-GO'
    print(f'verdict: {verdict}')
    print(
        'fixes: '
        f'{analysis.accepted_count} accepted, '
        f'{analysis.rejected_count} rejected, '
        f'{analysis.degraded_count} degraded'
    )
    print(f'effective_rate_hz: {_format_metric(analysis.effective_rate_hz)}')
    print(f'median_interval_s: {_format_metric(analysis.median_interval_s)}')
    print(f'p95_interval_s: {_format_metric(analysis.p95_interval_s)}')
    print(f'maximum_interval_s: {_format_metric(analysis.maximum_interval_s)}')
    print(f'gaps: {analysis.gap_count}')
    if analysis.stationary_p95_error_m is not None:
        print(
            'stationary_p95_error_m: '
            f'{analysis.stationary_p95_error_m:.6f}'
        )
    if analysis.rejected_reasons:
        reasons = ', '.join(
            f'{reason}={count}'
            for reason, count in analysis.rejected_reasons.items()
        )
        print(f'rejected_reasons: {reasons}')
    if analysis.failed_criteria:
        print('failed_criteria: ' + ', '.join(analysis.failed_criteria))


def _format_metric(value: typing.Optional[float]) -> str:
    return 'n/a' if value is None else f'{value:.6f}'


if __name__ == '__main__':
    raise SystemExit(main())
