"""Tests for the repeatable preview-v2 acceptance suite and CLI."""

import json

from usv_control.simulation.follow_scenarios import ScenarioConfig
from usv_control.simulation.follow_suite import run_robust_acceptance_suite
from usv_control.simulation.follow_suite_cli import main


def test_default_robust_suite_is_go_with_explicit_performance_gates():
    """Require actual tracking performance, not merely finite output."""
    report = run_robust_acceptance_suite()
    assert report.go
    assert all(item.go for item in report.scenarios)


def test_high_noise_suite_is_no_go():
    """Reject conditions outside the preview controller uncertainty gate."""
    report = run_robust_acceptance_suite(
        ScenarioConfig(measurement_noise_stddev_m=0.50)
    )
    assert not report.go
    assert any(not item.go for item in report.scenarios)


def test_cli_emits_json_and_expected_exit_codes(capsys):
    """Make GO and NO-GO automation-friendly for repeated pre-site runs."""
    assert main(['--json']) == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload['go'] is True
    assert len(payload['scenarios']) == 6

    assert main(['--noise-stddev-m', '0.50']) == 2
    assert 'overall: NO-GO' in capsys.readouterr().out


def test_cli_rejects_invalid_configuration(capsys):
    """Return an input error instead of a misleading scenario verdict."""
    assert main(['--latency-s', '-1.0']) == 1
    assert 'error:' in capsys.readouterr().err
