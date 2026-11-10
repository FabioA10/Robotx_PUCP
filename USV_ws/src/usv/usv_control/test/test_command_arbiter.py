"""Unit tests for the actuator-independent shadow command arbiter."""

import math

import pytest

from usv_control.core import (
    ArbiterConfig,
    ArbiterMode,
    ArbiterState,
    CandidateCommand,
    ShadowCommandArbiter,
)


def command(timestamp=10.0, linear=0.2, angular=0.0, source='uuv_follow'):
    """Build a valid candidate with concise test defaults."""
    return CandidateCommand(linear, angular, timestamp, source)


def enabled(model, candidate, now=10.0, **overrides):
    """Evaluate a candidate through the enabled follow path."""
    values = {
        'requested_mode': ArbiterMode.UUV_FOLLOW,
        'operator_enabled': True,
        'vehicle_connected': True,
        'kill_active': False,
        'vehicle_state_timestamp_s': now,
        'kill_state_timestamp_s': now,
    }
    values.update(overrides)
    return model.evaluate(candidate, now_s=now, **values)


def arm_mode(model, now=9.0):
    """Consume the mandatory neutral mode-change cycle."""
    result = enabled(model, None, now=now)
    assert result.state == ArbiterState.BLOCKED


def test_disabled_is_neutral_and_clears_abort_latch():
    """Disabling always clears output and the explicit abort latch."""
    model = ShadowCommandArbiter()
    result = model.evaluate(
        command(), 10.0, ArbiterMode.DISABLED, False, False, True
    )
    assert result.state == ArbiterState.DISABLED
    assert result.linear_x_mps == result.angular_z_radps == 0.0
    assert result.abort_latched is False


def test_mode_change_requires_one_neutral_cycle():
    """Changing mode cannot emit a command in the same cycle."""
    model = ShadowCommandArbiter()
    first = enabled(model, command(), now=10.0)
    second = enabled(model, command(timestamp=10.1), now=10.1)
    assert first.state == ArbiterState.BLOCKED
    assert first.linear_x_mps == 0.0
    assert second.state == ArbiterState.ACTIVE


def test_acceleration_and_speed_are_bounded():
    """Slew limits bound the first accepted transition from neutral."""
    model = ShadowCommandArbiter(ArbiterConfig(
        max_linear_acceleration_mps2=0.1,
        max_angular_acceleration_radps2=0.2,
    ))
    arm_mode(model)
    result = enabled(
        model,
        command(timestamp=9.1, linear=0.2, angular=0.3),
        now=9.1,
    )
    assert result.linear_x_mps == pytest.approx(0.01)
    assert result.angular_z_radps == pytest.approx(0.02)


@pytest.mark.parametrize(
    'candidate,now,reason',
    [
        (command(timestamp=9.0), 10.0, 'timed out'),
        (command(timestamp=11.0), 10.0, 'future'),
        (command(linear=0.21), 10.0, 'linear speed'),
        (command(angular=0.31), 10.0, 'angular speed'),
        (command(linear=math.nan), 10.0, 'non-finite'),
        (command(source='mission'), 10.0, 'source'),
    ],
)
def test_invalid_candidate_latches_zero(candidate, now, reason):
    """Malformed, stale or excessive input latches a neutral abort."""
    model = ShadowCommandArbiter()
    arm_mode(model, now=9.5)
    result = enabled(model, candidate, now=now)
    assert result.state == ArbiterState.ABORTED
    assert reason in result.reason
    assert result.linear_x_mps == result.angular_z_radps == 0.0
    assert result.abort_latched is True


def test_wrong_candidate_frame_latches_abort():
    """Commands expressed outside the configured body frame are rejected."""
    model = ShadowCommandArbiter()
    arm_mode(model, now=9.9)
    candidate = CandidateCommand(0.1, 0.0, 10.0, frame_id='map')
    result = enabled(model, candidate, now=10.0)
    assert result.state == ArbiterState.ABORTED
    assert 'frame' in result.reason


def test_missing_candidate_blocks_without_latching_during_startup():
    """Startup can wait safely for the first candidate without aborting."""
    model = ShadowCommandArbiter()
    arm_mode(model, now=9.5)
    result = enabled(model, None, now=10.0)
    assert result.state == ArbiterState.BLOCKED
    assert result.linear_x_mps == result.angular_z_radps == 0.0
    assert result.abort_latched is False


def test_abort_needs_disable_before_reenable():
    """A safety abort cannot recover merely because input becomes valid."""
    model = ShadowCommandArbiter()
    arm_mode(model, now=9.0)
    enabled(model, command(timestamp=9.0), now=10.0)
    latched = enabled(model, command(timestamp=10.1), now=10.1)
    assert latched.state == ArbiterState.ABORTED

    disabled = model.evaluate(
        None, 10.2, ArbiterMode.DISABLED, False, True, False
    )
    assert disabled.state == ArbiterState.DISABLED
    assert disabled.abort_latched is False
    reacquiring = enabled(model, command(timestamp=10.3), now=10.3)
    assert reacquiring.state == ArbiterState.BLOCKED
    active = enabled(model, command(timestamp=10.4), now=10.4)
    assert active.state == ArbiterState.ACTIVE


@pytest.mark.parametrize(
    'connected,kill,reason',
    [
        (False, False, 'not connected'),
        (True, True, 'killswitch'),
    ],
)
def test_vehicle_safety_faults_latch_zero(connected, kill, reason):
    """Connection and kill gates fail closed and latch."""
    model = ShadowCommandArbiter()
    result = enabled(
        model,
        command(),
        vehicle_connected=connected,
        kill_active=kill,
    )
    assert result.state == ArbiterState.ABORTED
    assert reason in result.reason
    assert result.linear_x_mps == result.angular_z_radps == 0.0


def test_reconnect_does_not_automatically_clear_abort():
    """Restored telemetry cannot re-enable a previously aborted command."""
    model = ShadowCommandArbiter()
    arm_mode(model, now=9.0)
    enabled(model, command(timestamp=9.1), now=9.1)
    lost = enabled(
        model,
        command(timestamp=9.2),
        now=9.2,
        vehicle_connected=False,
    )
    restored = enabled(model, command(timestamp=9.3), now=9.3)
    assert lost.state == ArbiterState.ABORTED
    assert restored.state == ArbiterState.ABORTED
    assert restored.linear_x_mps == restored.angular_z_radps == 0.0


@pytest.mark.parametrize(
    'overrides,reason',
    [
        ({'vehicle_state_timestamp_s': None}, 'has not been observed'),
        ({'kill_state_timestamp_s': None}, 'has not been observed'),
        ({'vehicle_state_timestamp_s': 9.0}, 'timed out'),
        ({'kill_state_timestamp_s': 9.0}, 'timed out'),
    ],
)
def test_safety_gates_require_fresh_observations(overrides, reason):
    """A remembered Boolean cannot outlive its safety observation timeout."""
    model = ShadowCommandArbiter()
    result = enabled(model, command(), now=10.0, **overrides)
    assert result.state == ArbiterState.ABORTED
    assert reason in result.reason
    assert result.abort_latched is True


@pytest.mark.parametrize(
    'bad_time,reason',
    [
        (9.0, 'moved backwards'),
        (11.2, 'step exceeded'),
    ],
)
def test_clock_discontinuity_latches_abort(bad_time, reason):
    """Backward and excessive time jumps cannot bypass slew limits."""
    model = ShadowCommandArbiter()
    arm_mode(model, now=10.0)
    result = enabled(
        model,
        command(timestamp=bad_time),
        now=bad_time,
    )
    assert result.state == ArbiterState.ABORTED
    assert reason in result.reason
    assert result.linear_x_mps == result.angular_z_radps == 0.0


def test_invalid_mode_latches_and_cannot_implicitly_reset():
    """A typo in requested_mode is a fault, not an implicit disable/reset."""
    model = ShadowCommandArbiter()
    result = model.evaluate(
        command(),
        10.0,
        'uuv_folow',
        True,
        True,
        False,
        10.0,
        10.0,
    )
    assert result.state == ArbiterState.ABORTED
    assert result.abort_latched is True
    assert 'invalid' in result.reason

    still_latched = enabled(model, command(timestamp=10.1), now=10.1)
    assert still_latched.state == ArbiterState.ABORTED

    reset = model.evaluate(
        None, 10.2, ArbiterMode.UUV_FOLLOW, False, False, True
    )
    assert reset.state == ArbiterState.DISABLED
    assert reset.abort_latched is False


def test_manual_and_mission_have_no_implicit_shadow_input():
    """Unsupported sources remain neutral even with a follower candidate."""
    for mode in (ArbiterMode.MANUAL, ArbiterMode.MISSION):
        model = ShadowCommandArbiter()
        enabled(model, command(), requested_mode=mode)
        result = enabled(
            model,
            command(timestamp=10.1),
            now=10.1,
            requested_mode=mode,
        )
        assert result.state == ArbiterState.BLOCKED
        assert result.linear_x_mps == result.angular_z_radps == 0.0


def test_invalid_configuration_is_rejected():
    """Unsafe or ambiguous limits are rejected during construction."""
    for config in (
        ArbiterConfig(command_timeout_s=0.0),
        ArbiterConfig(max_linear_speed_mps=math.nan),
        ArbiterConfig(future_tolerance_s=-1.0),
        ArbiterConfig(safety_gate_timeout_s=0.0),
        ArbiterConfig(max_clock_step_s=math.inf),
        ArbiterConfig(expected_source=''),
        ArbiterConfig(expected_frame_id=''),
    ):
        with pytest.raises(ValueError):
            ShadowCommandArbiter(config)
