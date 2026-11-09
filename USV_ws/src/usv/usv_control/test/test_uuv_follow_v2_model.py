"""Unit tests for the robust radial-band UUV follower."""

import math

import pytest

from usv_control.core.uuv_follow_model import RelativeFix
from usv_control.core.uuv_follow_v2_model import (
    DriveDirection,
    RobustFollowConfig,
    RobustFollowState,
    RobustUuvFollowController,
)


def controller(**changes):
    """Create a controller which acquires immediately unless overridden."""
    changes.setdefault('acquisition_fixes', 1)
    return RobustUuvFollowController(RobustFollowConfig(**changes))


def fix(x=3.0, y=0.0, timestamp=1.0, **changes):
    """Create a valid relative fix with conservative covariance."""
    values = {
        'variance_x_m2': 0.04,
        'variance_y_m2': 0.04,
        'covariance_xy_m2': 0.0,
    }
    values.update(changes)
    return RelativeFix(x, y, timestamp, **values)


def test_disabled_and_missing_fix_are_exactly_zero():
    """Never retain a command while disabled or without a target."""
    model = controller()
    disabled = model.evaluate(fix(), 1.0, False)
    missing = model.evaluate(None, 1.1, True)
    assert disabled.state == RobustFollowState.DISABLED
    assert missing.state == RobustFollowState.LOST_TARGET
    assert disabled.linear_x_mps == disabled.angular_z_radps == 0.0
    assert missing.linear_x_mps == missing.angular_z_radps == 0.0


def test_distinct_fixes_are_required_for_acquisition():
    """A control-loop repeat must not masquerade as a new acoustic fix."""
    model = controller(acquisition_fixes=2)
    first = model.evaluate(fix(timestamp=1.0), 1.0, True)
    repeated = model.evaluate(fix(timestamp=1.0), 1.1, True)
    second = model.evaluate(fix(timestamp=1.2), 1.2, True)
    assert first.state == RobustFollowState.ACQUIRING
    assert repeated.state == RobustFollowState.ACQUIRING
    assert second.state == RobustFollowState.APPROACHING


def test_far_target_ahead_produces_bounded_forward_command():
    """Approach a distant target with a ramped forward command."""
    model = controller()
    result = model.evaluate(fix(x=3.0), 1.0, True)
    assert result.state == RobustFollowState.APPROACHING
    assert 0.0 < result.linear_x_mps <= 0.60
    assert result.angular_z_radps == 0.0
    assert result.direction == DriveDirection.FORWARD


def test_far_target_behind_uses_reverse_without_full_turn():
    """Select the stern when it is clearly the shorter orientation."""
    model = controller()
    result = model.evaluate(fix(x=-3.0), 1.0, True)
    assert result.state == RobustFollowState.APPROACHING
    assert result.linear_x_mps < 0.0
    assert result.angular_z_radps == pytest.approx(0.0)
    assert result.direction == DriveDirection.REVERSE


def test_close_target_ahead_causes_controlled_retreat():
    """Back away when the target violates the inner radial boundary."""
    result = controller().evaluate(fix(x=0.6), 1.0, True)
    assert result.state == RobustFollowState.RETREATING
    assert result.linear_x_mps < 0.0
    assert result.direction == DriveDirection.REVERSE


def test_close_target_behind_moves_forward_to_separate():
    """Move the bow away when a close target is behind the hull."""
    result = controller().evaluate(fix(x=-0.6), 1.0, True)
    assert result.state == RobustFollowState.RETREATING
    assert result.linear_x_mps > 0.0
    assert result.direction == DriveDirection.FORWARD


def test_lateral_target_aligns_before_translating():
    """Suppress surge when the desired motion is nearly lateral."""
    result = controller().evaluate(fix(x=0.0, y=3.0), 1.0, True)
    assert result.state == RobustFollowState.ALIGNING
    assert result.linear_x_mps == 0.0
    assert result.angular_z_radps > 0.0


def test_target_in_band_holds_without_chasing_noise():
    """Produce neutral commands throughout the configured holding band."""
    result = controller().evaluate(fix(x=1.5), 1.0, True)
    assert result.state == RobustFollowState.HOLDING_BAND
    assert result.linear_x_mps == result.angular_z_radps == 0.0


def test_radial_hysteresis_prevents_boundary_chatter():
    """Keep approaching until reaching a margin inside the outer radius."""
    model = controller(max_relative_speed_mps=10.0)
    first = model.evaluate(fix(x=2.20, timestamp=1.0), 1.0, True)
    near = model.evaluate(fix(x=2.10, timestamp=1.1), 1.1, True)
    inside = model.evaluate(fix(x=1.84, timestamp=1.2), 1.2, True)
    assert first.state == RobustFollowState.APPROACHING
    assert near.state == RobustFollowState.APPROACHING
    assert inside.state == RobustFollowState.HOLDING_BAND


def test_direction_hysteresis_avoids_forward_reverse_chatter():
    """Retain stern selection through the ambiguous lateral sector."""
    model = controller(max_relative_speed_mps=100.0)

    def at_angle(angle_deg, timestamp):
        angle = math.radians(angle_deg)
        return model.evaluate(
            fix(3.0 * math.cos(angle), 3.0 * math.sin(angle), timestamp),
            timestamp,
            True,
        )

    assert at_angle(110.0, 1.0).direction == DriveDirection.REVERSE
    assert at_angle(90.0, 2.0).direction == DriveDirection.REVERSE
    assert at_angle(70.0, 3.0).direction == DriveDirection.FORWARD


def test_linear_acceleration_is_limited_even_after_long_scheduler_gap():
    """Clamp both acceleration and the control interval used by the ramp."""
    model = controller(
        max_linear_accel_mps2=0.20,
        initial_control_dt_s=0.10,
        max_control_dt_s=0.25,
    )
    first = model.evaluate(fix(), 1.0, True)
    second = model.evaluate(fix(timestamp=10.0), 10.0, True)
    assert first.linear_x_mps == pytest.approx(0.02)
    assert second.linear_x_mps - first.linear_x_mps <= 0.05 + 1.0e-12


def test_angular_acceleration_is_limited():
    """Avoid a step in yaw rate while beginning an alignment."""
    model = controller(
        max_angular_accel_radps2=0.50,
        initial_control_dt_s=0.10,
    )
    result = model.evaluate(fix(x=0.0, y=3.0), 1.0, True)
    assert result.angular_z_radps == pytest.approx(0.04)
    assert result.angular_acceleration_radps2 <= 0.50


def test_reversal_brakes_holds_neutral_then_changes_sign():
    """Never cross directly from positive surge to negative surge."""
    model = controller(
        max_relative_speed_mps=100.0,
        max_linear_accel_mps2=2.0,
        max_linear_decel_mps2=2.0,
        initial_control_dt_s=0.25,
        max_control_dt_s=0.25,
        reversal_neutral_s=0.40,
    )
    forward = model.evaluate(fix(x=3.0, timestamp=1.0), 1.0, True)
    braking = model.evaluate(fix(x=-3.0, timestamp=2.0), 2.0, True)
    neutral = model.evaluate(fix(x=-3.0, timestamp=2.2), 2.2, True)
    stopped = model.evaluate(fix(x=-3.0, timestamp=2.5), 2.5, True)
    holding = model.evaluate(fix(x=-3.0, timestamp=2.8), 2.8, True)
    release = model.evaluate(fix(x=-3.0, timestamp=3.0), 3.0, True)
    reverse = model.evaluate(fix(x=-3.0, timestamp=3.1), 3.1, True)
    assert forward.linear_x_mps > 0.0
    assert braking.state == RobustFollowState.REVERSAL_NEUTRAL
    assert 0.0 <= braking.linear_x_mps <= forward.linear_x_mps
    assert neutral.state == RobustFollowState.REVERSAL_NEUTRAL
    assert 0.0 <= neutral.linear_x_mps < braking.linear_x_mps
    assert stopped.state == RobustFollowState.REVERSAL_NEUTRAL
    assert stopped.linear_x_mps == 0.0
    assert holding.state == RobustFollowState.REVERSAL_NEUTRAL
    assert holding.linear_x_mps == 0.0
    assert release.linear_x_mps == 0.0
    assert reverse.linear_x_mps < 0.0


def test_linear_jerk_limits_acceleration_changes():
    """Ramp acceleration itself rather than applying an acceleration step."""
    model = controller(
        max_linear_accel_mps2=10.0,
        max_linear_jerk_mps3=1.0,
        initial_control_dt_s=0.10,
        max_control_dt_s=0.10,
    )
    first = model.evaluate(fix(timestamp=1.0), 1.0, True)
    second = model.evaluate(fix(timestamp=1.1), 1.1, True)
    third = model.evaluate(fix(timestamp=1.2), 1.2, True)
    assert first.linear_acceleration_mps2 == pytest.approx(0.10)
    assert second.linear_acceleration_mps2 == pytest.approx(0.20)
    assert third.linear_acceleration_mps2 == pytest.approx(0.30)
    assert second.linear_x_mps > first.linear_x_mps
    assert third.linear_x_mps > second.linear_x_mps


@pytest.mark.parametrize(
    'x, y, linear_sign, angular_sign',
    [
        (-3.0, 1.0, -1, -1),
        (-3.0, -1.0, -1, 1),
        (0.6, 0.2, -1, 1),
    ],
)
def test_reverse_steering_signs(x, y, linear_sign, angular_sign):
    """Preserve correct yaw signs while driving or retreating in reverse."""
    result = controller().evaluate(fix(x=x, y=y), 1.0, True)
    assert math.copysign(1.0, result.linear_x_mps) == linear_sign
    assert math.copysign(1.0, result.angular_z_radps) == angular_sign


@pytest.mark.parametrize(
    'bad_fix, now_s',
    [
        (fix(x=math.nan), 1.0),
        (fix(timestamp=2.0), 1.0),
        (fix(frame_id='sensor_frame'), 1.0),
        (fix(covariance_xy_m2=0.10), 1.0),
        (fix(x=21.0), 1.0),
    ],
)
def test_bad_measurements_are_rejected_with_exact_zero(bad_fix, now_s):
    """Cover independent numeric, time, frame, covariance, and range gates."""
    result = controller().evaluate(bad_fix, now_s, True)
    assert result.state == RobustFollowState.REJECTED_FIX
    assert result.linear_x_mps == result.angular_z_radps == 0.0


def test_stale_or_invalid_fix_bypasses_ramp_and_stops_exactly():
    """Safety stops must not wait for the normal deceleration limiter."""
    model = controller(max_linear_accel_mps2=10.0)
    moving = model.evaluate(fix(timestamp=1.0), 1.0, True)
    stale = model.evaluate(fix(timestamp=1.0), 3.0, True)
    moving_again = model.evaluate(fix(timestamp=4.0), 4.0, True)
    invalid = model.evaluate(
        fix(timestamp=4.1, variance_x_m2=1.0),
        4.1,
        True,
    )
    assert moving.linear_x_mps > 0.0
    assert stale.state == RobustFollowState.LOST_TARGET
    assert stale.linear_x_mps == stale.angular_z_radps == 0.0
    assert moving_again.linear_x_mps > 0.0
    assert invalid.state == RobustFollowState.REJECTED_FIX
    assert invalid.linear_x_mps == invalid.angular_z_radps == 0.0


def test_loss_requires_full_reacquisition():
    """Do not resume on the first fix after an acoustic outage."""
    model = controller(acquisition_fixes=2)
    model.evaluate(fix(timestamp=1.0), 1.0, True)
    tracking = model.evaluate(fix(timestamp=1.1), 1.1, True)
    lost = model.evaluate(fix(timestamp=1.1), 3.0, True)
    reacquire_one = model.evaluate(fix(timestamp=3.0), 3.0, True)
    reacquire_two = model.evaluate(fix(timestamp=3.1), 3.1, True)
    assert tracking.state == RobustFollowState.APPROACHING
    assert lost.state == RobustFollowState.LOST_TARGET
    assert reacquire_one.state == RobustFollowState.ACQUIRING
    assert reacquire_two.state == RobustFollowState.APPROACHING


def test_hard_stop_requires_new_fixes_before_motion():
    """Treat unexpected near-zero horizontal range as a latched stop."""
    model = controller(acquisition_fixes=2, max_relative_speed_mps=100.0)
    hard = model.evaluate(fix(x=0.2, timestamp=1.0), 1.0, True)
    first = model.evaluate(fix(x=3.0, timestamp=1.1), 1.1, True)
    second = model.evaluate(fix(x=3.0, timestamp=1.2), 1.2, True)
    assert hard.state == RobustFollowState.HARD_STOP
    assert hard.linear_x_mps == 0.0
    assert first.state == RobustFollowState.ACQUIRING
    assert second.state == RobustFollowState.APPROACHING


def test_jump_and_timestamp_mutation_are_rejected():
    """Reject impossible motion and reused timestamps with changed data."""
    model = controller(max_relative_speed_mps=1.0)
    model.evaluate(fix(x=3.0, timestamp=1.0), 1.0, True)
    jump = model.evaluate(fix(x=10.0, timestamp=1.1), 1.1, True)
    mutation = model.evaluate(fix(x=3.1, timestamp=1.0), 1.2, True)
    assert jump.state == RobustFollowState.REJECTED_FIX
    assert mutation.state == RobustFollowState.REJECTED_FIX
    assert jump.linear_x_mps == mutation.linear_x_mps == 0.0


def test_filtered_radial_velocity_is_observable():
    """Expose a deterministic closure estimate for scenario diagnostics."""
    model = controller(
        radial_velocity_alpha=1.0,
        max_relative_speed_mps=10.0,
    )
    model.evaluate(fix(x=3.0, timestamp=1.0), 1.0, True)
    result = model.evaluate(fix(x=3.5, timestamp=2.0), 2.0, True)
    assert result.radial_velocity_mps == pytest.approx(0.5)


def test_control_clock_rollback_is_fail_safe():
    """Stop exactly if the active clock domain jumps backwards."""
    model = controller()
    model.evaluate(fix(timestamp=5.0), 5.0, True)
    result = model.evaluate(fix(timestamp=4.0), 4.0, True)
    assert result.state == RobustFollowState.REJECTED_FIX
    assert result.linear_x_mps == result.angular_z_radps == 0.0


@pytest.mark.parametrize(
    'changes',
    [
        {'hard_stop_radius_m': 1.1},
        {'inner_radius_m': 2.5},
        {'target_radius_m': 2.0},
        {'radial_hysteresis_m': -0.1},
        {'radial_hysteresis_m': 0.70},
        {'radial_velocity_alpha': 1.1},
        {'reverse_exit_angle_rad': math.radians(100.0)},
        {'max_linear_accel_mps2': 0.0},
        {'max_linear_jerk_mps3': 0.0},
        {'acquisition_fixes': True},
        {'require_covariance': 'yes'},
        {'expected_frame': ''},
    ],
)
def test_invalid_configuration_is_rejected(changes):
    """Reject configurations which weaken deterministic safety behavior."""
    with pytest.raises(ValueError):
        RobustUuvFollowController(RobustFollowConfig(**changes))
