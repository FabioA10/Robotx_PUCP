"""Unit tests for the hardware-independent UUV follower."""

import math

from usv_control.core import (
    FollowConfig,
    FollowState,
    RelativeFix,
    UuvFollowController,
)


def controller(acquisition_fixes=1, **overrides):
    config = FollowConfig(acquisition_fixes=acquisition_fixes, **overrides)
    return UuvFollowController(config)


def fix(x=2.0, y=0.0, timestamp=10.0, **overrides):
    defaults = {
        "variance_x_m2": 0.04,
        "variance_y_m2": 0.04,
        "covariance_xy_m2": 0.0,
    }
    defaults.update(overrides)
    return RelativeFix(x, y, timestamp, **defaults)


def test_disabled_always_stops():
    result = controller().evaluate(fix(), 10.0, enabled=False)
    assert result.state == FollowState.DISABLED
    assert result.linear_x_mps == result.angular_z_radps == 0.0


def test_non_boolean_enable_cannot_activate_following():
    result = controller().evaluate(fix(), 10.0, enabled="true")
    assert result.state == FollowState.DISABLED


def test_no_fix_stops_while_waiting():
    result = controller().evaluate(None, 10.0, enabled=True)
    assert result.state == FollowState.WAITING_FIX
    assert result.linear_x_mps == 0.0


def test_non_finite_fix_is_rejected():
    result = controller().evaluate(fix(x=math.nan), 10.0, enabled=True)
    assert result.state == FollowState.REJECTED_FIX
    assert result.linear_x_mps == 0.0


def test_non_numeric_fix_is_rejected_without_raising():
    result = controller().evaluate(fix(x="2.0"), 10.0, enabled=True)
    assert result.state == FollowState.REJECTED_FIX


def test_three_distinct_fixes_are_required_for_acquisition():
    model = controller(acquisition_fixes=3)
    assert model.evaluate(fix(timestamp=10.0), 10.0, True).state == FollowState.WAITING_FIX
    assert model.evaluate(fix(timestamp=10.1), 10.1, True).state == FollowState.WAITING_FIX
    assert model.evaluate(fix(timestamp=10.2), 10.2, True).state == FollowState.TRACKING


def test_repeated_fix_does_not_complete_acquisition():
    model = controller(acquisition_fixes=2)
    assert model.evaluate(fix(), 10.0, True).state == FollowState.WAITING_FIX
    assert model.evaluate(fix(), 10.1, True).state == FollowState.WAITING_FIX


def test_stale_fix_causes_lost_target_and_zero_command():
    result = controller().evaluate(fix(timestamp=1.0), 10.0, enabled=True)
    assert result.state == FollowState.LOST_TARGET
    assert result.linear_x_mps == result.angular_z_radps == 0.0


def test_target_inside_standoff_is_held_without_reverse():
    result = controller().evaluate(fix(x=0.6), 10.0, enabled=True)
    assert result.state == FollowState.HOLDING
    assert result.linear_x_mps == 0.0


def test_target_ahead_produces_bounded_forward_motion():
    result = controller().evaluate(fix(x=5.0), 10.0, enabled=True)
    assert result.state == FollowState.TRACKING
    assert result.linear_x_mps == 0.2
    assert result.angular_z_radps == 0.0


def test_large_bearing_pivots_without_forward_motion():
    result = controller().evaluate(fix(x=0.5, y=2.0), 10.0, enabled=True)
    assert result.state == FollowState.TRACKING
    assert result.linear_x_mps == 0.0
    assert result.angular_z_radps == 0.3


def test_right_hand_target_has_negative_turn_sign():
    result = controller().evaluate(fix(x=2.0, y=-0.2), 10.0, enabled=True)
    assert result.angular_z_radps < 0.0


def test_impossible_position_jump_is_rejected():
    model = controller(max_relative_speed_mps=1.0)
    assert model.evaluate(fix(x=2.0, timestamp=10.0), 10.0, True).state == FollowState.TRACKING
    result = model.evaluate(fix(x=10.0, timestamp=10.1), 10.1, True)
    assert result.state == FollowState.REJECTED_FIX
    assert result.linear_x_mps == 0.0


def test_out_of_order_timestamp_is_rejected():
    model = controller()
    model.evaluate(fix(timestamp=10.0), 10.0, True)
    result = model.evaluate(fix(timestamp=9.9), 10.0, True)
    assert result.state == FollowState.REJECTED_FIX


def test_out_of_range_fix_is_rejected():
    result = controller(max_range_m=5.0).evaluate(
        fix(x=6.0), 10.0, enabled=True
    )
    assert result.state == FollowState.REJECTED_FIX


def test_invalid_quality_is_rejected():
    result = controller(min_quality=0.5).evaluate(
        fix(quality=0.2), 10.0, enabled=True
    )
    assert result.state == FollowState.REJECTED_FIX


def test_loss_requires_reacquisition():
    model = controller(acquisition_fixes=2)
    model.evaluate(fix(timestamp=10.0), 10.0, True)
    assert model.evaluate(fix(timestamp=10.1), 10.1, True).state == FollowState.TRACKING
    assert model.evaluate(fix(timestamp=10.1), 20.0, True).state == FollowState.LOST_TARGET
    assert model.evaluate(fix(timestamp=20.0), 20.0, True).state == FollowState.WAITING_FIX


def test_changed_sample_with_same_timestamp_is_rejected():
    model = controller()
    assert model.evaluate(fix(x=2.0), 10.0, True).state == FollowState.TRACKING
    result = model.evaluate(fix(x=4.0), 10.1, True)
    assert result.state == FollowState.REJECTED_FIX
    assert result.linear_x_mps == result.angular_z_radps == 0.0


def test_repeated_outlier_never_becomes_the_new_baseline():
    model = controller(max_relative_speed_mps=1.0)
    model.evaluate(fix(x=2.0, timestamp=10.0), 10.0, True)
    first = model.evaluate(fix(x=8.0, timestamp=10.1), 10.1, True)
    second = model.evaluate(fix(x=8.1, timestamp=10.2), 10.2, True)
    assert first.state == FollowState.REJECTED_FIX
    assert second.state == FollowState.REJECTED_FIX


def test_wrong_frame_is_rejected():
    result = controller().evaluate(
        fix(frame_id="usbl_x150"), 10.0, enabled=True
    )
    assert result.state == FollowState.REJECTED_FIX


def test_missing_covariance_is_rejected_by_default():
    result = controller().evaluate(
        fix(variance_x_m2=None, variance_y_m2=None, covariance_xy_m2=None),
        10.0,
        enabled=True,
    )
    assert result.state == FollowState.REJECTED_FIX


def test_excessive_or_impossible_covariance_is_rejected():
    excessive = controller(max_horizontal_stddev_m=0.5).evaluate(
        fix(variance_x_m2=1.0), 10.0, True
    )
    impossible = controller().evaluate(
        fix(variance_x_m2=0.04, variance_y_m2=0.04, covariance_xy_m2=0.1),
        10.0,
        True,
    )
    assert excessive.state == FollowState.REJECTED_FIX
    assert impossible.state == FollowState.REJECTED_FIX


def test_invalid_configuration_is_rejected():
    invalid_configs = (
        {"distance_deadband_m": -0.1},
        {"max_linear_speed_mps": math.nan},
        {"pivot_threshold_rad": 0.0},
        {"min_quality": 1.1},
        {"expected_frame": ""},
        {"acquisition_fixes": True},
        {"require_covariance": "yes"},
    )
    for overrides in invalid_configs:
        try:
            controller(**overrides)
        except ValueError:
            continue
        raise AssertionError("invalid config was accepted: %r" % overrides)
