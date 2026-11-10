"""End-to-end tests for the hardware-independent follow/shadow safety chain."""

from usv_control.core import (
    ArbiterMode,
    ArbiterState,
    CandidateCommand,
    FollowConfig,
    FollowState,
    RelativeFix,
    ShadowCommandArbiter,
    UuvFollowController,
)


def _fix(timestamp_s: float) -> RelativeFix:
    return RelativeFix(
        x_m=2.0,
        y_m=0.0,
        timestamp_s=timestamp_s,
        frame_id='base_link',
        variance_x_m2=0.04,
        variance_y_m2=0.04,
        covariance_xy_m2=0.0,
    )


def _evaluate_shadow(arbiter, candidate, now_s):
    return arbiter.evaluate(
        candidate=candidate,
        now_s=now_s,
        requested_mode=ArbiterMode.UUV_FOLLOW,
        operator_enabled=True,
        vehicle_connected=True,
        kill_active=False,
        vehicle_state_timestamp_s=now_s,
        kill_state_timestamp_s=now_s,
    )


def test_valid_fix_reaches_only_rate_limited_shadow_output():
    """A valid fix becomes a bounded command after the neutral mode cycle."""
    follower = UuvFollowController(FollowConfig(acquisition_fixes=1))
    arbiter = ShadowCommandArbiter()

    follow = follower.evaluate(_fix(10.0), now_s=10.0, enabled=True)
    assert follow.state == FollowState.TRACKING
    candidate = CandidateCommand(
        follow.linear_x_mps,
        follow.angular_z_radps,
        timestamp_s=10.0,
        frame_id='base_link',
    )

    neutral = _evaluate_shadow(arbiter, candidate, now_s=10.0)
    active = _evaluate_shadow(arbiter, candidate, now_s=10.1)

    assert neutral.state == ArbiterState.BLOCKED
    assert neutral.linear_x_mps == neutral.angular_z_radps == 0.0
    assert active.state == ArbiterState.ACTIVE
    assert 0.0 < active.linear_x_mps <= 0.02
    assert active.angular_z_radps == 0.0


def test_fix_dropout_stops_follower_and_latches_stale_candidate():
    """Loss of acoustic updates cannot leave the previous command active."""
    follower = UuvFollowController(FollowConfig(
        acquisition_fixes=1,
        fix_timeout_s=0.20,
    ))
    arbiter = ShadowCommandArbiter()

    follow = follower.evaluate(_fix(10.0), now_s=10.0, enabled=True)
    candidate = CandidateCommand(
        follow.linear_x_mps,
        follow.angular_z_radps,
        timestamp_s=10.0,
        frame_id='base_link',
    )
    _evaluate_shadow(arbiter, candidate, now_s=10.0)
    assert _evaluate_shadow(
        arbiter, candidate, now_s=10.1
    ).state == ArbiterState.ACTIVE

    lost = follower.evaluate(_fix(10.0), now_s=10.5, enabled=True)
    stopped = _evaluate_shadow(arbiter, candidate, now_s=10.5)

    assert lost.state == FollowState.LOST_TARGET
    assert lost.linear_x_mps == lost.angular_z_radps == 0.0
    assert stopped.state == ArbiterState.ABORTED
    assert 'timed out' in stopped.reason
    assert stopped.linear_x_mps == stopped.angular_z_radps == 0.0
    assert stopped.abort_latched is True
