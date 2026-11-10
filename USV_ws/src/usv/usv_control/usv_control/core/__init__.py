"""Hardware-independent control models for the USV."""

from .command_arbiter import (
    ArbiterConfig,
    ArbiterDecision,
    ArbiterMode,
    ArbiterState,
    CandidateCommand,
    ShadowCommandArbiter,
)

from .uuv_follow_model import (
    FollowConfig,
    FollowDecision,
    FollowState,
    RelativeFix,
    UuvFollowController,
)

__all__ = [
    "ArbiterConfig",
    "ArbiterDecision",
    "ArbiterMode",
    "ArbiterState",
    "CandidateCommand",
    "FollowConfig",
    "FollowDecision",
    "FollowState",
    "RelativeFix",
    "ShadowCommandArbiter",
    "UuvFollowController",
]
