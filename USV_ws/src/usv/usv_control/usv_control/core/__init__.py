"""Hardware-independent control models for the USV."""

from .uuv_follow_model import (
    FollowConfig,
    FollowDecision,
    FollowState,
    RelativeFix,
    UuvFollowController,
)

__all__ = [
    "FollowConfig",
    "FollowDecision",
    "FollowState",
    "RelativeFix",
    "UuvFollowController",
]
