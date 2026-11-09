"""Adapter from the robust pure controller to the scenario protocol."""

from typing import Optional

from usv_control.core.uuv_follow_model import RelativeFix
from usv_control.core.uuv_follow_v2_model import RobustUuvFollowController
from usv_control.simulation.follow_scenarios import SimCommand, SimFix


class RobustFollowSimulationAdapter:
    """Run the production preview-v2 core inside the offline simulator."""

    def __init__(self, controller: RobustUuvFollowController) -> None:
        """Wrap one controller instance for a complete scenario run."""
        self.controller = controller

    def reset(self) -> None:
        """Reset the wrapped controller and all its acquisition state."""
        self.controller.reset()

    def step(
        self,
        fix: Optional[SimFix],
        now_s: float,
        _dt_s: float,
    ) -> SimCommand:
        """Convert a simulated fix and return the candidate preview command."""
        relative_fix = None
        if fix is not None:
            relative_fix = RelativeFix(
                x_m=fix.x_m,
                y_m=fix.y_m,
                timestamp_s=fix.timestamp_s,
                frame_id='base_link',
                variance_x_m2=fix.variance_x_m2,
                variance_y_m2=fix.variance_y_m2,
                covariance_xy_m2=fix.covariance_xy_m2,
            )
        decision = self.controller.evaluate(relative_fix, now_s, True)
        return SimCommand(
            linear_x_mps=decision.linear_x_mps,
            angular_z_radps=decision.angular_z_radps,
            state=decision.state.value,
        )
