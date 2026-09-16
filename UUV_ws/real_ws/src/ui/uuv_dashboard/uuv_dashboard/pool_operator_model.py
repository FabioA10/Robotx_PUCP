"""Pure policy for the manually supervised pool profile."""


OPERATOR_MODES = ('MANUAL', 'ALT_HOLD', 'POSHOLD', 'GUIDED')


def mode_request_allowed(*, enabled, connected, armed, deadman):
    """Allow a GUI mode request only while the vehicle is disarmed and idle."""
    return all((enabled, connected, armed is False, deadman is False))
