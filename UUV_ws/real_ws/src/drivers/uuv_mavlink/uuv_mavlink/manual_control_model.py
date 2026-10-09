"""Pure validation helpers for owner-controlled manual command settings."""

import math


def normalize_command_scale(value):
    """Return a finite Xbox/MANUAL scale inside the complete 0–1 range."""
    if isinstance(value, bool):
        raise ValueError('command_scale: tipo inválido')
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError('command_scale: valor no numérico') from exc
    if not math.isfinite(result) or not 0.0 <= result <= 1.0:
        raise ValueError('command_scale: fuera del intervalo [0, 1]')
    return result
