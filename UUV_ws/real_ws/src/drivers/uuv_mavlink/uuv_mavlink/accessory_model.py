"""Pure helpers for safe camera and light accessory commands."""

import math


def button_mask(button_index):
    """Return a MAVLink MANUAL_CONTROL mask for a zero-based button index."""
    if type(button_index) is not int or not 0 <= button_index <= 15:
        raise ValueError('El índice del botón MAVLink debe estar entre 0 y 15')
    return 1 << button_index


def normalize_light_step(value):
    """Accept only one relative light step in either direction."""
    if type(value) is not int or value not in (-1, 1):
        raise ValueError('El cambio de luces debe ser -1 o 1')
    return value


def percent_to_level(percent, steps):
    """Map a finite 0–100 percentage to the nearest discrete light level."""
    if isinstance(percent, bool) or not isinstance(percent, (int, float)):
        raise ValueError('La intensidad debe ser numérica')
    if not math.isfinite(percent) or not 0.0 <= float(percent) <= 100.0:
        raise ValueError('La intensidad debe estar entre 0 y 100 %')
    if type(steps) is not int or steps < 1:
        raise ValueError('La cantidad de niveles debe ser positiva')
    return int(round(float(percent) * steps / 100.0))


def level_to_percent(level, steps):
    """Return the bounded percentage represented by a discrete level."""
    if type(level) is not int or type(steps) is not int or steps < 1:
        raise ValueError('Nivel de luces inválido')
    bounded = max(0, min(steps, level))
    return 100.0 * bounded / steps


def accessory_blocker(enabled, connected):
    """Describe why an accessory command cannot reach MAVLink."""
    if not enabled:
        return 'Salida de accesorios deshabilitada por el lanzamiento'
    if not connected:
        return 'Autopiloto sin heartbeat reciente'
    return ''
