"""Pure validation and safety rules for restricted ArduSub parameters."""

import math


PARAMETER_NAMES = (
    'MOT_PWM_MIN',
    'MOT_PWM_MAX',
    'WPNAV_SPEED_UP',
    'WPNAV_SPEED_DN',
    'WPNAV_ACCEL_Z',
)

READ_ONLY_PARAMETER_NAMES = (
    'MOT_BAT_CURR_MAX',
    'MOT_BAT_CURR_TC',
    'MOT_SLEW_UP_TIME',
    'MOT_SLEW_DN_TIME',
    'MOT_PWM_TYPE',
    'BATT_MONITOR',
    'BATT_CAPACITY',
    'BATT_LOW_VOLT',
    'BATT_CRT_VOLT',
    'BATT_FS_LOW_ACT',
    'BATT_FS_CRT_ACT',
    'ARMING_CHECK',
    'FS_GCS_ENABLE',
    'FS_PILOT_INPUT',
    'FS_LEAK_ENABLE',
)

REQUEST_PARAMETER_NAMES = PARAMETER_NAMES + READ_ONLY_PARAMETER_NAMES


def parameter_name(value):
    """Normalize MAVLink's fixed-width string/bytes parameter identifier."""
    if isinstance(value, bytes):
        value = value.split(b'\0', 1)[0].decode('ascii', errors='ignore')
    return str(value).split('\0', 1)[0].strip()


def _number(value, name, minimum, maximum):
    if isinstance(value, bool):
        raise ValueError(f'{name}: tipo inválido')
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f'{name}: valor no numérico') from exc
    if not math.isfinite(result) or not minimum <= result <= maximum:
        raise ValueError(f'{name}: fuera del intervalo [{minimum:g}, {maximum:g}]')
    return result


def normalize_parameter_values(values):
    """Accept only the five bounded parameters exposed by the dashboard."""
    if not isinstance(values, dict):
        raise ValueError('values debe ser un objeto')
    missing = [name for name in PARAMETER_NAMES if name not in values]
    extra = [name for name in values if name not in PARAMETER_NAMES]
    if missing:
        raise ValueError('Faltan parámetros: ' + ', '.join(missing))
    if extra:
        raise ValueError('Parámetros no permitidos: ' + ', '.join(sorted(extra)))
    normalized = {
        'MOT_PWM_MIN': _number(values['MOT_PWM_MIN'], 'MOT_PWM_MIN', 1300, 1400),
        'MOT_PWM_MAX': _number(values['MOT_PWM_MAX'], 'MOT_PWM_MAX', 1600, 1700),
        'WPNAV_SPEED_UP': _number(values['WPNAV_SPEED_UP'], 'WPNAV_SPEED_UP', 10, 30),
        'WPNAV_SPEED_DN': _number(values['WPNAV_SPEED_DN'], 'WPNAV_SPEED_DN', 10, 30),
        'WPNAV_ACCEL_Z': _number(values['WPNAV_ACCEL_Z'], 'WPNAV_ACCEL_Z', 50, 100),
    }
    low_span = 1500.0 - normalized['MOT_PWM_MIN']
    high_span = normalized['MOT_PWM_MAX'] - 1500.0
    if abs(low_span - high_span) > 0.5:
        raise ValueError('MOT_PWM_MIN y MOT_PWM_MAX no son simétricos respecto de 1500')
    return normalized


def normalize_observed_parameter_values(values):
    """Validate a complete readback without treating legacy values as writable."""
    if not isinstance(values, dict):
        raise ValueError('values debe ser un objeto')
    missing = [name for name in PARAMETER_NAMES if name not in values]
    if missing:
        raise ValueError('Faltan parámetros: ' + ', '.join(missing))
    normalized = {
        'MOT_PWM_MIN': _number(values['MOT_PWM_MIN'], 'MOT_PWM_MIN', 1100, 1499),
        'MOT_PWM_MAX': _number(values['MOT_PWM_MAX'], 'MOT_PWM_MAX', 1501, 1900),
        'WPNAV_SPEED_UP': _number(values['WPNAV_SPEED_UP'], 'WPNAV_SPEED_UP', 0, 1000),
        'WPNAV_SPEED_DN': _number(values['WPNAV_SPEED_DN'], 'WPNAV_SPEED_DN', 0, 500),
        'WPNAV_ACCEL_Z': _number(values['WPNAV_ACCEL_Z'], 'WPNAV_ACCEL_Z', 0, 500),
    }
    low_span = 1500.0 - normalized['MOT_PWM_MIN']
    high_span = normalized['MOT_PWM_MAX'] - 1500.0
    if abs(low_span - high_span) > 0.5:
        raise ValueError('MOT_PWM_MIN y MOT_PWM_MAX no son simétricos respecto de 1500')
    return normalized


def parameter_type(value):
    """Validate one MAV_PARAM_TYPE received with PARAM_VALUE."""
    if type(value) is not int or not 1 <= value <= 10:
        raise ValueError('Tipo MAVLink de parámetro no válido')
    return value


def parameter_sets_match(first, second, tolerance=0.05):
    """Compare two complete parameter sets using readback tolerance."""
    try:
        left = normalize_observed_parameter_values(first)
        right = normalize_observed_parameter_values(second)
    except ValueError:
        return False
    return all(abs(left[name] - right[name]) <= tolerance for name in PARAMETER_NAMES)


def write_blockers(
    *,
    write_enabled,
    connected,
    heartbeat_fresh,
    armed,
    mode,
    deadman,
    sequence_active,
):
    """Return every independent reason that forbids persistent writes."""
    reasons = []
    if not write_enabled:
        reasons.append('escritura de parámetros no habilitada en el lanzamiento')
    if not connected or not heartbeat_fresh:
        reasons.append('sin heartbeat reciente del autopiloto')
    if armed:
        reasons.append('el vehículo está armado')
    if mode != 'MANUAL':
        reasons.append('el modo real no es MANUAL')
    if deadman:
        reasons.append('RB/deadman está presionado')
    if sequence_active:
        reasons.append('hay una preparación o secuencia GUIDED activa')
    return reasons
