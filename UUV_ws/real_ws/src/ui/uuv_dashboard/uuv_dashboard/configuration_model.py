"""Persistent, validated operator profiles for the GUIDED dashboard."""

from copy import deepcopy
import json
import math
import os
from pathlib import Path
import tempfile


CONFIG_VERSION = 2
DEFAULT_PROFILE_NAME = 'Piscina conservador'

SESSION_DEFAULTS = {
    'sensor_depth_m': 0.0,
    'maximum_depth_m': 0.20,
    'radius_m': 0.50,
    'max_horizontal_speed_mps': 0.20,
    'max_vertical_speed_mps': 0.10,
    'max_yaw_speed_deg_s': 10.0,
}

CONTROLLER_DEFAULTS = {
    'xy_tolerance_m': 0.030,
    'recovery_radius_m': 0.080,
    'recovery_speed_mps': 0.080,
    'recovery_min_speed_mps': 0.020,
    'recovery_delay_s': 2.0,
    'position_gain_hz': 0.80,
    'velocity_damping_s': 1.20,
}

VEHICLE_DEFAULTS = {
    'propulsion_signal_limit_pct': 37.5,
    'max_ascent_speed_mps': 0.10,
    'max_descent_speed_mps': 0.10,
    'max_vertical_accel_mps2': 0.50,
}

DISPLAY_DEFAULTS = {
    'sonar_visible_range_m': 2.5,
    'sonar_persistence_s': 1.5,
    'sonar_visual_threshold': 20.0,
}

RAW_PARAMETER_NAMES = (
    'MOT_PWM_MIN',
    'MOT_PWM_MAX',
    'WPNAV_SPEED_UP',
    'WPNAV_SPEED_DN',
    'WPNAV_ACCEL_Z',
)


def default_profile():
    """Return an independent conservative configuration profile."""
    return {
        'session': deepcopy(SESSION_DEFAULTS),
        'controller': deepcopy(CONTROLLER_DEFAULTS),
        'vehicle': deepcopy(VEHICLE_DEFAULTS),
        'display': dict(DISPLAY_DEFAULTS, sonar_source='filtered'),
    }


def default_document():
    """Return the first-run profile document."""
    return {
        'version': CONFIG_VERSION,
        'active_profile': DEFAULT_PROFILE_NAME,
        'profiles': {DEFAULT_PROFILE_NAME: default_profile()},
        'last_vehicle_snapshot': None,
    }


def _number(value, name, minimum, maximum):
    if isinstance(value, bool):
        raise ValueError(f'{name}: un booleano no es un número válido')
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f'{name}: valor no numérico') from exc
    if not math.isfinite(result) or not minimum <= result <= maximum:
        raise ValueError(f'{name}: debe estar entre {minimum:g} y {maximum:g}')
    return result


def _group(source, defaults, ranges):
    source = source if isinstance(source, dict) else {}
    return {
        key: _number(source.get(key, default), key, *ranges[key])
        for key, default in defaults.items()
    }


def normalize_profile(profile):
    """Validate a profile and fill fields introduced by newer versions."""
    profile = profile if isinstance(profile, dict) else {}
    session = _group(
        profile.get('session'),
        SESSION_DEFAULTS,
        {
            'sensor_depth_m': (0.0, 0.50),
            'maximum_depth_m': (0.20, 3.0),
            'radius_m': (0.50, 5.0),
            'max_horizontal_speed_mps': (0.001, 2.0),
            'max_vertical_speed_mps': (0.001, 1.0),
            'max_yaw_speed_deg_s': (0.001, 180.0),
        },
    )
    controller = _group(
        profile.get('controller'),
        CONTROLLER_DEFAULTS,
        {
            'xy_tolerance_m': (0.010, 0.150),
            'recovery_radius_m': (0.015, 0.500),
            'recovery_speed_mps': (0.001, 2.0),
            'recovery_min_speed_mps': (0.001, 2.0),
            'recovery_delay_s': (0.5, 10.0),
            'position_gain_hz': (0.10, 2.0),
            'velocity_damping_s': (0.05, 3.0),
        },
    )
    vehicle = _group(
        profile.get('vehicle'),
        VEHICLE_DEFAULTS,
        {
            'propulsion_signal_limit_pct': (25.0, 50.0),
            'max_ascent_speed_mps': (0.10, 0.30),
            'max_descent_speed_mps': (0.10, 0.30),
            'max_vertical_accel_mps2': (0.50, 1.0),
        },
    )
    display_source = profile.get('display')
    display_source = display_source if isinstance(display_source, dict) else {}
    display = _group(
        display_source,
        DISPLAY_DEFAULTS,
        {
            'sonar_visible_range_m': (0.5, 50.0),
            'sonar_persistence_s': (0.2, 10.0),
            'sonar_visual_threshold': (0.0, 255.0),
        },
    )
    sonar_source = str(display_source.get('sonar_source', 'filtered')).strip().lower()
    if sonar_source not in ('filtered', 'raw'):
        raise ValueError('sonar_source: debe ser filtered o raw')
    display['sonar_source'] = sonar_source

    if controller['recovery_radius_m'] < controller['xy_tolerance_m'] + 0.005:
        raise ValueError('Recuperar desde debe superar la tolerancia XY en al menos 0.005 m')
    if controller['recovery_radius_m'] > min(0.50, session['radius_m']):
        raise ValueError('Recuperar desde no puede superar el radio de la sesión')
    if controller['recovery_speed_mps'] > session['max_horizontal_speed_mps']:
        raise ValueError('La recuperación supera la velocidad horizontal de la sesión')
    if controller['recovery_min_speed_mps'] > controller['recovery_speed_mps']:
        raise ValueError('El impulso mínimo supera la recuperación máxima')

    return {
        'session': session,
        'controller': controller,
        'vehicle': vehicle,
        'display': display,
    }


def profile_to_raw_parameters(profile):
    """Convert descriptive SI controls to the five ArduSub parameters."""
    vehicle = normalize_profile(profile)['vehicle']
    pwm_span = round(400.0 * vehicle['propulsion_signal_limit_pct'] / 100.0)
    return {
        'MOT_PWM_MIN': float(1500 - pwm_span),
        'MOT_PWM_MAX': float(1500 + pwm_span),
        'WPNAV_SPEED_UP': float(round(100.0 * vehicle['max_ascent_speed_mps'], 3)),
        'WPNAV_SPEED_DN': float(round(100.0 * vehicle['max_descent_speed_mps'], 3)),
        'WPNAV_ACCEL_Z': float(round(100.0 * vehicle['max_vertical_accel_mps2'], 3)),
    }


def normalize_raw_parameters(values, observed=False):
    """Validate requested values or a broader read-only vehicle snapshot."""
    if not isinstance(values, dict):
        raise ValueError('El conjunto de parámetros debe ser un objeto')
    missing = [name for name in RAW_PARAMETER_NAMES if name not in values]
    if missing:
        raise ValueError('Faltan parámetros: ' + ', '.join(missing))
    ranges = (
        {
            'MOT_PWM_MIN': (1100, 1499),
            'MOT_PWM_MAX': (1501, 1900),
            'WPNAV_SPEED_UP': (0, 1000),
            'WPNAV_SPEED_DN': (0, 500),
            'WPNAV_ACCEL_Z': (0, 500),
        }
        if observed
        else {
            'MOT_PWM_MIN': (1300, 1400),
            'MOT_PWM_MAX': (1600, 1700),
            'WPNAV_SPEED_UP': (10, 30),
            'WPNAV_SPEED_DN': (10, 30),
            'WPNAV_ACCEL_Z': (50, 100),
        }
    )
    result = {
        name: _number(values[name], name, *ranges[name])
        for name in RAW_PARAMETER_NAMES
    }
    low_span = 1500.0 - result['MOT_PWM_MIN']
    high_span = result['MOT_PWM_MAX'] - 1500.0
    if abs(low_span - high_span) > 0.5:
        raise ValueError('Los límites PWM deben ser simétricos alrededor de 1500 µs')
    return result


def raw_parameters_to_vehicle(values):
    """Convert verified ArduSub values back to descriptive SI controls."""
    raw = normalize_raw_parameters(values, observed=True)
    return {
        'propulsion_signal_limit_pct': (
            (raw['MOT_PWM_MAX'] - 1500.0) * 100.0 / 400.0
        ),
        'max_ascent_speed_mps': raw['WPNAV_SPEED_UP'] / 100.0,
        'max_descent_speed_mps': raw['WPNAV_SPEED_DN'] / 100.0,
        'max_vertical_accel_mps2': raw['WPNAV_ACCEL_Z'] / 100.0,
    }


def raw_parameters_match(first, second, tolerance=0.05):
    """Return whether two complete raw parameter sets match after readback."""
    try:
        left = normalize_raw_parameters(first, observed=True)
        right = normalize_raw_parameters(second, observed=True)
    except ValueError:
        return False
    return all(abs(left[name] - right[name]) <= tolerance for name in RAW_PARAMETER_NAMES)


def _profile_name(value):
    name = ' '.join(str(value).strip().split())
    if not name or len(name) > 64 or any(character in name for character in '/\\'):
        raise ValueError('El nombre del perfil debe tener entre 1 y 64 caracteres')
    return name


def _migrate_profile(profile, version):
    migrated = deepcopy(profile) if isinstance(profile, dict) else {}
    if version < 2:
        vehicle = migrated.get('vehicle')
        vehicle = dict(vehicle) if isinstance(vehicle, dict) else {}
        try:
            previous_accel = float(vehicle.get('max_vertical_accel_mps2', 0.50))
        except (TypeError, ValueError) as exc:
            raise ValueError(
                'max_vertical_accel_mps2: valor no numérico en el perfil anterior'
            ) from exc
        vehicle['max_vertical_accel_mps2'] = max(0.50, previous_accel)
        migrated['vehicle'] = vehicle
    return migrated


def _vehicle_identity(value):
    if value is None:
        return None
    if not isinstance(value, dict):
        raise ValueError('La identidad del vehículo debe ser un objeto')
    result = {}
    for key in ('system_id', 'component_id'):
        raw = value.get(key)
        if type(raw) is not int or not 1 <= raw <= 255:
            raise ValueError(f'{key}: identificador MAVLink inválido')
        result[key] = raw
    firmware = value.get('firmware')
    if firmware is not None:
        firmware = str(firmware).strip()
        if not firmware or len(firmware) > 128:
            raise ValueError('firmware: valor inválido')
        result['firmware'] = firmware
    return result


def normalize_snapshot(snapshot):
    """Normalize a versioned snapshot, including legacy value-only snapshots."""
    if snapshot is None:
        return None
    if not isinstance(snapshot, dict):
        raise ValueError('El respaldo del vehículo debe ser un objeto')
    if 'values' not in snapshot:
        return {
            'values': normalize_raw_parameters(snapshot, observed=True),
            'vehicle': None,
            'captured_at': None,
        }
    captured_at = snapshot.get('captured_at')
    if captured_at is not None:
        captured_at = str(captured_at).strip()
        if not captured_at or len(captured_at) > 64:
            raise ValueError('captured_at: valor inválido')
    return {
        'values': normalize_raw_parameters(snapshot.get('values'), observed=True),
        'vehicle': _vehicle_identity(snapshot.get('vehicle')),
        'captured_at': captured_at,
    }


def snapshot_matches_vehicle(snapshot, vehicle):
    """Require an exact, firmware-confirmed vehicle match before restore."""
    try:
        expected = normalize_snapshot(snapshot)['vehicle']
        observed = _vehicle_identity(vehicle)
    except (TypeError, ValueError):
        return False
    if expected is None or observed is None:
        return False
    expected_firmware = expected.get('firmware', '')
    observed_firmware = observed.get('firmware', '')
    if (
        not expected_firmware
        or not observed_firmware
        or 'no confirmado' in expected_firmware.lower()
        or 'no confirmado' in observed_firmware.lower()
    ):
        return False
    return expected == observed


def normalize_document(document):
    """Validate a complete persisted document."""
    if not isinstance(document, dict):
        raise ValueError('El archivo de configuración no contiene un objeto')
    version = document.get('version', 1)
    if type(version) is not int or version < 1:
        raise ValueError('Versión de configuración inválida')
    if version > CONFIG_VERSION:
        raise ValueError(
            f'El archivo usa la versión futura {version}; esta interfaz admite '
            f'hasta la versión {CONFIG_VERSION}'
        )
    source_profiles = document.get('profiles')
    if not isinstance(source_profiles, dict) or not source_profiles:
        raise ValueError('El archivo no contiene perfiles')
    profiles = {
        _profile_name(name): normalize_profile(_migrate_profile(profile, version))
        for name, profile in source_profiles.items()
    }
    active = _profile_name(document.get('active_profile', DEFAULT_PROFILE_NAME))
    if active not in profiles:
        active = next(iter(profiles))
    snapshot = normalize_snapshot(document.get('last_vehicle_snapshot'))
    return {
        'version': CONFIG_VERSION,
        'active_profile': active,
        'profiles': profiles,
        'last_vehicle_snapshot': snapshot,
    }


def update_profile(document, name, profile):
    """Return a normalized document with one saved and selected profile."""
    normalized = normalize_document(document)
    name = _profile_name(name)
    normalized['profiles'][name] = normalize_profile(profile)
    normalized['active_profile'] = name
    return normalized


def rename_profile(document, current_name, new_name):
    """Rename one profile without silently replacing another profile."""
    normalized = normalize_document(document)
    current_name = _profile_name(current_name)
    new_name = _profile_name(new_name)
    if current_name not in normalized['profiles']:
        raise ValueError('El perfil que se desea renombrar no existe')
    if new_name != current_name and new_name in normalized['profiles']:
        raise ValueError('Ya existe un perfil con ese nombre')
    if new_name != current_name:
        reordered = {}
        for name, profile in normalized['profiles'].items():
            reordered[new_name if name == current_name else name] = profile
        normalized['profiles'] = reordered
    if normalized['active_profile'] == current_name:
        normalized['active_profile'] = new_name
    return normalized


def delete_profile(document, name):
    """Delete one profile while always keeping at least one valid profile."""
    normalized = normalize_document(document)
    name = _profile_name(name)
    if name not in normalized['profiles']:
        raise ValueError('El perfil que se desea eliminar no existe')
    if len(normalized['profiles']) == 1:
        raise ValueError('No se puede eliminar el único perfil')
    del normalized['profiles'][name]
    if normalized['active_profile'] == name:
        normalized['active_profile'] = next(iter(normalized['profiles']))
    return normalized


def set_vehicle_snapshot(document, values, vehicle=None, captured_at=None):
    """Return a document containing the last pre-apply vehicle snapshot."""
    normalized = normalize_document(document)
    normalized['last_vehicle_snapshot'] = normalize_snapshot(
        {
            'values': values,
            'vehicle': vehicle,
            'captured_at': captured_at,
        }
    )
    return normalized


class ConfigurationStore:
    """Load and atomically save local dashboard profiles."""

    def __init__(self, path=None):
        if path is None:
            configured = os.environ.get('ROBOTX_UUV_CONFIG', '').strip()
            if configured:
                path = configured
            else:
                config_root = os.environ.get('XDG_CONFIG_HOME', '').strip()
                root = Path(config_root) if config_root else Path.home() / '.config'
                path = root / 'robotx_uuv' / 'sequence_profiles.json'
        self.path = Path(path).expanduser()
        self.last_notice = ''

    def load(self):
        """Load profiles; first run uses conservative defaults."""
        if not self.path.exists():
            self.last_notice = ''
            return default_document()
        with self.path.open('r', encoding='utf-8') as stream:
            document = json.load(stream)
        previous = document.get('version', 1) if isinstance(document, dict) else None
        normalized = normalize_document(document)
        self.last_notice = (
            'Perfil migrado a V2: WPNAV_ACCEL_Z se elevó al mínimo documentado '
            'de 0.50 m/s²; revisa y guarda antes de aplicar al ROV.'
            if previous == 1
            else ''
        )
        return normalized

    def save(self, document):
        """Atomically replace the profile file and return normalized content."""
        normalized = normalize_document(document)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        descriptor, temporary = tempfile.mkstemp(
            prefix=self.path.name + '.',
            suffix='.tmp',
            dir=str(self.path.parent),
            text=True,
        )
        try:
            with os.fdopen(descriptor, 'w', encoding='utf-8') as stream:
                json.dump(normalized, stream, indent=2, sort_keys=True, allow_nan=False)
                stream.write('\n')
                stream.flush()
                os.fsync(stream.fileno())
            os.chmod(temporary, 0o600)
            os.replace(temporary, self.path)
        except Exception:
            try:
                os.unlink(temporary)
            except FileNotFoundError:
                pass
            raise
        return normalized
