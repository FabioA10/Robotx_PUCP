"""Versioned, atomic storage and editing helpers for reusable UUV missions."""

from copy import deepcopy
import json
import math
import os
from pathlib import Path
import tempfile


MISSION_VERSION = 1


def _name(value):
    result = ' '.join(str(value).strip().split())
    if not result or len(result) > 64 or any(character in result for character in '/\\'):
        raise ValueError('El nombre de misión debe tener entre 1 y 64 caracteres')
    return result


def _number(value, label, minimum, maximum):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'{label}: se requiere un número')
    result = float(value)
    if not math.isfinite(result) or not minimum <= result <= maximum:
        raise ValueError(f'{label}: debe estar entre {minimum:g} y {maximum:g}')
    return result


def normalize_step(step):
    """Validate and copy one relative or absolute editor entry."""
    if not isinstance(step, dict):
        raise ValueError('Cada paso debe ser un objeto')
    kind = step.get('kind')
    if kind == 'absolute_waypoint':
        return {
            'kind': kind,
            'x': _number(step.get('x'), 'X', -5.0, 5.0),
            'y': _number(step.get('y'), 'Y', -5.0, 5.0),
            'depth': _number(step.get('depth'), 'Profundidad', 0.0, 3.0),
            'yaw_deg': _number(step.get('yaw_deg'), 'Ángulo', -180.0, 180.0),
            'speed': _number(step.get('speed'), 'Velocidad X-Y', 0.001, 2.0),
            'vertical_speed': _number(
                step.get('vertical_speed'), 'Velocidad vertical', 0.001, 1.0
            ),
            'yaw_speed_deg': _number(
                step.get('yaw_speed_deg'), 'Velocidad angular', 0.001, 180.0
            ),
            'tolerance_x': _number(step.get('tolerance_x'), 'Tolerancia X', 0.01, 0.15),
            'tolerance_y': _number(step.get('tolerance_y'), 'Tolerancia Y', 0.01, 0.15),
            'tolerance_depth': _number(
                step.get('tolerance_depth'), 'Tolerancia de profundidad', 0.005, 0.10
            ),
            'tolerance_yaw_deg': _number(
                step.get('tolerance_yaw_deg'), 'Tolerancia angular', 1.0, 20.0
            ),
            'dwell': _number(step.get('dwell'), 'Confirmación estable', 0.5, 5.0),
        }
    if kind == 'wait':
        return {
            'kind': kind,
            'duration': _number(step.get('duration'), 'Duración', 0.0, 30.0),
        }
    timed = {'velocity_surge', 'velocity_sway', 'velocity_heave', 'velocity_yaw'}
    if kind in timed:
        return {
            'kind': kind,
            'speed': _number(step.get('speed'), 'Velocidad', -math.pi, math.pi),
            'duration': _number(step.get('duration'), 'Duración', 0.1, 5.0),
        }
    if kind in {'advance', 'lateral', 'turn', 'depth'}:
        value_range = (-180.0, 180.0) if kind == 'turn' else (-5.0, 5.0)
        if kind == 'depth':
            value_range = (0.0, 3.0)
        return {
            'kind': kind,
            'value': _number(step.get('value'), 'Objetivo', *value_range),
            'speed': _number(step.get('speed'), 'Velocidad máxima', 0.001, math.pi),
        }
    raise ValueError('Tipo de movimiento desconocido')


def normalize_mission(mission):
    """Normalize both editors so switching tabs never discards work."""
    if not isinstance(mission, dict):
        raise ValueError('La misión debe ser un objeto')
    mode = mission.get('plan_mode', 'relative')
    if mode not in ('relative', 'absolute'):
        raise ValueError('El modo de misión debe ser relative o absolute')
    relative = mission.get('relative_steps', [])
    absolute = mission.get('absolute_steps', [])
    if not isinstance(relative, list) or not isinstance(absolute, list):
        raise ValueError('Las listas de la misión no son válidas')
    if len(relative) > 50 or len(absolute) > 50:
        raise ValueError('Cada plan admite como máximo 50 pasos')
    normalized_relative = [normalize_step(step) for step in relative]
    normalized_absolute = [normalize_step(step) for step in absolute]
    if any(step['kind'] == 'absolute_waypoint' for step in normalized_relative):
        raise ValueError('El plan relativo contiene un punto absoluto')
    if any(step['kind'] != 'absolute_waypoint' for step in normalized_absolute):
        raise ValueError('El plan absoluto contiene una orden relativa')
    return {
        'plan_mode': mode,
        'relative_steps': normalized_relative,
        'absolute_steps': normalized_absolute,
    }


def empty_document():
    """Return the first-run mission library."""
    return {
        'version': MISSION_VERSION,
        'active_mission': None,
        'missions': {},
        'draft': normalize_mission({}),
    }


def normalize_document(document):
    """Validate a complete on-disk mission library."""
    if not isinstance(document, dict):
        raise ValueError('El archivo de misiones debe contener un objeto')
    version = document.get('version')
    if version != MISSION_VERSION:
        raise ValueError('Versión del archivo de misiones no compatible')
    source = document.get('missions', {})
    if not isinstance(source, dict):
        raise ValueError('La biblioteca de misiones no es válida')
    missions = {_name(name): normalize_mission(mission) for name, mission in source.items()}
    active = document.get('active_mission')
    if active is not None:
        active = _name(active)
        if active not in missions:
            active = None
    return {
        'version': MISSION_VERSION,
        'active_mission': active,
        'missions': missions,
        'draft': normalize_mission(document.get('draft', {})),
    }


def save_mission(document, name, mission):
    """Return a copy with one named mission saved and selected."""
    result = normalize_document(document)
    name = _name(name)
    result['missions'][name] = normalize_mission(mission)
    result['active_mission'] = name
    result['draft'] = deepcopy(result['missions'][name])
    return result


def rename_mission(document, current_name, new_name):
    """Rename without silently overwriting another mission."""
    result = normalize_document(document)
    current_name, new_name = _name(current_name), _name(new_name)
    if current_name not in result['missions']:
        raise ValueError('La misión seleccionada no existe')
    if new_name != current_name and new_name in result['missions']:
        raise ValueError('Ya existe una misión con ese nombre')
    result['missions'][new_name] = result['missions'].pop(current_name)
    if result['active_mission'] == current_name:
        result['active_mission'] = new_name
    return result


def delete_mission(document, name):
    """Delete a named mission while preserving the current draft."""
    result = normalize_document(document)
    name = _name(name)
    if name not in result['missions']:
        raise ValueError('La misión seleccionada no existe')
    del result['missions'][name]
    if result['active_mission'] == name:
        result['active_mission'] = next(iter(result['missions']), None)
    return result


def duplicate_step(steps, index):
    """Insert a deep copy immediately after the selected row."""
    if not isinstance(index, int) or not 0 <= index < len(steps):
        raise IndexError('No hay un paso seleccionado')
    result = deepcopy(steps)
    result.insert(index + 1, deepcopy(result[index]))
    return result, index + 1


def mission_metrics(mission):
    """Return conservative planning totals without pretending to model dynamics."""
    mission = normalize_mission(mission)
    mode = mission['plan_mode']
    steps = mission['absolute_steps'] if mode == 'absolute' else mission['relative_steps']
    distance = 0.0
    minimum_time = 0.0
    previous_x, previous_y, previous_depth = 0.0, 0.0, 0.0
    for step in steps:
        kind = step['kind']
        if kind == 'absolute_waypoint':
            horizontal = math.hypot(step['x'] - previous_x, step['y'] - previous_y)
            vertical = abs(step['depth'] - previous_depth)
            distance += horizontal
            minimum_time += horizontal / step['speed']
            minimum_time += vertical / step['vertical_speed']
            minimum_time += step['dwell']
            previous_x, previous_y, previous_depth = (
                step['x'], step['y'], step['depth']
            )
        elif kind in ('advance', 'lateral'):
            distance += abs(step['value'])
            minimum_time += abs(step['value']) / step['speed']
        elif kind == 'turn':
            minimum_time += abs(math.radians(step['value'])) / step['speed']
        elif kind == 'depth':
            minimum_time += abs(step['value'] - previous_depth) / step['speed']
            previous_depth = step['value']
        elif kind == 'wait':
            minimum_time += step['duration']
        elif kind.startswith('velocity_'):
            minimum_time += step['duration']
            if kind in ('velocity_surge', 'velocity_sway'):
                distance += abs(step['speed']) * step['duration']
    return {
        'mode': mode,
        'steps': len(steps),
        'horizontal_distance_m': distance,
        'minimum_time_s': minimum_time,
    }


def rectangle_waypoints(template, width, height):
    """Create a closed four-edge path from one validated waypoint template."""
    template = normalize_step(template)
    if template['kind'] != 'absolute_waypoint':
        raise ValueError('La plantilla del rectángulo debe ser un punto absoluto')
    width = _number(width, 'Ancho X', -5.0, 5.0)
    height = _number(height, 'Largo Y', -5.0, 5.0)
    if abs(width) < 0.01 or abs(height) < 0.01:
        raise ValueError('El rectángulo requiere lados de al menos 0.01 m')
    result = []
    for x, y in ((width, 0.0), (width, height), (0.0, height), (0.0, 0.0)):
        point = deepcopy(template)
        point.update(x=x, y=y)
        result.append(point)
    return result


class MissionStore:
    """Load and atomically save named missions plus an automatic draft."""

    def __init__(self, path=None):
        if path is None:
            configured = os.environ.get('ROBOTX_UUV_MISSIONS', '').strip()
            if configured:
                path = configured
            else:
                config_root = os.environ.get('XDG_CONFIG_HOME', '').strip()
                root = Path(config_root) if config_root else Path.home() / '.config'
                path = root / 'robotx_uuv' / 'sequence_missions.json'
        self.path = Path(path).expanduser()

    def load(self):
        """Load the library or return an empty first-run document."""
        if not self.path.exists():
            return empty_document()
        with self.path.open('r', encoding='utf-8') as stream:
            return normalize_document(json.load(stream))

    def save(self, document):
        """Atomically replace the private mission file."""
        normalized = normalize_document(document)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        descriptor, temporary = tempfile.mkstemp(
            prefix=self.path.name + '.', suffix='.tmp', dir=str(self.path.parent), text=True
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
