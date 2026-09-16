"""Read-only diagnostics and geometric previews; no actuator interface."""

from dataclasses import dataclass, replace
import math
import time


def wrap_deg(angle):
    """Shortest signed angle, including the -180/180 boundary."""
    return (angle + 180.0) % 360.0 - 180.0


def finite(value):
    if isinstance(value, (tuple, list)):
        return all(finite(item) for item in value)
    if isinstance(value, (float, int)):
        return math.isfinite(value)
    return value is not None


class Telemetry:
    """Arrival freshness only: this does not certify sensor measurement age."""

    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.samples = {}

    def put(self, key, value):
        self.samples[key] = (value, self.clock())

    def age(self, key):
        sample = self.samples.get(key)
        return self.clock() - sample[1] if sample else None

    def get(self, key, timeout=3.0):
        sample = self.samples.get(key)
        if sample is None:
            return None
        value, stamp = sample
        if not 0.0 <= self.clock() - stamp < timeout or not finite(value):
            return None
        return value


def ekf_description(flags):
    """Decode the reported EKF flags without declaring navigation ready."""
    if flags is None:
        return 'Sin flags recientes'
    parts = []
    if not flags & 1:
        parts.append('actitud no válida')
    if not flags & 2:
        parts.append('velocidad horizontal no válida')
    if not flags & (8 | 16):
        parts.append('posición horizontal no válida')
    if flags & 128:
        parts.append('posición constante')
    if flags & 1024:
        parts.append('EKF sin inicializar')
    if flags & 32768:
        parts.append('anomalía GPS')
    return '; '.join(parts) if parts else (
        'Flags horizontales presentes; precisión y DVL aún por comprobar')


@dataclass(frozen=True)
class Pose:
    north: float
    east: float
    depth: float
    yaw: float

    def __post_init__(self):
        if not all(math.isfinite(v) for v in (
                self.north, self.east, self.depth, self.yaw)):
            raise ValueError('La referencia contiene valores no finitos')
        if self.depth < 0:
            raise ValueError('La profundidad debe ser positiva hacia abajo')


def preview(start, steps):
    """Sequential ideal targets. A new preview always starts from start.

    NED yaw is clockwise positive. Depth is a separate surface reference,
    never local NED z. No timing, motion or controller execution is implied.
    """
    pose = start
    targets = []
    for kind, value in steps:
        if not math.isfinite(value):
            raise ValueError('El valor debe ser finito')
        if kind == 'advance':
            angle = math.radians(pose.yaw)
            pose = replace(pose,
                           north=pose.north + value * math.cos(angle),
                           east=pose.east + value * math.sin(angle))
        elif kind == 'turn':
            pose = replace(pose, yaw=wrap_deg(pose.yaw + value))
        elif kind == 'depth':
            if value < 0:
                raise ValueError('La profundidad no puede ser negativa')
            pose = replace(pose, depth=value)
        elif kind == 'wait':
            if value < 0:
                raise ValueError('La espera no puede ser negativa')
        else:
            raise ValueError('Paso desconocido: ' + kind)
        targets.append(pose)
    return targets


def home_steps(current, home):
    """Depth, face home, translate, then restore the saved heading."""
    dn, de = home.north - current.north, home.east - current.east
    distance = math.hypot(dn, de)
    result = [('depth', home.depth)]
    facing = current.yaw
    if distance > 1e-9:
        facing = math.degrees(math.atan2(de, dn))
        result.extend([('turn', wrap_deg(facing - current.yaw)),
                       ('advance', distance)])
    result.append(('turn', wrap_deg(home.yaw - facing)))
    return result
