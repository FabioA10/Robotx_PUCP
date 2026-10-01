"""Hardware-independent mixed sequence controller; NED velocities."""

from dataclasses import dataclass
import math


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def number(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError('Se requiere un número finito')
    return float(value)


@dataclass(frozen=True)
class Pose:
    n: float
    e: float
    depth: float
    yaw: float

    def __post_init__(self):
        for value in (self.n, self.e, self.depth, self.yaw):
            number(value)


@dataclass
class Limits:
    max_depth: float = 1.0
    radius: float = 2.0
    horizontal_speed: float = 0.20
    vertical_speed: float = 0.10
    yaw_speed: float = math.radians(10)
    xy_tolerance: float = 0.25
    depth_tolerance: float = 0.15
    yaw_tolerance: float = math.radians(5)
    dwell: float = 1.0
    step_timeout: float = 60.0
    timed_motion_limit: float = 30.0

    def __post_init__(self):
        for value in vars(self).values():
            if number(value) <= 0:
                raise ValueError('Los límites deben ser positivos')


POSITION_KINDS = {'advance', 'lateral', 'turn', 'depth'}
TIMED_KINDS = {'velocity_surge', 'velocity_sway', 'velocity_heave', 'velocity_yaw'}
ALL_KINDS = POSITION_KINDS | TIMED_KINDS | {'wait'}


def _legacy_step(item):
    """Accept the previous [kind, value] format during migration."""
    if not isinstance(item, (list, tuple)) or len(item) != 2:
        return None
    kind, value = item
    if kind == 'wait':
        return {'kind': 'wait', 'duration': value}
    return {'kind': kind, 'value': value}


def parse_steps(steps, limits=None):
    """Normalize old and new queue entries and enforce motion limits."""
    limits = limits or Limits()
    if not isinstance(steps, list) or not 1 <= len(steps) <= 50:
        raise ValueError('La secuencia necesita entre 1 y 50 pasos')
    result = []
    for raw in steps:
        item = dict(raw) if isinstance(raw, dict) else _legacy_step(raw)
        if item is None:
            raise ValueError('Paso mal formado')
        kind = item.get('kind')
        if kind not in ALL_KINDS:
            raise ValueError('Tipo de paso desconocido')
        normalized = {'kind': kind}
        if kind == 'wait':
            duration = number(item.get('duration'))
            if not 0 <= duration <= limits.timed_motion_limit:
                raise ValueError('La espera está fuera del rango permitido')
            normalized['duration'] = duration
        elif kind in TIMED_KINDS:
            speed = number(item.get('speed'))
            duration = number(item.get('duration'))
            speed_limit = (
                limits.yaw_speed
                if kind == 'velocity_yaw'
                else (
                    limits.vertical_speed if kind == 'velocity_heave' else limits.horizontal_speed
                )
            )
            if abs(speed) > speed_limit + 1e-9:
                unit = 'rad/s' if kind == 'velocity_yaw' else 'm/s'
                raise ValueError(f'Velocidad temporal fuera del límite ({speed_limit:g} {unit})')
            if not 0 < duration <= limits.timed_motion_limit:
                raise ValueError('La duración debe ser mayor que cero y no superar 30 s')
            normalized.update(speed=speed, duration=duration)
        else:
            value = number(item.get('value'))
            if kind == 'turn' and abs(value) > 180:
                raise ValueError('Cada giro debe estar entre -180 y 180 grados')
            if kind == 'depth' and value < 0:
                raise ValueError('La profundidad debe ser no negativa')
            default_speed = (
                limits.yaw_speed
                if kind == 'turn'
                else limits.vertical_speed
                if kind == 'depth'
                else limits.horizontal_speed
            )
            speed = abs(number(item.get('speed', default_speed)))
            if speed <= 0 or speed > default_speed + 1e-9:
                raise ValueError('Velocidad máxima del paso fuera de límites')
            normalized.update(value=value, speed=speed)
        result.append(normalized)
    return result


def goal_for(pose, step):
    kind = step['kind']
    n, e, d, yaw = pose.n, pose.e, pose.depth, pose.yaw
    if kind == 'advance':
        n += step['value'] * math.cos(yaw)
        e += step['value'] * math.sin(yaw)
    elif kind == 'lateral':
        n -= step['value'] * math.sin(yaw)
        e += step['value'] * math.cos(yaw)
    elif kind == 'turn':
        yaw = wrap(yaw + math.radians(step['value']))
    elif kind == 'depth':
        d = step['value']
    return Pose(n, e, d, yaw)


class Sequence:
    def __init__(self, limits=None):
        self.limits = limits or Limits()
        self.state = 'IDLE'
        self.reason = 'Sin ejecución'
        self.steps = []
        self.index = 0
        self.goal = None
        self.anchor = None
        self.started = None
        self.inside_since = None
        self.yaw_command = 0.0
        self.last_time = None
        self.errors = (0.0, 0.0, 0.0)
        self.home_target = None
        self.last_pose = None

    @property
    def current_step(self):
        return self.steps[self.index] if self.state == 'RUNNING' else None

    @property
    def timed(self):
        return bool(self.current_step and self.current_step['kind'] in TIMED_KINDS)

    def check_bounds(self, pose, anchor, margin=0.0):
        if not -margin <= pose.depth <= self.limits.max_depth + margin:
            raise ValueError('Profundidad fuera del límite configurado')
        if math.hypot(pose.n - anchor.n, pose.e - anchor.e) > self.limits.radius + margin:
            raise ValueError('Objetivo fuera del radio configurado')

    def validate(self, pose, steps, home=None):
        parsed = parse_steps(steps, self.limits)
        anchor = home or pose
        self.check_bounds(pose, anchor, self.limits.depth_tolerance)
        goal = pose
        for step in parsed:
            if step['kind'] in POSITION_KINDS:
                goal = goal_for(goal, step)
                self.check_bounds(goal, anchor)
        return parsed

    def start(self, pose, steps, now, home=None, returning=False):
        if self.state == 'RUNNING':
            raise ValueError('Ya hay una secuencia activa')
        parsed = self.validate(pose, steps, home)
        self.steps = parsed
        self.anchor = home or pose
        self.home_target = home if returning else None
        self.index = 0
        self.state = 'RUNNING'
        self.reason = 'Ejecutando'
        self.yaw_command = pose.yaw
        self.last_time = now
        self.last_pose = pose
        self.begin_step(pose, now)

    def begin_step(self, pose, now):
        step = self.steps[self.index]
        self.goal = goal_for(pose, step)
        if self.home_target:
            h = self.home_target
            kind = step['kind']
            if kind == 'depth':
                self.goal = Pose(pose.n, pose.e, h.depth, pose.yaw)
            elif kind == 'advance':
                self.goal = Pose(h.n, h.e, h.depth, pose.yaw)
            elif kind == 'turn':
                yaw = (
                    h.yaw
                    if self.index == len(self.steps) - 1
                    else math.atan2(h.e - pose.e, h.n - pose.n)
                )
                self.goal = Pose(pose.n, pose.e, h.depth, yaw)
        if step['kind'] in POSITION_KINDS:
            self.check_bounds(self.goal, self.anchor)
        self.started = now
        self.inside_since = None

    def cancel(self, reason):
        self.state = 'CANCELLED'
        self.reason = reason
        self.steps = []
        self.goal = None
        self.inside_since = None

    def hold(self, now):
        if self.state != 'RUNNING':
            return
        now = number(now)
        if self.last_time is not None:
            elapsed = max(0.0, now - self.last_time)
            if self.started is not None:
                self.started += elapsed
        self.last_time = now
        self.inside_since = None

    def _advance_step(self, pose, now):
        self.index += 1
        if self.index == len(self.steps):
            self.state = 'COMPLETED'
            self.reason = 'Secuencia completada'
            self.steps = []
            self.goal = None
            return None
        self.begin_step(pose, now)
        return (0.0, 0.0, 0.0, pose.yaw)

    def _timed_command(self, step, pose, dt):
        kind, speed = step['kind'], step['speed']
        vn = ve = vd = 0.0
        if kind == 'velocity_surge':
            vn, ve = speed * math.cos(pose.yaw), speed * math.sin(pose.yaw)
        elif kind == 'velocity_sway':
            vn, ve = -speed * math.sin(pose.yaw), speed * math.cos(pose.yaw)
        elif kind == 'velocity_heave':
            vd = speed
        elif kind == 'velocity_yaw':
            self.yaw_command = wrap(self.yaw_command + speed * dt)
        return (vn, ve, vd, self.yaw_command)

    def update(self, pose, now, valid=True, reason='Datos no válidos', speed=0.0):
        if self.state != 'RUNNING':
            return None
        if not valid:
            self.cancel(reason)
            return None
        if pose is not None:
            self.last_pose = pose
        pose = pose or self.last_pose
        if pose is None:
            self.cancel('No existe una referencia para el control')
            return None
        if self.last_time is None or not 0 <= now - self.last_time <= 0.5:
            self.cancel('Interrupción del bucle de control')
            return None
        dt = now - self.last_time
        self.last_time = now
        step = self.current_step
        if step['kind'] in TIMED_KINDS:
            if now - self.started >= step['duration']:
                return self._advance_step(pose, now)
            return self._timed_command(step, pose, dt)
        try:
            self.check_bounds(pose, self.anchor, self.limits.depth_tolerance)
        except ValueError as exc:
            self.cancel(str(exc))
            return None
        if now - self.started > self.limits.step_timeout:
            self.cancel('Tiempo máximo del paso superado')
            return None
        goal = self.goal
        dn, de, dd = goal.n - pose.n, goal.e - pose.e, goal.depth - pose.depth
        dyaw = wrap(goal.yaw - pose.yaw)
        xy = math.hypot(dn, de)
        self.errors = (xy, abs(dd), abs(math.degrees(dyaw)))
        inside = (
            xy <= self.limits.xy_tolerance
            and abs(dd) <= self.limits.depth_tolerance
            and abs(dyaw) <= self.limits.yaw_tolerance
            and speed <= 0.08
        )
        if inside:
            if self.inside_since is None:
                self.inside_since = now
            wait = step['duration'] if step['kind'] == 'wait' else 0
            if now - self.inside_since >= max(self.limits.dwell, wait):
                return self._advance_step(pose, now)
        else:
            self.inside_since = None
        max_horizontal = (
            step['speed']
            if step['kind'] in ('advance', 'lateral')
            else self.limits.horizontal_speed
        )
        scale = min(0.5, max_horizontal / max(xy, 1e-9))
        vn, ve = dn * scale, de * scale
        if abs(dyaw) > math.radians(15):
            vn = ve = 0.0
        vertical_limit = step['speed'] if step['kind'] == 'depth' else self.limits.vertical_speed
        vd = max(-vertical_limit, min(vertical_limit, dd * 0.5))
        delta = wrap(goal.yaw - self.yaw_command)
        yaw_speed = step['speed'] if step['kind'] == 'turn' else self.limits.yaw_speed
        yaw_limit = yaw_speed * dt
        self.yaw_command = wrap(self.yaw_command + max(-yaw_limit, min(yaw_limit, delta)))
        return (vn, ve, vd, self.yaw_command)


def return_steps(current, home):
    dn, de = home.n - current.n, home.e - current.e
    distance = math.hypot(dn, de)
    yaw = math.atan2(de, dn) if distance > 1e-6 else current.yaw
    steps = [{'kind': 'depth', 'value': home.depth}]
    if distance > 1e-6:
        steps += [
            {'kind': 'turn', 'value': math.degrees(wrap(yaw - current.yaw))},
            {'kind': 'advance', 'value': distance},
        ]
    steps += [{'kind': 'turn', 'value': math.degrees(wrap(home.yaw - yaw))}]
    return steps
