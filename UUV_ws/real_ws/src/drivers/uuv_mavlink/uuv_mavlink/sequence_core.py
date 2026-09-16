"""Hardware-independent sequence controller. NED velocities; surface depth in m."""

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

    def __post_init__(self):
        for value in vars(self).values():
            if number(value) <= 0:
                raise ValueError('Los límites deben ser positivos')
        if self.horizontal_speed > 0.20 or self.vertical_speed > 0.10:
            raise ValueError('Esta versión limita velocidad a 0.20/0.10 m/s')


def parse_steps(steps):
    if not isinstance(steps, list) or not 1 <= len(steps) <= 50:
        raise ValueError('La secuencia necesita entre 1 y 50 pasos')
    result = []
    for item in steps:
        if not isinstance(item, (list, tuple)) or len(item) != 2:
            raise ValueError('Paso mal formado')
        kind, value = item
        value = number(value)
        if kind not in ('advance', 'turn', 'depth', 'wait'):
            raise ValueError('Tipo de paso desconocido')
        if kind in ('depth', 'wait') and value < 0:
            raise ValueError('Profundidad y espera deben ser no negativas')
        if kind == 'turn' and abs(value) > 180:
            raise ValueError('Cada giro debe estar entre -180 y 180 grados')
        if kind == 'wait' and value > 30:
            raise ValueError('La espera máxima por paso es 30 s')
        result.append((kind, value))
    return result


def goal_for(pose, step):
    kind, value = step
    n, e, d, yaw = pose.n, pose.e, pose.depth, pose.yaw
    if kind == 'advance':
        n += value * math.cos(yaw)
        e += value * math.sin(yaw)
    elif kind == 'turn':
        yaw = wrap(yaw + math.radians(value))
    elif kind == 'depth':
        d = value
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

    def check_bounds(self, pose, anchor, margin=0.0):
        if not -margin <= pose.depth <= self.limits.max_depth + margin:
            raise ValueError('Profundidad fuera del límite configurado')
        if math.hypot(pose.n - anchor.n, pose.e - anchor.e) > self.limits.radius + margin:
            raise ValueError('Objetivo fuera del radio configurado')

    def validate(self, pose, steps, home=None):
        parsed = parse_steps(steps)
        anchor = home or pose
        self.check_bounds(pose, anchor, self.limits.depth_tolerance)
        goal = pose
        for step in parsed:
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
        self.begin_step(pose, now)

    def begin_step(self, pose, now):
        self.goal = goal_for(pose, self.steps[self.index])
        # Home has a fixed saved endpoint even if earlier steps ended with error.
        if self.home_target:
            h = self.home_target
            kind = self.steps[self.index][0]
            if kind == 'depth':
                self.goal = Pose(pose.n, pose.e, h.depth, pose.yaw)
            elif kind == 'advance':
                self.goal = Pose(h.n, h.e, h.depth, pose.yaw)
            elif kind == 'turn':
                yaw = h.yaw if self.index == len(self.steps) - 1 else math.atan2(h.e - pose.e, h.n - pose.n)
                self.goal = Pose(pose.n, pose.e, h.depth, yaw)
        self.check_bounds(self.goal, self.anchor)
        self.started = now
        self.inside_since = None

    def cancel(self, reason):
        self.state = 'CANCELLED'
        self.reason = reason
        self.steps = []
        self.goal = None
        self.inside_since = None

    def update(self, pose, now, valid=True, reason='Datos no válidos', speed=0.0):
        if self.state != 'RUNNING':
            return None
        if not valid:
            self.cancel(reason)
            return None
        try:
            self.check_bounds(pose, self.anchor, self.limits.depth_tolerance)
        except ValueError as exc:
            self.cancel(str(exc))
            return None
        if self.last_time is None or not 0 <= now - self.last_time <= 0.5:
            self.cancel('Interrupción del bucle de control')
            return None
        dt = now - self.last_time
        self.last_time = now
        if now - self.started > self.limits.step_timeout:
            self.cancel('Tiempo máximo del paso superado')
            return None
        goal = self.goal
        dn, de, dd = goal.n - pose.n, goal.e - pose.e, goal.depth - pose.depth
        dyaw = wrap(goal.yaw - pose.yaw)
        xy = math.hypot(dn, de)
        self.errors = (xy, abs(dd), abs(math.degrees(dyaw)))
        inside = (xy <= self.limits.xy_tolerance and abs(dd) <= self.limits.depth_tolerance
                  and abs(dyaw) <= self.limits.yaw_tolerance and speed <= 0.08)
        if inside:
            if self.inside_since is None:
                self.inside_since = now
            wait = self.steps[self.index][1] if self.steps[self.index][0] == 'wait' else 0
            if now - self.inside_since >= max(self.limits.dwell, wait):
                self.index += 1
                if self.index == len(self.steps):
                    self.state = 'COMPLETED'
                    self.reason = 'Secuencia completada'
                    self.steps = []
                    self.goal = None
                    return None
                try:
                    self.begin_step(pose, now)
                except ValueError as exc:
                    self.cancel(str(exc))
                return (0.0, 0.0, 0.0, pose.yaw)
        else:
            self.inside_since = None
        # Heading must be close before translating; depth steps retain XY.
        scale = min(0.5, self.limits.horizontal_speed / max(xy, 1e-9))
        vn, ve = dn * scale, de * scale
        if abs(dyaw) > math.radians(15):
            vn = ve = 0.0
        vd = max(-self.limits.vertical_speed, min(self.limits.vertical_speed, dd * 0.5))
        delta = wrap(goal.yaw - self.yaw_command)
        limit = self.limits.yaw_speed * dt
        self.yaw_command = wrap(self.yaw_command + max(-limit, min(limit, delta)))
        return (vn, ve, vd, self.yaw_command)


def return_steps(current, home):
    dn, de = home.n - current.n, home.e - current.e
    distance = math.hypot(dn, de)
    yaw = math.atan2(de, dn) if distance > 1e-6 else current.yaw
    steps = [('depth', home.depth)]
    if distance > 1e-6:
        steps += [('turn', math.degrees(wrap(yaw - current.yaw))), ('advance', distance)]
    steps += [('turn', math.degrees(wrap(home.yaw - yaw)))]
    return steps
