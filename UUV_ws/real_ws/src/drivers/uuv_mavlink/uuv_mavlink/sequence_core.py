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
    xy_tolerance: float = 0.03
    depth_tolerance: float = 0.02
    yaw_tolerance: float = math.radians(5)
    dwell: float = 1.0
    step_timeout: float = 60.0
    timed_motion_limit: float = 30.0
    settle_speed: float = 0.03
    hold_horizontal_speed: float = 0.10
    hold_vertical_speed: float = 0.05
    position_gain: float = 0.80
    velocity_damping: float = 1.20
    recovery_radius: float = 0.08
    recovery_speed: float = 0.08
    recovery_min_speed: float = 0.020
    recovery_delay: float = 2.0
    response_ratio: float = 0.35
    velocity_filter_tau: float = 0.25

    def __post_init__(self):
        for value in vars(self).values():
            if number(value) <= 0:
                raise ValueError('Los límites deben ser positivos')
        if self.recovery_radius <= self.xy_tolerance:
            raise ValueError('El umbral de recuperación debe superar la tolerancia XY')
        if self.recovery_speed > self.horizontal_speed:
            raise ValueError('La recuperación no puede superar el máximo horizontal')
        if self.recovery_min_speed > self.recovery_speed:
            raise ValueError('El impulso mínimo no puede superar la recuperación máxima')
        if self.response_ratio > 1:
            raise ValueError('La fracción de respuesta debe estar entre 0 y 1')


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
        self.reference_target = None
        self.completed_goal = None
        self.control_mode = 'PRECISION'
        self.recovery_reason = ''
        self.recovery_active = False
        self.recovery_boost_along = False
        self.slow_since = None
        self.filtered_velocity = (0.0, 0.0, 0.0)
        self.measured_speed = 0.0
        self.command_debug = {}

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
        self.reference_target = pose
        self.completed_goal = None
        self.begin_step(pose, now)

    def begin_step(self, pose, now):
        step = self.steps[self.index]
        # Carry the commanded pose between discrete steps. A measured residual
        # must not become the requested depth, heading or lateral position.
        reference = self.reference_target or pose
        self.goal = goal_for(reference, step)
        if self.home_target:
            h = self.home_target
            kind = step['kind']
            if kind == 'depth':
                self.goal = Pose(reference.n, reference.e, h.depth, reference.yaw)
            elif kind == 'advance':
                self.goal = Pose(h.n, h.e, h.depth, reference.yaw)
            elif kind == 'turn':
                yaw = (
                    h.yaw
                    if self.index == len(self.steps) - 1
                    else math.atan2(h.e - reference.e, h.n - reference.n)
                )
                self.goal = Pose(reference.n, reference.e, h.depth, yaw)
        if step['kind'] in POSITION_KINDS:
            self.check_bounds(self.goal, self.anchor)
        self.started = now
        self.inside_since = None
        self.control_mode = 'PRECISION'
        self.recovery_reason = ''
        self.recovery_active = False
        self.recovery_boost_along = False
        self.slow_since = None
        self.filtered_velocity = (0.0, 0.0, 0.0)
        self.command_debug = {}

    def cancel(self, reason):
        self.state = 'CANCELLED'
        self.reason = reason
        self.steps = []
        self.goal = None
        self.inside_since = None
        self.reference_target = None
        self.completed_goal = None
        self.control_mode = 'PRECISION'
        self.recovery_reason = ''
        self.recovery_active = False
        self.recovery_boost_along = False
        self.slow_since = None
        self.command_debug = {}

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
        if self.slow_since is not None:
            self.slow_since += elapsed

    @staticmethod
    def _clamp(value, limit):
        return max(-limit, min(limit, value))

    def _filtered_measured_velocity(self, velocity, dt):
        if velocity is None:
            return self.filtered_velocity
        if not isinstance(velocity, (list, tuple)) or len(velocity) != 3:
            raise ValueError('La velocidad medida debe tener tres componentes')
        measured = tuple(number(value) for value in velocity)
        alpha = min(1.0, max(0.0, dt / (self.limits.velocity_filter_tau + dt)))
        self.filtered_velocity = tuple(
            previous + alpha * (current - previous)
            for previous, current in zip(self.filtered_velocity, measured)
        )
        return self.filtered_velocity

    def _pd(self, error, measured_velocity, limit, minimum=0.0):
        command = self._clamp(
            self.limits.position_gain * error
            - self.limits.velocity_damping * measured_velocity,
            limit,
        )
        # A minimum impulse is useful only far from the target and while the
        # vehicle is stopped or travelling too slowly toward it. Close to the
        # target the PD command remains free to taper to zero and brake.
        toward = measured_velocity if error > 0 else -measured_velocity
        floor = min(minimum, limit)
        if (
            floor > 0
            and abs(error) >= self.limits.recovery_radius
            and toward < floor * 0.5
            and abs(command) < floor
        ):
            command = math.copysign(floor, error)
        return command

    def _horizontal_pd(self, dn, de, vn_measured, ve_measured, limit, minimum=0.0):
        vn = self.limits.position_gain * dn - self.limits.velocity_damping * vn_measured
        ve = self.limits.position_gain * de - self.limits.velocity_damping * ve_measured
        magnitude = math.hypot(vn, ve)
        if magnitude > limit:
            scale = limit / magnitude
            vn, ve = vn * scale, ve * scale
            magnitude = limit
        distance = math.hypot(dn, de)
        toward = (
            (dn * vn_measured + de * ve_measured) / distance
            if distance > 1e-9
            else 0.0
        )
        floor = min(minimum, limit)
        if (
            floor > 0
            and distance >= self.limits.recovery_radius
            and toward < floor * 0.5
            and magnitude < floor
        ):
            vn, ve = floor * dn / distance, floor * de / distance
        return vn, ve

    def _set_recovery(self, reasons, xy, measured_speed, boost_along=False):
        if reasons:
            self.recovery_active = True
            self.recovery_boost_along = self.recovery_boost_along or boost_along
            self.recovery_reason = '; '.join(reasons)
        elif self.recovery_active:
            exit_radius = max(
                self.limits.xy_tolerance * 1.5,
                self.limits.recovery_radius * 0.6,
            )
            if xy <= exit_radius and measured_speed <= self.limits.settle_speed:
                self.recovery_active = False
                self.recovery_boost_along = False
                self.recovery_reason = ''
        self.control_mode = 'RECOVERY' if self.recovery_active else 'PRECISION'

    def _advance_step(self, pose, now):
        was_timed = self.timed
        # Timed commands intentionally do not specify a final position. Only
        # their measured endpoint may seed a subsequent positional command.
        self.reference_target = pose if was_timed else self.goal
        self.index += 1
        if self.index == len(self.steps):
            self.state = 'COMPLETED'
            self.reason = 'Secuencia completada'
            self.steps = []
            self.completed_goal = None if was_timed else self.reference_target
            self.goal = None
            return None
        self.begin_step(pose, now)
        return (0.0, 0.0, 0.0, self.yaw_command)

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

    def update(
        self,
        pose,
        now,
        valid=True,
        reason='Datos no válidos',
        speed=0.0,
        velocity=None,
    ):
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
        try:
            filtered_velocity = self._filtered_measured_velocity(velocity, dt)
        except ValueError as exc:
            self.cancel(str(exc))
            return None
        measured_speed = (
            math.sqrt(sum(number(value) ** 2 for value in velocity))
            if velocity is not None
            else number(speed)
        )
        self.measured_speed = measured_speed
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
            and measured_speed <= self.limits.settle_speed
        )
        if inside:
            if self.inside_since is None:
                self.inside_since = now
            wait = step['duration'] if step['kind'] == 'wait' else 0
            if now - self.inside_since >= max(self.limits.dwell, wait):
                return self._advance_step(pose, now)
        else:
            self.inside_since = None
        hold_limit = min(self.limits.hold_horizontal_speed, self.limits.horizontal_speed)
        vn_measured, ve_measured, vd_measured = filtered_velocity
        if step['kind'] in ('advance', 'lateral'):
            ux, uy = math.cos(goal.yaw), math.sin(goal.yaw)
            if step['kind'] == 'lateral':
                ux, uy = -uy, ux
            along_error = dn * ux + de * uy
            cross_error = -dn * uy + de * ux
            along_velocity = vn_measured * ux + ve_measured * uy
            cross_velocity = -vn_measured * uy + ve_measured * ux
            direction = 1.0 if step['value'] >= 0 else -1.0
            remaining = direction * along_error
            toward_velocity = direction * along_velocity
            slow_threshold = self.limits.response_ratio * step['speed']
            monitoring_response = (
                remaining > self.limits.recovery_radius
                and abs(dyaw) <= math.radians(15)
            )
            if monitoring_response and toward_velocity < slow_threshold:
                if self.slow_since is None:
                    self.slow_since = now
            else:
                self.slow_since = None
            slow_response = (
                self.slow_since is not None
                and now - self.slow_since >= self.limits.recovery_delay
            )
            reasons = []
            if abs(cross_error) >= self.limits.recovery_radius:
                reasons.append('deriva transversal fuera del umbral')
            if remaining <= -self.limits.recovery_radius:
                reasons.append('sobrepasó el objetivo')
            if slow_response:
                reasons.append('respuesta más lenta de la esperada')
            self._set_recovery(
                reasons,
                xy,
                measured_speed,
                boost_along=slow_response or remaining <= -self.limits.recovery_radius,
            )
            cross_limit = self.limits.recovery_speed if self.recovery_active else hold_limit
            minimum = self.limits.recovery_min_speed if self.recovery_active else 0.0
            cross = self._pd(cross_error, cross_velocity, cross_limit, minimum)
            requested_limit = (
                self.limits.recovery_speed
                if self.recovery_active and self.recovery_boost_along
                else step['speed']
            )
            # Reserve speed for cross-track recovery before the travel axis.
            # The magnitude, not merely each component, stays below the larger
            # of the active travel/recovery limits and the global hard limit.
            total_limit = min(
                self.limits.horizontal_speed,
                max(requested_limit, cross_limit),
            )
            budget = math.sqrt(max(0.0, total_limit**2 - cross**2))
            along_limit = min(requested_limit, budget)
            along_minimum = minimum if self.recovery_boost_along else 0.0
            along = self._pd(along_error, along_velocity, along_limit, along_minimum)
            if abs(dyaw) > math.radians(15):
                along = 0.0
            vn, ve = along * ux - cross * uy, along * uy + cross * ux
            self.command_debug = {
                'along_error': along_error,
                'cross_error': cross_error,
                'toward_velocity': toward_velocity,
                'along_limit': along_limit,
                'cross_limit': cross_limit,
            }
        else:
            # Turns, depth changes and waits still restore horizontal position,
            # including while the yaw controller is aligning the vehicle.
            reasons = ['posición fuera del umbral'] if xy >= self.limits.recovery_radius else []
            self._set_recovery(reasons, xy, measured_speed)
            horizontal_limit = (
                self.limits.recovery_speed if self.recovery_active else hold_limit
            )
            minimum = self.limits.recovery_min_speed if self.recovery_active else 0.0
            vn, ve = self._horizontal_pd(
                dn,
                de,
                vn_measured,
                ve_measured,
                horizontal_limit,
                minimum,
            )
            self.command_debug = {
                'horizontal_limit': horizontal_limit,
                'north_error': dn,
                'east_error': de,
            }
        vertical_limit = (
            step['speed'] if step['kind'] == 'depth'
            else min(self.limits.vertical_speed, self.limits.hold_vertical_speed)
        )
        vd = self._pd(dd, vd_measured, vertical_limit)
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
