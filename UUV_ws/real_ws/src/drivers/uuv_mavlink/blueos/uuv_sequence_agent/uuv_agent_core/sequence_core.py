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
    wait_limit: float = 30.0
    timed_motion_limit: float = 5.0
    timed_braking_time: float = 1.0
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
    waypoint_timeout: float = 120.0
    waypoint_horizontal_accel: float = 0.12
    waypoint_horizontal_decel: float = 0.30
    waypoint_vertical_accel: float = 0.08
    recovery_progress: float = 0.02
    waypoint_recovery_persistence: float = 0.75
    waypoint_transition_grace: float = 0.35
    waypoint_turn_grace: float = 1.50
    waypoint_corridor_min: float = 0.10
    waypoint_corridor_scale: float = 1.50
    waypoint_cross_damping_ratio: float = 0.35
    waypoint_response_delay: float = 6.00
    waypoint_soft_corridor_min: float = 0.04
    waypoint_gate_margin_ratio: float = 0.80
    waypoint_recovery_speed_cap: float = 0.18
    waypoint_corner_speed: float = 0.03
    waypoint_transition_cross_speed: float = 0.03
    waypoint_straight_angle_deg: float = 15.0

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
        if self.waypoint_horizontal_decel < self.waypoint_horizontal_accel:
            raise ValueError('La desaceleración del punto debe superar la aceleración')
        if self.recovery_progress >= self.recovery_radius:
            raise ValueError('El progreso mínimo debe ser menor al radio de recuperación')
        if self.waypoint_cross_damping_ratio > 1:
            raise ValueError('La fracción de amortiguamiento transversal debe ser ≤ 1')
        if self.waypoint_gate_margin_ratio > 1:
            raise ValueError('El margen utilizable de la caja debe ser ≤ 1')
        if self.waypoint_straight_angle_deg >= 90:
            raise ValueError('El umbral de tramo recto debe ser menor de 90 grados')


POSITION_KINDS = {'advance', 'lateral', 'turn', 'depth'}
TIMED_KINDS = {'velocity_surge', 'velocity_sway', 'velocity_heave', 'velocity_yaw'}
ABSOLUTE_KINDS = {'absolute_waypoint'}
ALL_KINDS = POSITION_KINDS | TIMED_KINDS | ABSOLUTE_KINDS | {'wait'}


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
        if kind == 'absolute_waypoint':
            x = number(item.get('x'))
            y = number(item.get('y'))
            depth = number(item.get('depth'))
            yaw_deg = number(
                item['yaw_deg']
                if 'yaw_deg' in item
                else math.degrees(item.get('yaw'))
            )
            speed = abs(number(item.get('speed', min(0.15, limits.horizontal_speed))))
            vertical_speed = abs(
                number(item.get('vertical_speed', min(0.08, limits.vertical_speed)))
            )
            yaw_speed_deg = abs(
                number(
                    item['yaw_speed_deg']
                    if 'yaw_speed_deg' in item
                    else (
                        math.degrees(item['yaw_speed'])
                        if 'yaw_speed' in item
                        else min(10.0, math.degrees(limits.yaw_speed))
                    )
                )
            )
            tolerance_x = number(item.get('tolerance_x', 0.04))
            tolerance_y = number(item.get('tolerance_y', 0.04))
            tolerance_xy = number(item.get('tolerance_xy', limits.xy_tolerance))
            tolerance_depth = number(item.get('tolerance_depth', limits.depth_tolerance))
            tolerance_yaw_deg = number(
                item['tolerance_yaw_deg']
                if 'tolerance_yaw_deg' in item
                else (
                    math.degrees(item['tolerance_yaw'])
                    if 'tolerance_yaw' in item
                    else math.degrees(limits.yaw_tolerance)
                )
            )
            dwell = number(item.get('dwell', limits.dwell))
            if math.hypot(x, y) > limits.radius + 1e-9:
                raise ValueError('El punto absoluto supera el radio de la sesión')
            if not 0 <= depth <= limits.max_depth:
                raise ValueError('La profundidad del punto absoluto está fuera del límite')
            if not -180 <= yaw_deg <= 180:
                raise ValueError('El rumbo absoluto debe estar entre -180 y 180 grados')
            if not 0 < speed <= limits.horizontal_speed:
                raise ValueError('La velocidad XY del punto supera el límite horizontal')
            if not 0 < vertical_speed <= limits.vertical_speed:
                raise ValueError('La velocidad vertical del punto supera el límite vertical')
            if not 0 < yaw_speed_deg <= math.degrees(limits.yaw_speed) + 1e-9:
                raise ValueError('La velocidad angular del punto supera el límite de giro')
            if not 0.01 <= tolerance_x <= 0.15 or not 0.01 <= tolerance_y <= 0.15:
                raise ValueError('Las tolerancias X/Y deben estar entre 0.01 y 0.15 m')
            if not 0.01 <= tolerance_xy <= 0.15:
                raise ValueError('La tolerancia radial XY debe estar entre 0.01 y 0.15 m')
            if not 0.005 <= tolerance_depth <= 0.10:
                raise ValueError('La tolerancia de profundidad debe estar entre 0.005 y 0.10 m')
            if not 1.0 <= tolerance_yaw_deg <= 20.0:
                raise ValueError('La tolerancia angular debe estar entre 1 y 20 grados')
            if not 0.5 <= dwell <= 5.0:
                raise ValueError('La confirmación estable debe estar entre 0.5 y 5 s')
            normalized.update(
                x=x,
                y=y,
                depth=depth,
                yaw=math.radians(yaw_deg),
                speed=speed,
                vertical_speed=vertical_speed,
                yaw_speed=math.radians(yaw_speed_deg),
                tolerance_x=tolerance_x,
                tolerance_y=tolerance_y,
                tolerance_xy=tolerance_xy,
                tolerance_depth=tolerance_depth,
                tolerance_yaw=math.radians(tolerance_yaw_deg),
                dwell=dwell,
            )
        elif kind == 'wait':
            duration = number(item.get('duration'))
            if not 0 <= duration <= limits.wait_limit:
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
            if abs(speed) < 1e-9:
                raise ValueError('La velocidad temporal no puede ser cero')
            if not 0 < duration <= limits.timed_motion_limit:
                raise ValueError(
                    'La duración debe ser mayor que cero y no superar '
                    f'{limits.timed_motion_limit:g} s'
                )
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


def absolute_goal(anchor, step):
    """Convert mission-frame X/Y/depth/yaw into the fixed local NED frame."""
    x, y = step['x'], step['y']
    cosine, sine = math.cos(anchor.yaw), math.sin(anchor.yaw)
    return Pose(
        anchor.n + x * cosine - y * sine,
        anchor.e + x * sine + y * cosine,
        step['depth'],
        wrap(anchor.yaw + step['yaw']),
    )


def projected_timed_goal(pose, step, braking_time):
    """Return a conservative endpoint including one braking-time margin."""
    kind = step['kind']
    speed = step['speed']
    duration = step['duration']
    travel_time = duration + braking_time
    n, e, depth, yaw = pose.n, pose.e, pose.depth, pose.yaw
    if kind == 'velocity_surge':
        distance = speed * travel_time
        n += distance * math.cos(yaw)
        e += distance * math.sin(yaw)
    elif kind == 'velocity_sway':
        distance = speed * travel_time
        n -= distance * math.sin(yaw)
        e += distance * math.cos(yaw)
    elif kind == 'velocity_heave':
        depth += speed * travel_time
    elif kind == 'velocity_yaw':
        yaw = wrap(yaw + speed * duration)
    return Pose(n, e, depth, yaw)


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
        self.waypoint_phase = None
        self.waypoint_hold = None
        self.waypoint_transit_yaw = None
        self.waypoint_translation_unit = None
        self.waypoint_braking = False
        self.waypoint_progress_history = []
        self.waypoint_recovery_clear_since = None
        self.waypoint_fault_since = None
        self.waypoint_fault_reason = ''
        self.waypoint_transition_grace_until = None
        self.waypoint_transition_angle_deg = None
        self.waypoint_recovery_corridor = self.limits.waypoint_corridor_min
        self.waypoint_previous_error = None
        self.last_linear_command = (0.0, 0.0, 0.0)

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
            if step['kind'] in ABSOLUTE_KINDS:
                goal = absolute_goal(anchor, step)
                self.check_bounds(goal, anchor)
            elif step['kind'] in POSITION_KINDS:
                goal = goal_for(goal, step)
                self.check_bounds(goal, anchor)
            elif step['kind'] in TIMED_KINDS:
                goal = projected_timed_goal(
                    goal,
                    step,
                    self.limits.timed_braking_time,
                )
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

    def begin_step(self, pose, now, preserve_motion=False):
        previous_velocity = self.filtered_velocity
        previous_command = self.last_linear_command
        previous_translation_unit = self.waypoint_translation_unit
        step = self.steps[self.index]
        # Carry the commanded pose between discrete steps. A measured residual
        # must not become the requested depth, heading or lateral position.
        reference = self.reference_target or pose
        self.goal = (
            absolute_goal(self.anchor, step)
            if step['kind'] in ABSOLUTE_KINDS
            else goal_for(reference, step)
        )
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
        if step['kind'] in POSITION_KINDS | ABSOLUTE_KINDS:
            self.check_bounds(self.goal, self.anchor)
        self.started = now
        self.inside_since = None
        self.control_mode = 'PRECISION'
        self.recovery_reason = ''
        self.recovery_active = False
        self.recovery_boost_along = False
        self.slow_since = None
        self.filtered_velocity = previous_velocity if preserve_motion else (0.0, 0.0, 0.0)
        self.command_debug = {}
        self.waypoint_phase = None
        self.waypoint_hold = None
        self.waypoint_transit_yaw = None
        self.waypoint_translation_unit = None
        self.waypoint_braking = False
        self.waypoint_progress_history = []
        self.waypoint_recovery_clear_since = None
        self.waypoint_fault_since = None
        self.waypoint_fault_reason = ''
        self.waypoint_transition_grace_until = None
        self.waypoint_transition_angle_deg = None
        self.waypoint_recovery_corridor = self.limits.waypoint_corridor_min
        self.waypoint_previous_error = None
        self.last_linear_command = (
            previous_command if preserve_motion else (0.0, 0.0, 0.0)
        )
        if step['kind'] in ABSOLUTE_KINDS:
            self._select_waypoint_phase(
                pose,
                now,
                preserve_motion=preserve_motion,
                incoming_unit=previous_translation_unit,
            )

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
        self.waypoint_phase = None
        self.waypoint_hold = None
        self.waypoint_transit_yaw = None
        self.waypoint_translation_unit = None
        self.waypoint_braking = False
        self.waypoint_progress_history = []
        self.waypoint_recovery_clear_since = None
        self.waypoint_fault_since = None
        self.waypoint_fault_reason = ''
        self.waypoint_transition_grace_until = None
        self.waypoint_transition_angle_deg = None
        self.waypoint_recovery_corridor = self.limits.waypoint_corridor_min
        self.waypoint_previous_error = None
        self.last_linear_command = (0.0, 0.0, 0.0)

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

    def _horizontal_pd(
        self,
        dn,
        de,
        vn_measured,
        ve_measured,
        limit,
        minimum=0.0,
        damping=None,
    ):
        damping = self.limits.velocity_damping if damping is None else number(damping)
        vn = self.limits.position_gain * dn - damping * vn_measured
        ve = self.limits.position_gain * de - damping * ve_measured
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

    def _set_absolute_recovery(
        self,
        reasons,
        now,
        cross_error,
        radial_toward_velocity,
        recovery_corridor,
        boost_along=False,
    ):
        """Latch real absolute faults, but release after measured progress resumes."""
        if reasons:
            self.recovery_active = True
            self.recovery_boost_along = self.recovery_boost_along or boost_along
            self.recovery_reason = '; '.join(reasons)
            self.waypoint_recovery_clear_since = None
            return
        if not self.recovery_active:
            self.waypoint_recovery_clear_since = None
            return
        healthy = (
            abs(cross_error) <= recovery_corridor * 0.65
            and radial_toward_velocity >= 0.0
        )
        if not healthy:
            self.waypoint_recovery_clear_since = None
            return
        if self.waypoint_recovery_clear_since is None:
            self.waypoint_recovery_clear_since = now
        elif now - self.waypoint_recovery_clear_since >= 0.5:
            self.recovery_active = False
            self.recovery_boost_along = False
            self.recovery_reason = ''
            self.waypoint_recovery_clear_since = None

    def _persistent_absolute_faults(self, reasons, now):
        """Confirm a waypoint fault only when it survives sensor/turn transients."""
        if not reasons:
            self.waypoint_fault_since = None
            self.waypoint_fault_reason = ''
            return []
        self.waypoint_fault_reason = '; '.join(reasons)
        if self.waypoint_fault_since is None:
            self.waypoint_fault_since = now
            return []
        if now - self.waypoint_fault_since < self.limits.waypoint_recovery_persistence:
            return []
        return reasons

    def _waypoint_progress(self, now, along_remaining):
        """Measure real progress on the segment, ignoring sideways motion."""
        self.waypoint_progress_history.append((now, along_remaining))
        cutoff = now - self.limits.recovery_delay
        while (
            len(self.waypoint_progress_history) > 1
            and self.waypoint_progress_history[1][0] <= cutoff
        ):
            self.waypoint_progress_history.pop(0)
        started, initial_remaining = self.waypoint_progress_history[0]
        return initial_remaining - along_remaining, now - started

    def _mission_errors(self, pose):
        """Return goal error expressed in the fixed mission X/Y axes."""
        dn, de = self.goal.n - pose.n, self.goal.e - pose.e
        cosine, sine = math.cos(self.anchor.yaw), math.sin(self.anchor.yaw)
        return (
            dn * cosine + de * sine,
            -dn * sine + de * cosine,
            self.goal.depth - pose.depth,
            wrap(self.goal.yaw - pose.yaw),
        )

    def _enter_waypoint_phase(
        self,
        phase,
        pose,
        now,
        preserve_motion=False,
        incoming_unit=None,
    ):
        self.waypoint_phase = phase
        self.inside_since = None
        self.slow_since = None
        self.recovery_active = False
        self.recovery_boost_along = False
        self.recovery_reason = ''
        self.control_mode = 'ABSOLUTE_' + phase
        self.waypoint_hold = None
        self.waypoint_transit_yaw = self.goal.yaw
        dn, de = self.goal.n - pose.n, self.goal.e - pose.e
        distance = math.hypot(dn, de)
        self.waypoint_translation_unit = (
            (dn / distance, de / distance)
            if distance > 1e-9
            else None
        )
        self.waypoint_progress_history = []
        self.waypoint_recovery_clear_since = None
        self.waypoint_fault_since = None
        self.waypoint_fault_reason = ''
        self.waypoint_transition_angle_deg = None
        grace = 0.0
        if preserve_motion and incoming_unit and self.waypoint_translation_unit:
            cosine = max(
                -1.0,
                min(
                    1.0,
                    sum(
                        first * second
                        for first, second in zip(
                            incoming_unit,
                            self.waypoint_translation_unit,
                        )
                    ),
                ),
            )
            self.waypoint_transition_angle_deg = math.degrees(math.acos(cosine))
            grace = (
                self.limits.waypoint_transition_grace
                + self.limits.waypoint_turn_grace
                * self.waypoint_transition_angle_deg / 180.0
            )
        self.waypoint_transition_grace_until = now + grace
        self.waypoint_braking = False
        self.waypoint_previous_error = self._mission_errors(pose)
        self.command_debug = {'waypoint_phase': phase, 'phase_started': now}

    def _select_waypoint_phase(
        self,
        pose,
        now,
        preserve_motion=False,
        incoming_unit=None,
    ):
        self._enter_waypoint_phase(
            'TRACK',
            pose,
            now,
            preserve_motion=preserve_motion,
            incoming_unit=incoming_unit,
        )
        self.yaw_command = pose.yaw

    @staticmethod
    def _segment_hits_box(previous, current, tolerances):
        """Return whether one sampled 4-D segment crosses XYZ/yaw tolerance."""
        start = list(previous)
        end = list(current)
        end[3] = start[3] + wrap(end[3] - start[3])
        low, high = 0.0, 1.0
        for first, last, tolerance in zip(start, end, tolerances):
            delta = last - first
            if abs(delta) <= 1e-12:
                if abs(first) > tolerance:
                    return False
                continue
            enter = (-tolerance - first) / delta
            leave = (tolerance - first) / delta
            if enter > leave:
                enter, leave = leave, enter
            low = max(low, enter)
            high = min(high, leave)
            if low > high:
                return False
        return high >= 0.0 and low <= 1.0

    def _next_absolute_goal(self):
        next_index = self.index + 1
        if next_index >= len(self.steps):
            return None, None
        next_step = self.steps[next_index]
        if next_step['kind'] not in ABSOLUTE_KINDS:
            return None, None
        return absolute_goal(self.anchor, next_step), next_step

    def _corner_plan(self, step):
        """Return look-ahead speed for a straight, corner or terminal waypoint."""
        next_goal, next_step = self._next_absolute_goal()
        if next_goal is None or self.waypoint_translation_unit is None:
            return 0.0, None
        outgoing_n = next_goal.n - self.goal.n
        outgoing_e = next_goal.e - self.goal.e
        outgoing_length = math.hypot(outgoing_n, outgoing_e)
        if outgoing_length <= 1e-9:
            return 0.0, None
        incoming_n, incoming_e = self.waypoint_translation_unit
        outgoing_n /= outgoing_length
        outgoing_e /= outgoing_length
        cosine = max(
            -1.0,
            min(1.0, incoming_n * outgoing_n + incoming_e * outgoing_e),
        )
        angle = math.acos(cosine)
        # Preserve speed on an almost straight route, but make sharp corners
        # genuinely slow.  The V5 half-speed command at 90 degrees left too
        # much transverse momentum for the following segment.
        factor = max(0.0, cosine) ** 2
        return min(step['speed'], next_step['speed']) * factor, math.degrees(angle)

    def _transition_velocity(self, step, filtered_velocity):
        """Describe whether one sample can safely flow into the next segment."""
        next_goal, _ = self._next_absolute_goal()
        corner_speed, turn_angle_deg = self._corner_plan(step)
        result = {
            'corner_speed': corner_speed,
            'turn_angle_deg': turn_angle_deg,
            'outgoing_along_velocity': None,
            'outgoing_cross_velocity': None,
            'transition_speed_limit': min(
                step['speed'],
                self.limits.waypoint_transition_cross_speed,
            ),
            'transition_velocity_ready': True,
        }
        if (
            next_goal is None
            or turn_angle_deg is None
            or turn_angle_deg <= self.limits.waypoint_straight_angle_deg
        ):
            return result
        outgoing_n = next_goal.n - self.goal.n
        outgoing_e = next_goal.e - self.goal.e
        outgoing_length = math.hypot(outgoing_n, outgoing_e)
        if outgoing_length <= 1e-9:
            return result
        outgoing_n /= outgoing_length
        outgoing_e /= outgoing_length
        vn_measured, ve_measured, _ = filtered_velocity
        outgoing_along = (
            vn_measured * outgoing_n + ve_measured * outgoing_e
        )
        outgoing_cross = (
            -vn_measured * outgoing_e + ve_measured * outgoing_n
        )
        limit = result['transition_speed_limit']
        result.update(
            outgoing_along_velocity=outgoing_along,
            outgoing_cross_velocity=outgoing_cross,
            transition_velocity_ready=(
                abs(outgoing_cross) <= limit
                and outgoing_along >= -limit
            ),
        )
        return result

    def _absolute_recovery_corridor(self, step):
        """Return a corridor compatible with the waypoint acceptance box."""
        box_allowance = self.limits.waypoint_corridor_scale * max(
            step['tolerance_x'],
            step['tolerance_y'],
        )
        return min(
            self.limits.radius,
            max(
                self.limits.recovery_radius,
                self.limits.waypoint_corridor_min,
                box_allowance,
            ),
        )

    def _absolute_gate_allowance(self, step):
        """Project the mission-frame tolerance box onto segment axes."""
        if self.waypoint_translation_unit is None:
            return step['tolerance_x'], step['tolerance_y']
        ux, uy = self.waypoint_translation_unit
        cosine, sine = math.cos(self.anchor.yaw), math.sin(self.anchor.yaw)
        along_x = ux * cosine + uy * sine
        along_y = -ux * sine + uy * cosine
        cross_x, cross_y = -along_y, along_x
        along_allowance = (
            abs(along_x) * step['tolerance_x']
            + abs(along_y) * step['tolerance_y']
        )
        cross_allowance = (
            abs(cross_x) * step['tolerance_x']
            + abs(cross_y) * step['tolerance_y']
        )
        return along_allowance, cross_allowance

    def _absolute_recovery_speed_limit(self, step):
        """Bound returns so recovery cannot cause another large overshoot."""
        adaptive = max(0.08, step['speed'] * 0.60)
        return min(
            self.limits.horizontal_speed,
            self.limits.recovery_speed,
            self.limits.waypoint_recovery_speed_cap,
            max(self.limits.recovery_min_speed, adaptive),
        )

    def _waypoint_command_v6(self, pose, step, now, dt, filtered_velocity):
        """Track one direct segment and shape velocity before sharp turns."""
        errors = self._mission_errors(pose)
        ex, ey, ez, eyaw = errors
        dn, de = self.goal.n - pose.n, self.goal.e - pose.e
        xy = math.hypot(dn, de)
        self.errors = (xy, abs(ez), abs(math.degrees(eyaw)))
        vn_measured, ve_measured, vd_measured = filtered_velocity
        transition = self._transition_velocity(step, filtered_velocity)
        tolerances = (
            step['tolerance_x'],
            step['tolerance_y'],
            step['tolerance_depth'],
            step['tolerance_yaw'],
        )
        previous = self.waypoint_previous_error or errors
        reached = self._segment_hits_box(previous, errors, tolerances)
        transition_pending = (
            reached and not transition['transition_velocity_ready']
        )
        if reached and not transition_pending:
            self.command_debug = {
                'waypoint_phase': 'TRACK',
                'acceptance': 'swept_xyz_yaw_box_plus_instant_turn_velocity',
                'crossed_tolerance_box': True,
                'error_x': ex,
                'error_y': ey,
                'error_depth': ez,
                'error_yaw_deg': math.degrees(eyaw),
                **transition,
            }
            self.waypoint_previous_error = errors
            return self._advance_step(pose, now)

        radial_toward_velocity = (
            (dn * vn_measured + de * ve_measured) / xy
            if xy > 1e-9
            else 0.0
        )
        monitoring_response = xy > max(
            step['tolerance_x'],
            step['tolerance_y'],
            self.limits.recovery_radius,
        )
        along_remaining = xy
        cross_error = 0.0
        along_velocity = radial_toward_velocity
        cross_velocity = 0.0
        if self.waypoint_translation_unit is not None:
            ux, uy = self.waypoint_translation_unit
            along_remaining = dn * ux + de * uy
            cross_error = -dn * uy + de * ux
            along_velocity = vn_measured * ux + ve_measured * uy
            cross_velocity = -vn_measured * uy + ve_measured * ux
        progress, progress_elapsed = self._waypoint_progress(
            now,
            along_remaining,
        )
        recovery_corridor = self._absolute_recovery_corridor(step)
        self.waypoint_recovery_corridor = recovery_corridor
        gate_along_allowance, gate_cross_allowance = (
            self._absolute_gate_allowance(step)
        )
        usable_gate_cross = max(
            0.005,
            gate_cross_allowance * self.limits.waypoint_gate_margin_ratio,
        )
        soft_corridor = min(
            recovery_corridor * 0.50,
            max(
                self.limits.waypoint_soft_corridor_min,
                usable_gate_cross * 0.60,
            ),
        )
        positive_along_velocity = max(0.0, along_velocity)
        prediction_speed = max(
            0.04,
            positive_along_velocity,
            math.hypot(*self.last_linear_command[:2]) * 0.50,
        )
        time_to_gate = (
            max(0.0, along_remaining - gate_along_allowance)
            / prediction_speed
        )
        prediction_horizon = min(
            time_to_gate,
            self.limits.waypoint_response_delay,
        )
        predicted_cross_error = (
            cross_error - cross_velocity * prediction_horizon
        )
        predicted_gate_miss = abs(predicted_cross_error) > usable_gate_cross
        cross_risk = max(abs(cross_error), abs(predicted_cross_error))
        correction_denominator = max(
            0.005,
            recovery_corridor - soft_corridor,
        )
        correction_urgency = max(
            0.0,
            self._clamp(
                (cross_risk - soft_corridor) / correction_denominator,
                1.0,
            ),
        )
        predictive_correction = (
            cross_risk > soft_corridor or predicted_gate_miss
        )
        overshoot_margin = max(
            0.02,
            gate_along_allowance * self.limits.waypoint_gate_margin_ratio,
        )
        transition_grace_remaining = max(
            0.0,
            (self.waypoint_transition_grace_until or now) - now,
        )

        corner_speed = transition['corner_speed']
        turn_angle_deg = transition['turn_angle_deg']
        if xy > 1e-9:
            box_scales = [1.0]
            if abs(ex) > 1e-12:
                box_scales.append(step['tolerance_x'] / abs(ex))
            if abs(ey) > 1e-12:
                box_scales.append(step['tolerance_y'] / abs(ey))
            arrival_distance = xy * min(box_scales)
        else:
            arrival_distance = 0.0
        yaw_ready = abs(eyaw) <= step['tolerance_yaw']
        yaw_braking_zone = arrival_distance + max(0.05, step['speed'])
        yaw_holding = not yaw_ready and xy <= yaw_braking_zone
        if yaw_holding:
            corner_speed = 0.0
        sharp_turn = (
            turn_angle_deg is not None
            and turn_angle_deg > self.limits.waypoint_straight_angle_deg
        )
        if yaw_holding or transition_pending:
            passage_speed = 0.0
        elif sharp_turn or turn_angle_deg is None:
            passage_speed = max(
                corner_speed,
                min(self.limits.waypoint_corner_speed, step['speed']),
            )
        else:
            passage_speed = corner_speed
        remaining_to_box = max(0.0, xy - arrival_distance)
        response_distance = (
            positive_along_velocity * self.limits.waypoint_response_delay
        )
        predictive_remaining = max(
            0.0,
            remaining_to_box - response_distance,
        )
        profile_limit = min(
            step['speed'],
            math.sqrt(
                passage_speed * passage_speed
                + 2.0
                * self.limits.waypoint_horizontal_decel
                * predictive_remaining
            ),
        )
        self.waypoint_braking = profile_limit < step['speed'] - 1e-6
        turn_management = (
            transition_pending
            or self.waypoint_braking
            or transition_grace_remaining > 0.0
        )
        slow_response = (
            monitoring_response
            and not turn_management
            and progress_elapsed >= self.limits.recovery_delay
            and progress < self.limits.recovery_progress
        )
        gate_risk_distance = max(
            0.20,
            positive_along_velocity
            * (self.limits.waypoint_response_delay + 0.75),
        )
        raw_reasons = []
        if (
            abs(cross_error) >= recovery_corridor
            and predicted_gate_miss
            and along_remaining <= gate_risk_distance
        ):
            raw_reasons.append('deriva transversal persistente fuera del corredor')
        if along_remaining <= -overshoot_margin:
            raw_reasons.append('sobrepasó persistentemente el punto absoluto')
        moving_away_limit = max(0.02, step['speed'] * 0.25)
        moving_away = (
            monitoring_response
            and not turn_management
            and along_velocity <= -moving_away_limit
        )
        if moving_away:
            raw_reasons.append('movimiento persistente alejándose del punto')
        elif slow_response:
            raw_reasons.append('sin progreso suficiente durante la ventana')
        reasons = self._persistent_absolute_faults(
            [] if transition_grace_remaining > 0.0 else raw_reasons,
            now,
        )
        self._set_absolute_recovery(
            reasons,
            now,
            cross_error,
            radial_toward_velocity,
            recovery_corridor,
            boost_along=(
                slow_response
                or moving_away
                or along_remaining <= -overshoot_margin
            ),
        )
        recovery_speed_limit = self._absolute_recovery_speed_limit(step)
        horizontal_limit = (
            recovery_speed_limit if self.recovery_active else profile_limit
        )
        recovery_minimum = self.limits.recovery_min_speed if self.recovery_active else 0.0
        if self.waypoint_translation_unit is not None:
            ux, uy = self.waypoint_translation_unit
            normal_cross_limit = min(
                horizontal_limit,
                max(
                    self.limits.waypoint_soft_corridor_min,
                    profile_limit * (0.65 + 0.35 * correction_urgency),
                ),
            )
            cross_limit = (
                horizontal_limit
                if self.recovery_active or predicted_gate_miss
                else normal_cross_limit
            )
            cross_damping = (
                self.limits.velocity_damping
                * (
                    1.0
                    if predicted_gate_miss
                    else self.limits.waypoint_cross_damping_ratio
                )
            )
            controlled_cross_error = (
                math.copysign(
                    max(
                        0.0,
                        abs(cross_error)
                        - usable_gate_cross,
                    ),
                    cross_error,
                )
                if self.recovery_active
                else predicted_cross_error
            )
            cross = self._clamp(
                self.limits.position_gain * controlled_cross_error
                - cross_damping * cross_velocity,
                cross_limit,
            )
            recovery_return_distance = 0.0
            if along_remaining >= 0.0:
                if transition_pending:
                    along = 0.0
                else:
                    along = (
                        horizontal_limit
                        if self.recovery_active and self.recovery_boost_along
                        else profile_limit
                    )
                if predictive_correction and not self.recovery_active:
                    along *= max(0.0, 1.0 - correction_urgency)
                if self.recovery_active and along < recovery_minimum:
                    along = recovery_minimum
            else:
                return_distance = max(
                    0.0,
                    abs(along_remaining)
                    - gate_along_allowance
                    * self.limits.waypoint_gate_margin_ratio,
                )
                recovery_return_distance = return_distance
                return_speed = min(
                    horizontal_limit,
                    math.sqrt(
                        2.0
                        * self.limits.waypoint_horizontal_decel
                        * return_distance
                    ),
                )
                if self.recovery_active and return_distance > 0.005:
                    return_speed = max(return_speed, recovery_minimum)
                along = -return_speed
            along_budget = math.sqrt(max(0.0, horizontal_limit**2 - cross**2))
            along = self._clamp(along, along_budget)
            vn = along * ux - cross * uy
            ve = along * uy + cross * ux
        else:
            cross_limit = 0.0
            cross = 0.0
            along = 0.0
            vn = ve = 0.0
            recovery_return_distance = 0.0
        requested_horizontal = (vn, ve)

        vd = self._pd(ez, vd_measured, step['vertical_speed'])
        previous_command = self.last_linear_command
        vn, ve = self._slew_horizontal(
            previous_command[:2],
            requested_horizontal,
            dt,
        )
        vertical_delta = self.limits.waypoint_vertical_accel * dt
        vd = previous_command[2] + self._clamp(
            vd - previous_command[2],
            vertical_delta,
        )
        self.last_linear_command = (vn, ve, vd)

        yaw_delta = wrap(self.goal.yaw - self.yaw_command)
        yaw_limit = step['yaw_speed'] * dt
        self.yaw_command = wrap(
            self.yaw_command + self._clamp(yaw_delta, yaw_limit)
        )
        if self.recovery_active:
            self.control_mode = 'ABSOLUTE_RECOVERY'
        elif transition_pending:
            self.control_mode = 'ABSOLUTE_TURN_TRANSITION'
        elif self.waypoint_braking or predictive_correction:
            self.control_mode = 'ABSOLUTE_CORNER'
        else:
            self.control_mode = 'ABSOLUTE_TRACK'
        self.waypoint_previous_error = errors
        self.command_debug = {
            'waypoint_phase': 'TRACK',
            'acceptance': 'swept_xyz_yaw_box_plus_instant_turn_velocity',
            'crossed_tolerance_box': reached,
            'transition_pending': transition_pending,
            **transition,
            'error_x': ex,
            'error_y': ey,
            'error_depth': ez,
            'error_yaw_deg': math.degrees(eyaw),
            'tolerance_x': step['tolerance_x'],
            'tolerance_y': step['tolerance_y'],
            'tolerance_depth': step['tolerance_depth'],
            'tolerance_yaw_deg': math.degrees(step['tolerance_yaw']),
            'recovery_active': self.recovery_active,
            'recovery_reason': self.recovery_reason,
            'braking': self.waypoint_braking,
            'corner_speed': corner_speed,
            'passage_speed': passage_speed,
            'turn_angle_deg': turn_angle_deg,
            'arrival_distance': arrival_distance,
            'gate_along_allowance': gate_along_allowance,
            'gate_cross_allowance': gate_cross_allowance,
            'usable_gate_cross': usable_gate_cross,
            'soft_corridor': soft_corridor,
            'time_to_gate': time_to_gate,
            'prediction_horizon': prediction_horizon,
            'predicted_cross_error': predicted_cross_error,
            'predicted_gate_miss': predicted_gate_miss,
            'predictive_correction': predictive_correction,
            'correction_urgency': correction_urgency,
            'response_distance': response_distance,
            'predictive_remaining': predictive_remaining,
            'profile_speed_limit': profile_limit,
            'recovery_speed_limit': recovery_speed_limit,
            'recovery_return_distance': recovery_return_distance,
            'requested_horizontal_command': requested_horizontal,
            'limited_horizontal_command': (vn, ve),
            'radial_toward_velocity': radial_toward_velocity,
            'along_velocity': along_velocity,
            'cross_velocity': cross_velocity,
            'along_command': along,
            'cross_command': cross,
            'cross_limit': cross_limit,
            'along_remaining': along_remaining,
            'cross_error': cross_error,
            'recovery_corridor': recovery_corridor,
            'raw_recovery_reason': '; '.join(raw_reasons),
            'recovery_fault_age': (
                max(0.0, now - self.waypoint_fault_since)
                if self.waypoint_fault_since is not None
                else 0.0
            ),
            'transition_grace_remaining': transition_grace_remaining,
            'transition_angle_deg': self.waypoint_transition_angle_deg,
            'progress_window': progress_elapsed,
            'progress_m': progress,
            'yaw_ready': yaw_ready,
            'turn_management': turn_management,
        }
        return (vn, ve, vd, self.yaw_command)

    @staticmethod
    def _slew_vector(previous, requested, maximum_delta):
        delta = tuple(new - old for old, new in zip(previous, requested))
        magnitude = math.sqrt(sum(value * value for value in delta))
        if magnitude <= maximum_delta or magnitude <= 1e-12:
            return requested
        scale = maximum_delta / magnitude
        return tuple(old + value * scale for old, value in zip(previous, delta))

    def _slew_horizontal(self, previous, requested, dt):
        delta = tuple(new - old for old, new in zip(previous, requested))
        decelerating = sum(old * change for old, change in zip(previous, delta)) < 0
        rate = (
            self.limits.waypoint_horizontal_decel
            if decelerating
            else self.limits.waypoint_horizontal_accel
        )
        return self._slew_vector(previous, requested, rate * dt)

    def _advance_step(self, pose, now):
        was_timed = self.timed
        was_absolute = bool(self.current_step and self.current_step['kind'] in ABSOLUTE_KINDS)
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
        preserve_motion = was_absolute and self.current_step['kind'] in ABSOLUTE_KINDS
        self.begin_step(pose, now, preserve_motion=preserve_motion)
        return (*self.last_linear_command, self.yaw_command)

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
        if pose is None:
            self.cancel('Posición actual no disponible; movimiento cancelado')
            return None
        self.last_pose = pose
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
        try:
            self.check_bounds(pose, self.anchor, self.limits.depth_tolerance)
        except ValueError as exc:
            self.cancel(str(exc))
            return None
        if step['kind'] in TIMED_KINDS:
            if now - self.started >= step['duration']:
                return self._advance_step(pose, now)
            return self._timed_command(step, pose, dt)
        timeout = (
            self.limits.waypoint_timeout
            if step['kind'] in ABSOLUTE_KINDS
            else self.limits.step_timeout
        )
        if now - self.started > timeout:
            self.cancel('Tiempo máximo del paso superado')
            return None
        if step['kind'] in ABSOLUTE_KINDS:
            return self._waypoint_command_v6(pose, step, now, dt, filtered_velocity)
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
