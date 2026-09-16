"""Pure safety gate for the future UUV sequence executor.

This module deliberately has no ROS or MAVLink dependency.  It validates a
small test plan and models the cancellation/watchdog decisions so they can be
tested without connecting a vehicle.  It does not implement vehicle control.
"""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Limits:
    """Conservative software limits for a future, supervised pool test.

    They are validation limits, not a statement that a pool or vehicle is safe
    at these values.
    """

    max_steps: int = 16
    max_advance_m: float = 2.0
    max_turn_deg: float = 90.0
    max_depth_m: float = 3.0
    max_wait_s: float = 10.0
    max_total_advance_m: float = 5.0
    max_total_wait_s: float = 30.0


def _number(value, label):
    if not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f'{label} debe ser un número finito')
    return float(value)


def validate_steps(steps, limits=Limits()):
    """Return a normalized plan, rejecting unsupported or excessive inputs."""
    if not isinstance(steps, list) or not steps:
        raise ValueError('La secuencia debe contener al menos un paso')
    if len(steps) > limits.max_steps:
        raise ValueError('La secuencia supera el número máximo de pasos')
    result = []
    total_advance = 0.0
    total_wait = 0.0
    for item in steps:
        if not isinstance(item, (list, tuple)) or len(item) != 2:
            raise ValueError('Cada paso debe ser [tipo, valor]')
        kind, raw_value = item
        value = _number(raw_value, 'El valor del paso')
        if kind == 'advance':
            if abs(value) > limits.max_advance_m:
                raise ValueError('El avance supera el límite por paso')
            total_advance += abs(value)
        elif kind == 'turn':
            if abs(value) > limits.max_turn_deg:
                raise ValueError('El giro supera el límite por paso')
        elif kind == 'depth':
            if not 0.0 <= value <= limits.max_depth_m:
                raise ValueError('La profundidad está fuera del límite')
        elif kind == 'wait':
            if not 0.0 <= value <= limits.max_wait_s:
                raise ValueError('La espera está fuera del límite')
            total_wait += value
        else:
            raise ValueError('Paso desconocido: ' + str(kind))
        result.append((kind, value))
    if total_advance > limits.max_total_advance_m:
        raise ValueError('La distancia acumulada supera el límite')
    if total_wait > limits.max_total_wait_s:
        raise ValueError('La espera acumulada supera el límite')
    return result


class ExecutionGuard:
    """State and watchdog for a future physical executor.

    The caller must keep ``output_enabled`` false until the complete vehicle
    transport has separately been validated.  A heartbeat loss and every
    explicit cancellation permanently leave this plan in ``CANCELLED``.
    """

    IDLE = 'IDLE'
    ACTIVE = 'ACTIVE'
    CANCELLED = 'CANCELLED'

    def __init__(self, heartbeat_timeout_s=0.5):
        if heartbeat_timeout_s <= 0.0:
            raise ValueError('El timeout debe ser positivo')
        self.heartbeat_timeout_s = heartbeat_timeout_s
        self.state = self.IDLE
        self.reason = 'Sin secuencia activa'
        self.last_heartbeat = None

    def start(self, now, output_enabled):
        if not output_enabled:
            self.state = self.CANCELLED
            self.reason = 'Salida física deshabilitada por configuración'
            return False
        self.state = self.ACTIVE
        self.reason = 'Secuencia supervisada activa'
        self.last_heartbeat = now
        return True

    def heartbeat(self, now):
        if self.state == self.ACTIVE:
            self.last_heartbeat = now

    def cancel(self, reason):
        self.state = self.CANCELLED
        self.reason = reason

    def watchdog(self, now):
        if self.state != self.ACTIVE:
            return False
        if now - self.last_heartbeat > self.heartbeat_timeout_s:
            self.cancel('Cancelada: el ejecutor dejó de responder')
            return True
        return False

    def allowed(self, *, connected, armed, mode, telemetry_ok,
                failsafe_ok, output_enabled):
        if self.state != self.ACTIVE:
            return False
        return all((connected, armed, mode == 'GUIDED', telemetry_ok,
                    failsafe_ok, output_enabled))
