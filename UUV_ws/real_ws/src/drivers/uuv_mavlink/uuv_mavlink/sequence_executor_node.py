"""ROS supervision of measured depth/distance/yaw sequences; transport in bridge."""

from collections import deque
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time
import uuid

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .sequence_core import Limits, number, Pose, return_steps, Sequence, TIMED_KINDS


class SequenceExecutor(Node):
    def __init__(self):
        super().__init__('uuv_sequence_executor')
        self.declare_parameter('water_density', 1000.0)
        self.declare_parameter('event_log_enabled', False)
        self.declare_parameter('event_log_directory', '')
        self.density = float(self.get_parameter('water_density').value)
        if not 950 <= self.density <= 1050:
            raise ValueError('water_density fuera del rango 950–1050 kg/m³')
        self.core = Sequence()
        self.feedback = None
        self.feedback_at = 0.0
        self.feedback_key = None
        self.good_since = None
        self.reference = None
        self.home = None
        self.validated = None
        self.ui_owner = None
        self.ui_seen = {}
        self.ui_serial = {}
        self.run = None
        self.run_permit = None
        self.command_serial = 0
        self.session = uuid.uuid4().hex
        self.tickets = deque(maxlen=20)
        self.note = 'Esperando puente de secuencias'
        self.pressure_history = deque(maxlen=40)
        self.total = 0
        self.action_serial = {}
        self.prepared_by = None
        self.last_output_command = None
        self.event_log = None
        self.event_log_path = None
        self.last_event_snapshot = None
        if bool(self.get_parameter('event_log_enabled').value):
            configured = str(self.get_parameter('event_log_directory').value).strip()
            directory = (
                Path(configured).expanduser()
                if configured
                else Path.home() / '.ros' / 'uuv_sequence_logs'
            )
            try:
                directory.mkdir(parents=True, exist_ok=True)
                stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
                self.event_log_path = str(directory / f'uuv_lanzamiento_{stamp}.jsonl')
                self.event_log = open(self.event_log_path, 'a', encoding='utf-8', buffering=1)
            except OSError as exc:
                self.get_logger().error('No se pudo abrir el registro de lanzamiento: ' + str(exc))
        self.state_pub = self.create_publisher(String, '/uuv/sequence/status', 1)
        self.command_pub = self.create_publisher(String, '/uuv/sequence/setpoint', 1)
        self.action_pub = self.create_publisher(String, '/uuv/sequence/transport_action', 10)
        self.create_subscription(String, '/uuv/sequence/bridge_status', self.on_feedback, 1)
        self.create_subscription(String, '/uuv/sequence/request', self.on_request, 10)
        self.create_subscription(String, '/uuv/sequence/ui_alive', self.on_ui, 1)
        self.timer = self.create_timer(0.05, self.tick)
        self.record_event('INFO', 'INICIO', 'Ejecutor de secuencias iniciado')

    def record_event(self, level, event, detail, **fields):
        """Append one durable, uploadable event without flooding the file."""
        if self.event_log is None:
            return
        payload = {
            'time_utc': datetime.now(timezone.utc).isoformat(timespec='milliseconds'),
            'monotonic_s': round(time.monotonic(), 3),
            'level': level,
            'event': event,
            'detail': detail,
        }
        payload.update(fields)
        try:
            self.event_log.write(json.dumps(payload, ensure_ascii=False, allow_nan=False) + '\n')
            self.event_log.flush()
        except (OSError, TypeError, ValueError) as exc:
            self.get_logger().error('No se pudo escribir el registro de lanzamiento: ' + str(exc))

    @staticmethod
    def publish(pub, data):
        msg = String()
        msg.data = json.dumps(data, allow_nan=False)
        pub.publish(msg)

    def bridge_action(self, action):
        if self.feedback is not None:
            f = self.feedback
            self.publish(
                self.action_pub,
                {
                    'action': action,
                    'session': f['session'],
                    'permit': f['permit'],
                    'ticket': f['ticket'],
                },
            )

    def clear_references(self, reason):
        self.reference = None
        self.home = None
        self.validated = None
        self.good_since = None
        self.pressure_history.clear()
        if self.core.state == 'RUNNING':
            self.cancel(reason)
        self.note = reason
        self.record_event('ERROR', 'REFERENCIAS_INVALIDADAS', reason)

    def cancel(self, reason, action='cancel'):
        self.core.cancel(reason)
        self.validated = None
        self.bridge_action(action)
        self.run = None
        self.last_output_command = None
        self.note = reason
        level = (
            'INFO'
            if reason in ('Cancelación solicitada desde interfaz', 'Ejecutor cerrado')
            else 'ERROR'
        )
        self.record_event(level, 'EJECUCION_CANCELADA', reason, action=action)

    def on_feedback(self, msg):
        try:
            f = json.loads(msg.data)
            if not isinstance(f, dict) or not all(
                k in f
                for k in (
                    'session',
                    'nav_epoch',
                    'permit',
                    'ticket',
                    'samples',
                    'health',
                    'blockers',
                    'mode',
                    'armed',
                )
            ):
                return
            # Malformed numeric data must never enter the controller.
            for key in ('position', 'velocity'):
                if f['samples'].get(key) is not None:
                    if len(f['samples'][key]) != 3:
                        return
                    for v in f['samples'][key]:
                        number(v)
            for key in ('pressure', 'yaw'):
                if f['samples'].get(key) is not None:
                    number(f['samples'][key])
            context = (f['session'], f['nav_epoch'])
            changed = self.feedback_key is not None and context != self.feedback_key
            previous = self.feedback
            self.feedback = f
            self.feedback_at = time.monotonic()
            self.feedback_key = context
            if changed:
                self.clear_references('Reinicio/cambio de origen; guardar referencia otra vez')
            previous_health = tuple(previous.get('health', [])) if previous else ()
            current_health = tuple(f['health'])
            if current_health != previous_health:
                self.record_event(
                    'WARN' if current_health else 'INFO',
                    'SALUD_NAVEGACION',
                    '; '.join(current_health) if current_health else 'Telemetría recuperada',
                    health=list(current_health),
                )
            previous_diagnostics = tuple(previous.get('diagnostics', [])) if previous else ()
            current_diagnostics = tuple(f.get('diagnostics', []))
            if current_diagnostics != previous_diagnostics:
                self.record_event(
                    'WARN' if current_diagnostics else 'INFO',
                    'DIAGNOSTICO_SENSORES',
                    (
                        '; '.join(current_diagnostics)
                        if current_diagnostics
                        else 'Diagnósticos recuperados'
                    ),
                    diagnostics=list(current_diagnostics),
                )
            previous_blockers = tuple(previous.get('blockers', [])) if previous else ()
            current_blockers = tuple(f['blockers'])
            if current_blockers != previous_blockers:
                self.record_event(
                    'WARN' if current_blockers else 'INFO',
                    'BLOQUEOS',
                    '; '.join(current_blockers) if current_blockers else 'Sin bloqueos',
                    blockers=list(current_blockers),
                )
            if previous and f.get('reason') and f.get('reason') != previous.get('reason'):
                self.record_event('WARN', 'ESTADO_PUENTE', f['reason'])
            if previous and (previous.get('mode'), previous.get('armed')) != (
                f.get('mode'),
                f.get('armed'),
            ):
                self.record_event(
                    'INFO',
                    'MODO_ARMADO',
                    f'Modo {f.get("mode")}; armado {f.get("armed")}',
                    mode=f.get('mode'),
                    armed=f.get('armed'),
                )
            samples_ready = all(
                f['samples'].get(key) is not None
                for key in ('position', 'velocity', 'yaw', 'pressure')
            )
            if f['health']:
                reason = 'Navegación interrumpida: ' + '; '.join(f['health'])
                if self.core.state == 'RUNNING':
                    if not f.get('paused'):
                        self.clear_references(reason)
                elif self.reference is not None:
                    # Antes de ejecutar, conservar superficie, Home y plan.
                    # navigation_ready() seguirá bloqueando hasta recuperar
                    # 2 s continuos de telemetría saludable.
                    self.note = reason + '; esperando recuperación'
                self.good_since = None
                self.pressure_history.clear()
            elif samples_ready:
                if self.good_since is None:
                    self.good_since = self.feedback_at
                self.pressure_history.append((self.feedback_at, f['samples']['pressure']))
            else:
                self.good_since = None
            if self.core.state == 'RUNNING' and (
                f['permit'] != self.run_permit or f['mode'] != 'GUIDED' or not f['armed']
            ):
                self.cancel('Cambio de modo, armado o permiso: ejecución descartada')
        except (ValueError, TypeError, KeyError):
            return

    def on_ui(self, msg):
        try:
            p = json.loads(msg.data)
            ui, serial = p['ui'], p['serial']
            if not isinstance(ui, str) or len(ui) > 80 or type(serial) is not int:
                return
            if serial > self.ui_serial.get(ui, -1):
                # UI liveness also echoes a recent executor ticket.
                if self.request_ticket(p):
                    self.ui_serial[ui] = serial
                    self.ui_seen[ui] = time.monotonic()
        except (ValueError, TypeError, KeyError):
            return

    def request_ticket(self, p):
        return p.get('session') == self.session and any(
            p.get('ticket') == token and 0 <= time.monotonic() - stamp < 0.5
            for token, stamp in self.tickets
        )

    def navigation_ready(self):
        f = self.feedback
        if f is None or time.monotonic() - self.feedback_at > 0.35:
            raise ValueError('Puente sin datos recientes')
        if f['health']:
            raise ValueError('; '.join(f['health']))
        missing = [
            key
            for key in ('position', 'velocity', 'yaw', 'pressure')
            if f['samples'].get(key) is None
        ]
        if missing:
            raise ValueError('Sin datos recientes requeridos: ' + ', '.join(missing))
        if self.good_since is None or time.monotonic() - self.good_since < 2:
            raise ValueError('Esperando 2 s de telemetría continua')
        return f

    def pose(self):
        f = self.navigation_ready()
        if self.reference is None:
            raise ValueError('Falta guardar referencia de superficie')
        p0, d0, z0 = self.reference
        s = f['samples']
        depth = d0 + (s['pressure'] - p0) * 100 / (self.density * 9.80665)
        # Large disagreement can indicate an EKF reset not accompanied by an origin message.
        if abs((s['position'][2] - z0) - (depth - d0)) > 0.40:
            self.clear_references('Profundidad y NED z discrepan >0.40 m')
            raise ValueError(self.note)
        return Pose(s['position'][0], s['position'][1], depth, s['yaw'])

    @staticmethod
    def requested_limits(packet):
        maximum = number(packet['max_depth'])
        radius = number(packet['radius'])
        if not 0.2 <= maximum <= 3 or not 0.5 <= radius <= 5:
            raise ValueError('Límites: profundidad 0.2–3 m, radio 0.5–5 m')
        return Limits(
            max_depth=maximum,
            radius=radius,
            horizontal_speed=number(packet['max_horizontal_speed']),
            vertical_speed=number(packet['max_vertical_speed']),
            yaw_speed=math.radians(number(packet['max_yaw_speed_deg'])),
        )

    def on_request(self, msg):
        try:
            p = json.loads(msg.data)
            if not isinstance(p, dict) or not self.request_ticket(p):
                return
            ui = p.get('ui')
            serial = p.get('serial')
            if not isinstance(ui, str) or not ui or len(ui) > 80 or type(serial) is not int:
                return
            if serial <= self.action_serial.get(ui, -1):
                return
            self.action_serial[ui] = serial
            action = p.get('action')
            self.record_event('INFO', 'SOLICITUD_UI', str(action), ui=ui, serial=serial)
            if action in ('cancel', 'manual'):
                self.cancel('Cancelación solicitada desde interfaz', action)
                return
            if self.core.state == 'RUNNING':
                raise ValueError('Cancela la ejecución antes de cambiar objetivos o referencias')
            self.ui_owner = ui
            if action == 'prepare':
                # One transaction; never retain a previous plan after failure.
                self.validated = None
                self.prepared_by = None
                self.reference = None
                self.home = None
                f = self.navigation_ready()
                if f['armed'] or f['mode'] != 'MANUAL':
                    raise ValueError('Preparar requiere MANUAL y ROV desarmado')
                depth = number(p['sensor_depth'])
                if not 0 <= depth <= 0.5:
                    raise ValueError('Profundidad conocida del sensor: 0–0.50 m')
                limits = self.requested_limits(p)
                recent = [v for t, v in self.pressure_history if time.monotonic() - t < 1.5]
                if len(recent) < 10 or max(recent) - min(recent) > 2:
                    raise ValueError('Presión inestable: mantén el sensor a profundidad fija')
                pose = Pose(*f['samples']['position'][:2], depth, f['samples']['yaw'])
                candidate = Sequence(limits)
                parsed = candidate.validate(pose, p['steps'], pose)
                self.reference = (sum(recent) / len(recent), depth, f['samples']['position'][2])
                self.home = pose
                self.core.limits = candidate.limits
                self.validated = {'steps': parsed, 'returning': False, 'pose': pose}
                self.prepared_by = [ui, serial]
                self.note = (
                    'Prueba preparada: superficie, Home y plan guardados. '
                    'No se ha armado ni movido el ROV.'
                )
                self.record_event(
                    'INFO',
                    'PRUEBA_PREPARADA',
                    self.note,
                    steps=[dict(step) for step in parsed],
                    sensor_depth=depth,
                    limits=vars(limits),
                )
            elif action == 'guided':
                self.navigation_ready()
                if self.feedback['blockers']:
                    raise ValueError('; '.join(self.feedback['blockers']))
                self.bridge_action('guided')
                self.note = 'GUIDED solicitado; esperar confirmación del modo real'
                self.record_event('INFO', 'GUIDED_SOLICITADO', self.note)
            elif action == 'surface':
                f = self.navigation_ready()
                if f['armed']:
                    raise ValueError('Guardar superficie requiere ROV desarmado')
                depth = number(p['sensor_depth'])
                if not 0 <= depth <= 0.5:
                    raise ValueError('Indica profundidad conocida del sensor entre 0 y 0.50 m')
                recent = [
                    value
                    for stamp, value in self.pressure_history
                    if time.monotonic() - stamp < 1.5
                ]
                if len(recent) < 10 or max(recent) - min(recent) > 2:
                    raise ValueError(
                        'Mantén el sensor a profundidad fija; presión todavía inestable'
                    )
                self.reference = (sum(recent) / len(recent), depth, f['samples']['position'][2])
                self.home = None
                self.validated = None
                self.note = (
                    'Referencia guardada; la profundidad indicada es la del sensor de presión'
                )
            elif action == 'home':
                pose = self.pose()
                if self.feedback['armed']:
                    raise ValueError('Esta primera versión guarda Home desarmado')
                self.home = pose
                self.validated = None
                self.note = 'Home real guardado en esta sesión'
            elif action in ('validate', 'validate_home'):
                pose = self.pose()
                self.core.limits = self.requested_limits(p)
                returning = action == 'validate_home'
                if returning and self.home is None:
                    raise ValueError('No hay Home real guardado')
                steps = return_steps(pose, self.home) if returning else p['steps']
                parsed = self.core.validate(pose, steps, self.home)
                self.validated = {'steps': parsed, 'returning': returning, 'pose': pose}
                self.note = (
                    'Geometría validada. Ejecutar volverá a tomar la posición y rumbo actuales.'
                )
            elif action == 'execute':
                pose = self.pose()
                if self.validated is None:
                    raise ValueError('Primero pulsa Validar secuencia o Validar regreso')
                if self.feedback['blockers']:
                    raise ValueError('; '.join(self.feedback['blockers']))
                if not self.feedback['armed'] or self.feedback['mode'] != 'GUIDED':
                    raise ValueError('Requiere armado por el piloto y GUIDED confirmado')
                if time.monotonic() - self.ui_seen.get(ui, -100) > 0.5:
                    raise ValueError('La interfaz no tiene señal de vida reciente')
                returning = self.validated['returning']
                # Require the same GUI list that was validated; edits cannot
                # silently execute old work.
                if not returning:
                    current_plan = self.core.validate(pose, p.get('steps'), self.home)
                    if current_plan != self.validated['steps']:
                        raise ValueError('La lista cambió; vuelve a validar')
                if self.requested_limits(p) != self.core.limits:
                    raise ValueError('Los límites cambiaron; vuelve a validar')
                steps = return_steps(pose, self.home) if returning else self.validated['steps']
                self.core.start(pose, steps, time.monotonic(), self.home, returning)
                self.total = len(steps)
                self.run = uuid.uuid4().hex
                self.run_permit = self.feedback['permit']
                self.command_serial = 0
                self.last_output_command = (0.0, 0.0, 0.0, pose.yaw)
                self.note = 'Ejecución iniciada desde la referencia actual'
                self.record_event(
                    'INFO',
                    'EJECUCION_INICIADA',
                    self.note,
                    run=self.run,
                    steps=[dict(step) for step in self.core.steps],
                )
            else:
                raise ValueError('Solicitud desconocida')
        except (ValueError, TypeError, KeyError) as exc:
            self.note = str(exc)
            self.record_event(
                'ERROR', 'SOLICITUD_RECHAZADA', self.note, action=locals().get('action')
            )
            if 'action' in locals() and action in ('prepare', 'validate', 'validate_home'):
                self.validated = None

    def tick(self):
        now = time.monotonic()
        try:
            current = self.pose()
            nav_error = ''
        except ValueError as exc:
            current = None
            nav_error = str(exc)
            if self.reference is not None and (
                self.feedback is None or now - self.feedback_at > 0.35
            ):
                self.clear_references('Puente desconectado; referencias invalidadas')
        if self.core.state == 'RUNNING':
            error = nav_error
            if now - self.ui_seen.get(self.ui_owner, -100) > 0.5:
                error = 'Interfaz cerrada o sin respuesta'
            if self.feedback and self.feedback['blockers']:
                error = '; '.join(self.feedback['blockers'])
            timed_without_pose = self.core.timed and error == nav_error
            if error and not timed_without_pose:
                self.cancel(error)
            else:
                velocity = self.feedback['samples'].get('velocity')
                measured_speed = math.sqrt(sum(v * v for v in velocity)) if velocity else 0.0
                command = self.core.update(current, now, speed=measured_speed)
                if command is None or self.core.state != 'RUNNING':
                    self.bridge_action('complete' if self.core.state == 'COMPLETED' else 'cancel')
                    self.note = self.core.reason
                    self.validated = None
                    self.run = None
                else:
                    self.last_output_command = command
                    f = self.feedback
                    self.publish(
                        self.command_pub,
                        {
                            'session': f['session'],
                            'permit': self.run_permit,
                            'ticket': f['ticket'],
                            'run': self.run,
                            'serial': self.command_serial,
                            'command': command,
                            'limits': {
                                'horizontal_speed': self.core.limits.horizontal_speed,
                                'vertical_speed': self.core.limits.vertical_speed,
                                'yaw_speed': self.core.limits.yaw_speed,
                            },
                        },
                    )
                    self.command_serial += 1
        token = uuid.uuid4().hex
        self.tickets.append((token, now))
        f = self.feedback if self.feedback and now - self.feedback_at < 0.35 else {}
        goal = None if self.core.timed else self.core.goal
        timeout_left = None
        if self.core.state == 'RUNNING':
            limit = (
                self.core.current_step['duration']
                if self.core.timed
                else self.core.limits.step_timeout
            )
            timeout_left = max(0, limit - (now - self.core.started))
        phase = self.phase_status(now, current, nav_error)
        snapshot = (self.core.state, self.core.index, phase['code'])
        if snapshot != self.last_event_snapshot:
            level = (
                'ERROR'
                if phase['code'] in ('FALLA', 'BLOQUEADO')
                else ('WARN' if phase['code'] == 'PAUSA' else 'INFO')
            )
            self.record_event(
                level,
                'ESTADO',
                phase['title'],
                state=self.core.state,
                step=self.core.index + 1 if self.core.state == 'RUNNING' else 0,
                total=self.total,
                phase=phase,
            )
            self.last_event_snapshot = snapshot
        self.publish(
            self.state_pub,
            {
                'session': self.session,
                'ticket': token,
                'state': self.core.state,
                'note': self.note,
                'nav_error': nav_error,
                'mode': f.get('mode'),
                'armed': f.get('armed'),
                'enabled': f.get('enabled', False),
                'blockers': f.get('blockers', ['Puente sin conexión']),
                'bridge_reason': f.get('reason'),
                'stopping': f.get('stopping'),
                'surface': self.reference is not None,
                'home': vars(self.home) if self.home else None,
                'pose': vars(current) if current else None,
                'validated': self.validated is not None,
                'prepared_by': self.prepared_by if self.validated is not None else None,
                'returning': self.validated['returning'] if self.validated else False,
                'step': self.core.index + 1 if self.core.state == 'RUNNING' else 0,
                'total': self.total,
                'goal': vars(goal) if goal else None,
                'motion_limits': {
                    'horizontal_speed': self.core.limits.horizontal_speed,
                    'vertical_speed': self.core.limits.vertical_speed,
                    'yaw_speed': self.core.limits.yaw_speed,
                },
                'errors': self.core.errors,
                'timeout_left': timeout_left,
                'dvl_confidence': f.get('samples', {}).get('dvl'),
                'diagnostics': f.get('diagnostics', []),
                'paused': False,
                'pause_elapsed': None,
                'dvl_velocity_valid': f.get('dvl_velocity_valid'),
                'dvl_altitude': f.get('dvl_altitude'),
                'navigation_ready': current is not None,
                'phase': phase,
                'current_step': (
                    dict(self.core.steps[self.core.index])
                    if self.core.state == 'RUNNING' and self.core.index < len(self.core.steps)
                    else None
                ),
                'step_elapsed': (
                    max(0.0, now - self.core.started)
                    if self.core.state == 'RUNNING' and self.core.started is not None
                    else None
                ),
                'event_log_path': self.event_log_path,
            },
        )

    def phase_status(self, now, current, nav_error):
        """Describe what the controller is doing in operator language."""
        if self.core.state == 'RUNNING':
            step = self.core.current_step
            kind = step['kind']
            if kind in TIMED_KINDS:
                remaining = max(0.0, step['duration'] - (now - self.core.started))
                labels = {
                    'velocity_surge': 'VELOCIDAD AVANCE/RETROCESO',
                    'velocity_sway': 'VELOCIDAD LATERAL',
                    'velocity_heave': 'VELOCIDAD VERTICAL',
                    'velocity_yaw': 'VELOCIDAD DE GIRO',
                }
                detail = (
                    f'Paso {self.core.index + 1}/{self.total} · '
                    f'{step["speed"]:.3f} · faltan {remaining:.1f} s'
                )
                if current is None:
                    detail += ' · DVL/posición no disponibles; ejecución temporal continúa'
                return {
                    'code': 'TEMPORAL',
                    'title': labels[kind],
                    'detail': detail,
                    'remaining_s': remaining,
                }
            if current is None:
                return {'code': 'BLOQUEADO', 'title': 'SIN POSICIÓN VÁLIDA', 'detail': nav_error}
            xy, depth, yaw = self.core.errors
            inside = self.core.inside_since is not None
            if kind == 'wait':
                required = max(self.core.limits.dwell, step['duration'])
                elapsed = max(0.0, now - self.core.inside_since) if inside else 0.0
                remaining = max(0.0, required - elapsed)
                if not inside:
                    return {
                        'code': 'CORRIGIENDO',
                        'title': 'RECUPERANDO POSICIÓN PARA ESPERAR',
                        'detail': f'Paso {self.core.index + 1}/{self.total} · fuera de tolerancia',
                    }
                return {
                    'code': 'ESPERA',
                    'title': 'ESPERANDO',
                    'detail': (
                        f'Paso {self.core.index + 1}/{self.total} · faltan {remaining:.1f} s'
                    ),
                    'remaining_s': remaining,
                }
            if depth > self.core.limits.depth_tolerance:
                return {
                    'code': 'PROFUNDIDAD',
                    'title': 'CORRIGIENDO PROFUNDIDAD',
                    'detail': f'Paso {self.core.index + 1}/{self.total} · error {depth:.2f} m',
                }
            if yaw > 15.0:
                return {
                    'code': 'RUMBO',
                    'title': 'ALINEANDO RUMBO',
                    'detail': f'Paso {self.core.index + 1}/{self.total} · error {yaw:.1f}°',
                }
            if kind in ('advance', 'lateral') and xy > self.core.limits.xy_tolerance:
                title = 'AVANZANDO' if kind == 'advance' else 'DESPLAZAMIENTO LATERAL'
                return {
                    'code': 'AVANCE',
                    'title': title,
                    'detail': f'Paso {self.core.index + 1}/{self.total} · faltan {xy:.2f} m',
                }
            if kind == 'turn' and yaw > self.core.limits.yaw_tolerance * 180 / math.pi:
                return {
                    'code': 'GIRO',
                    'title': 'GIRANDO',
                    'detail': f'Paso {self.core.index + 1}/{self.total} · faltan {yaw:.1f}°',
                }
            return {
                'code': 'ESTABILIZANDO',
                'title': 'ESTABILIZANDO EN EL OBJETIVO',
                'detail': (
                    f'Paso {self.core.index + 1}/{self.total} · '
                    'debe permanecer dentro de tolerancia'
                ),
            }
        if self.core.state == 'COMPLETED':
            return {'code': 'COMPLETADO', 'title': 'PLAN COMPLETADO', 'detail': self.core.reason}
        if self.core.state == 'CANCELLED':
            return {'code': 'FALLA', 'title': 'PLAN CANCELADO', 'detail': self.note}
        if nav_error:
            return {'code': 'BLOQUEADO', 'title': 'ESPERANDO TELEMETRÍA', 'detail': nav_error}
        if self.validated is not None and self.feedback:
            if self.feedback.get('armed') and self.feedback.get('mode') == 'GUIDED':
                return {
                    'code': 'LISTO',
                    'title': 'LISTO PARA INICIAR',
                    'detail': 'Plan preparado y GUIDED confirmado',
                }
            if self.feedback.get('armed'):
                return {
                    'code': 'LISTO_GUIDED',
                    'title': 'LISTO PARA SOLICITAR GUIDED',
                    'detail': 'Plan preparado y ROV armado',
                }
            return {
                'code': 'PREPARADO',
                'title': 'PRUEBA PREPARADA',
                'detail': 'Falta armar y solicitar GUIDED',
            }
        return {'code': 'IDLE', 'title': 'SIN PLAN EN EJECUCIÓN', 'detail': self.note}


def main(args=None):
    rclpy.init(args=args)
    node = SequenceExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.cancel('Ejecutor cerrado')
        except Exception:
            pass
        if node.event_log is not None:
            node.event_log.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
