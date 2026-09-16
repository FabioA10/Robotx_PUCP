"""ROS supervision of measured depth/distance/yaw sequences; transport in bridge."""

from collections import deque
import json
import math
import time
import uuid

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .sequence_core import Limits, Pose, Sequence, number, return_steps


class SequenceExecutor(Node):
    def __init__(self):
        super().__init__('uuv_sequence_executor')
        self.declare_parameter('water_density', 1000.0)
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
        self.limits_confirmed = False
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
        self.state_pub = self.create_publisher(String, '/uuv/sequence/status', 1)
        self.command_pub = self.create_publisher(String, '/uuv/sequence/setpoint', 1)
        self.action_pub = self.create_publisher(String, '/uuv/sequence/transport_action', 10)
        self.create_subscription(String, '/uuv/sequence/bridge_status', self.on_feedback, 1)
        self.create_subscription(String, '/uuv/sequence/request', self.on_request, 10)
        self.create_subscription(String, '/uuv/sequence/ui_alive', self.on_ui, 1)
        self.timer = self.create_timer(0.05, self.tick)

    @staticmethod
    def publish(pub, data):
        msg = String()
        msg.data = json.dumps(data, allow_nan=False)
        pub.publish(msg)

    def bridge_action(self, action):
        if self.feedback is not None:
            f = self.feedback
            self.publish(self.action_pub, dict(action=action, session=f['session'],
                                               permit=f['permit'], ticket=f['ticket']))

    def clear_references(self, reason):
        self.reference = None
        self.home = None
        self.validated = None
        self.good_since = None
        self.pressure_history.clear()
        if self.core.state == 'RUNNING':
            self.cancel(reason)
        self.note = reason

    def cancel(self, reason, action='cancel'):
        self.core.cancel(reason)
        self.validated = None
        self.bridge_action(action)
        self.run = None
        self.note = reason

    def on_feedback(self, msg):
        try:
            f = json.loads(msg.data)
            if not isinstance(f, dict) or not all(k in f for k in ('session', 'nav_epoch', 'permit', 'ticket', 'samples', 'health', 'blockers', 'mode', 'armed')):
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
            self.feedback = f
            self.feedback_at = time.monotonic()
            self.feedback_key = context
            if changed:
                self.clear_references('Reinicio/cambio de origen; guardar referencia otra vez')
            if f['health']:
                if self.reference is not None or self.core.state == 'RUNNING':
                    self.clear_references('Navegación interrumpida: ' + '; '.join(f['health']))
                self.good_since = None
                self.pressure_history.clear()
            else:
                if self.good_since is None:
                    self.good_since = self.feedback_at
                self.pressure_history.append((self.feedback_at, f['samples']['pressure']))
            if self.core.state == 'RUNNING' and (f['permit'] != self.run_permit or f['mode'] != 'GUIDED' or not f['armed']):
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
            for token, stamp in self.tickets)

    def navigation_ready(self):
        f = self.feedback
        if f is None or time.monotonic() - self.feedback_at > 0.35:
            raise ValueError('Puente sin datos recientes')
        if f['health']:
            raise ValueError('; '.join(f['health']))
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
            if action in ('cancel', 'manual'):
                self.cancel('Cancelación solicitada desde interfaz', action)
                return
            if self.core.state == 'RUNNING':
                raise ValueError('Cancela la ejecución antes de cambiar objetivos o referencias')
            self.ui_owner = ui
            if action == 'guided':
                self.navigation_ready()
                if self.feedback['blockers']:
                    raise ValueError('; '.join(self.feedback['blockers']))
                self.bridge_action('guided')
                self.note = 'GUIDED solicitado; esperar confirmación del modo real'
            elif action == 'surface':
                f = self.navigation_ready()
                if f['armed']:
                    raise ValueError('Guardar superficie requiere ROV desarmado')
                depth = number(p['sensor_depth'])
                if not 0 <= depth <= 0.5:
                    raise ValueError('Indica profundidad conocida del sensor entre 0 y 0.50 m')
                recent = [value for stamp, value in self.pressure_history if time.monotonic() - stamp < 1.5]
                if len(recent) < 10 or max(recent) - min(recent) > 2:
                    raise ValueError('Mantén el sensor a profundidad fija; presión todavía inestable')
                self.reference = (sum(recent) / len(recent), depth, f['samples']['position'][2])
                self.home = None
                self.validated = None
                self.note = 'Referencia guardada; la profundidad indicada es la del sensor de presión'
            elif action == 'home':
                pose = self.pose()
                if self.feedback['armed']:
                    raise ValueError('Esta primera versión guarda Home desarmado')
                self.home = pose
                self.validated = None
                self.note = 'Home real guardado en esta sesión'
            elif action in ('validate', 'validate_home'):
                pose = self.pose()
                if p.get('limits_confirmed') is not True:
                    raise ValueError('Confirma espacio, profundidad y recuperación de la piscina')
                maximum = number(p['max_depth'])
                radius = number(p['radius'])
                if not 0.2 <= maximum <= 3 or not 0.5 <= radius <= 5:
                    raise ValueError('Límites de piscina: profundidad 0.2–3 m, radio 0.5–5 m')
                self.core.limits = Limits(max_depth=maximum, radius=radius)
                returning = action == 'validate_home'
                if returning and self.home is None:
                    raise ValueError('No hay Home real guardado')
                steps = return_steps(pose, self.home) if returning else p['steps']
                parsed = self.core.validate(pose, steps, self.home)
                self.validated = {'steps': parsed, 'returning': returning, 'pose': pose}
                self.limits_confirmed = True
                self.note = 'Geometría validada. Ejecutar volverá a tomar la posición y rumbo actuales.'
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
                # Require the same GUI list that was validated; edits cannot silently execute old work.
                if not returning and p.get('steps') != [list(step) for step in self.validated['steps']]:
                    raise ValueError('La lista cambió; vuelve a validar')
                if [number(p['max_depth']), number(p['radius'])] != [self.core.limits.max_depth, self.core.limits.radius] or p.get('limits_confirmed') is not True:
                    raise ValueError('Los límites cambiaron; vuelve a validar')
                steps = return_steps(pose, self.home) if returning else self.validated['steps']
                self.core.start(pose, steps, time.monotonic(), self.home, returning)
                self.total = len(steps)
                self.run = uuid.uuid4().hex
                self.run_permit = self.feedback['permit']
                self.command_serial = 0
                self.note = 'Ejecución iniciada desde la referencia actual'
            else:
                raise ValueError('Solicitud desconocida')
        except (ValueError, TypeError, KeyError) as exc:
            self.note = str(exc)
            if 'action' in locals() and action in ('validate', 'validate_home'):
                self.validated = None

    def tick(self):
        now = time.monotonic()
        try:
            current = self.pose()
            nav_error = ''
        except ValueError as exc:
            current = None
            nav_error = str(exc)
            if self.reference is not None and (self.feedback is None or now - self.feedback_at > 0.35):
                self.clear_references('Puente desconectado; referencias invalidadas')
        if self.core.state == 'RUNNING':
            error = nav_error
            if now - self.ui_seen.get(self.ui_owner, -100) > 0.5:
                error = 'Interfaz cerrada o sin respuesta'
            if self.feedback and self.feedback['blockers']:
                error = '; '.join(self.feedback['blockers'])
            if error:
                self.cancel(error)
            else:
                velocity = self.feedback['samples']['velocity']
                command = self.core.update(current, now, speed=math.sqrt(sum(v*v for v in velocity)))
                if command is None or self.core.state != 'RUNNING':
                    self.bridge_action('complete' if self.core.state == 'COMPLETED' else 'cancel')
                    self.note = self.core.reason
                    self.validated = None
                    self.run = None
                else:
                    f = self.feedback
                    self.publish(self.command_pub, dict(session=f['session'], permit=self.run_permit,
                                                        ticket=f['ticket'], run=self.run,
                                                        serial=self.command_serial, command=command))
                    self.command_serial += 1
        token = uuid.uuid4().hex
        self.tickets.append((token, now))
        f = self.feedback if self.feedback and now - self.feedback_at < 0.35 else {}
        goal = self.core.goal
        timeout_left = max(0, self.core.limits.step_timeout - (now - self.core.started)) if self.core.state == 'RUNNING' else None
        self.publish(self.state_pub, {
            'session': self.session, 'ticket': token, 'state': self.core.state,
            'note': self.note, 'nav_error': nav_error, 'mode': f.get('mode'),
            'armed': f.get('armed'), 'enabled': f.get('enabled', False),
            'blockers': f.get('blockers', ['Puente sin conexión']),
            'bridge_reason': f.get('reason'), 'stopping': f.get('stopping'),
            'surface': self.reference is not None,
            'home': vars(self.home) if self.home else None,
            'pose': vars(current) if current else None,
            'validated': self.validated is not None,
            'returning': self.validated['returning'] if self.validated else False,
            'step': self.core.index + 1 if self.core.state == 'RUNNING' else 0,
            'total': self.total, 'goal': vars(goal) if goal else None,
            'errors': self.core.errors, 'timeout_left': timeout_left,
            'dvl_confidence': f.get('samples', {}).get('dvl'),
        })


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
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
