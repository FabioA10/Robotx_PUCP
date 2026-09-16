"""GUIDED transport with a local watchdog, single-owner lease and fresh telemetry."""

from collections import deque
import json
import math
import time
import uuid

from std_msgs.msg import String
from .sequence_core import number


class GuidedBridge:
    def __init__(self, bridge, clock=time.monotonic):
        self.b = bridge
        self.clock = clock
        defaults = {
            'sequence_monitor_enabled': False, 'enable_sequence_output': False,
            'dvl_source_system': 255, 'dvl_source_component': 0,
            'dvl_min_confidence': 50.0,
        }
        for name, value in defaults.items():
            bridge.declare_parameter(name, value)
        self.monitor = bool(bridge.get_parameter('sequence_monitor_enabled').value)
        self.enabled = bool(bridge.get_parameter('enable_sequence_output').value)
        self.dvl_ids = (int(bridge.get_parameter('dvl_source_system').value),
                        int(bridge.get_parameter('dvl_source_component').value))
        self.min_confidence = float(bridge.get_parameter('dvl_min_confidence').value)
        if self.enabled and (not self.monitor or not bridge.enable_command_output
                             or not bridge.require_manual_mode):
            raise ValueError('GUIDED necesita monitor, salida manual y require_manual_mode=true')
        self.session = uuid.uuid4().hex
        self.nav_epoch = 0
        self.permit = uuid.uuid4().hex
        self.samples = {}
        self.source_times = {}
        self.params = {}
        self.version = None
        self.origin = None
        self.tokens = deque(maxlen=30)
        self.owner = None
        self.last_command = None
        self.last_command_at = 0.0
        self.serial = -1
        self.mode_at_last_tick = None
        self.armed_at_last_tick = None
        self.last_reason = 'Salida de secuencia deshabilitada' if not self.enabled else 'Esperando validación'
        self.abort_mode = None
        self.abort_until = 0.0
        self.next_mode_request = 0.0
        self.next_requests = 0.0
        self.status_pub = bridge.create_publisher(String, '/uuv/sequence/bridge_status', 1)
        bridge.create_subscription(String, '/uuv/sequence/setpoint', self.receive, 1)
        bridge.create_subscription(String, '/uuv/sequence/transport_action', self.action, 10)
        self.timer = bridge.create_timer(0.05, self.tick)

    def put(self, key, value, source_time=None):
        now = self.clock()
        if source_time is not None:
            previous = self.source_times.get(key)
            if previous is not None and source_time <= previous:
                if source_time < previous - 1000:
                    self.reset_navigation('Reinicio o retroceso del reloj de telemetría')
                else:
                    return
            self.source_times[key] = source_time
        self.samples[key] = (value, now)

    def reset_navigation(self, reason):
        self.nav_epoch += 1
        self.samples.clear()
        self.source_times.clear()
        self.params.clear()
        self.version = None
        self.next_requests = 0.0
        self.stop(reason, fallback='MANUAL')

    def get(self, key, timeout=0.8):
        item = self.samples.get(key)
        if item is None or not 0 <= self.clock() - item[1] < timeout:
            return None
        return item[0]

    def observe(self, msg):
        if not self.monitor:
            return
        kind = msg.get_type()
        source = (msg.get_srcSystem(), msg.get_srcComponent())
        try:
            if kind == 'VISION_POSITION_DELTA' and source == self.dvl_ids:
                confidence = number(msg.confidence)
                values = list(msg.position_delta) + list(msg.angle_delta)
                for value in values:
                    number(value)
                if not 0 < msg.time_delta_usec <= 1000000 or not 0 <= confidence <= 100:
                    return
                self.put('dvl', confidence, int(msg.time_usec))
                return
            if source != (self.b.target_system_id, self.b.target_component_id):
                return
            if kind == 'LOCAL_POSITION_NED':
                p = [number(msg.x), number(msg.y), number(msg.z)]
                v = [number(msg.vx), number(msg.vy), number(msg.vz)]
                old = self.samples.get('position')
                if old and self.clock() - old[1] < 0.8:
                    dt = max(0, self.clock() - old[1])
                    if math.dist(p, old[0]) > 0.35 + 2.0 * dt:
                        self.reset_navigation('Salto de posición: referencias invalidadas')
                self.put('position', p, int(msg.time_boot_ms))
                self.put('velocity', v, int(msg.time_boot_ms))
            elif kind == 'ATTITUDE':
                self.put('yaw', number(msg.yaw), int(msg.time_boot_ms))
            elif kind == 'SCALED_PRESSURE2':
                self.put('pressure', number(msg.press_abs), int(msg.time_boot_ms))
            elif kind == 'EKF_STATUS_REPORT':
                self.put('ekf', int(msg.flags))
            elif kind == 'SYS_STATUS' and msg.voltage_battery != 65535:
                self.put('voltage', float(msg.voltage_battery) / 1000)
            elif kind == 'GPS_GLOBAL_ORIGIN':
                origin = (int(msg.latitude), int(msg.longitude), int(msg.altitude))
                if self.origin is not None and self.origin != origin:
                    self.reset_navigation('Origen EKF modificado')
                self.origin = origin
            elif kind == 'AUTOPILOT_VERSION':
                ver = int(msg.flight_sw_version)
                self.version = [(ver >> 24) & 255, (ver >> 16) & 255, (ver >> 8) & 255]
            elif kind == 'PARAM_VALUE':
                name = msg.param_id.decode(errors='replace') if isinstance(msg.param_id, bytes) else str(msg.param_id)
                self.params[name.rstrip('\x00')] = (number(msg.param_value), self.clock())
        except (ValueError, TypeError, AttributeError):
            self.last_reason = 'Telemetría mal formada: ' + kind

    def health(self):
        reasons = []
        b = self.b
        if not b.connected or b.last_heartbeat_time is None or self.clock() - b.last_heartbeat_time >= 1.5:
            reasons.append('Sin heartbeat reciente')
        for key in ('position', 'velocity', 'yaw', 'pressure', 'ekf', 'dvl', 'voltage'):
            if self.get(key, 1.5 if key == 'voltage' else 0.8) is None:
                reasons.append('Sin datos recientes: ' + key)
        flags = self.get('ekf') or 0
        if (flags & 7) != 7 or not flags & (8 | 16) or flags & (128 | 1024 | 32768):
            reasons.append('EKF no válido para navegar')
        confidence = self.get('dvl')
        if confidence is not None and confidence < self.min_confidence:
            reasons.append('Confianza del mensaje DVL insuficiente')
        pressure = self.get('pressure')
        if pressure is not None and not 500 <= pressure <= 1500:
            reasons.append('Presión fuera del rango de esta prueba de piscina')
        return reasons

    def output_reasons(self):
        reasons = self.health()
        if not self.enabled:
            reasons.append('Salida real deshabilitada')
        if self.version != [4, 5, 7]:
            reasons.append('Falta confirmar ArduSub 4.5.7 por AUTOPILOT_VERSION')
        values = {k: v for k, (v, stamp) in self.params.items() if self.clock() - stamp < 15}
        if values.get('FS_PILOT_INPUT') != 2:
            reasons.append('Requiere FS_PILOT_INPUT=2; no se cambia automáticamente')
        if not 0.1 <= values.get('FS_PILOT_TIMEOUT', 0) <= 3:
            reasons.append('FS_PILOT_TIMEOUT sin confirmar o fuera de 0.1–3 s')
        if values.get('SYSID_MYGCS') != self.b.source_system_id:
            reasons.append('SYSID_MYGCS no coincide con el emisor ROS')
        if not self.b.motors_enabled:
            reasons.append('Potencia de propulsión no habilitada')
        if self.b.last_cmd_time is None or self.clock() - self.b.last_cmd_time > 0.4:
            reasons.append('Mando sin datos recientes')
        if self.b.deadman:
            reasons.append('RB presionado: prioridad manual')
        return reasons

    def ticket_ok(self, packet):
        if packet.get('session') != self.session or packet.get('permit') != self.permit:
            return False
        return any(token == packet.get('ticket') and 0 <= self.clock() - stamp < 0.4
                   for token, stamp in self.tokens)

    def receive(self, msg):
        try:
            packet = json.loads(msg.data)
            if not isinstance(packet, dict) or not self.ticket_ok(packet):
                return
            reasons = self.output_reasons()
            if reasons or not self.b.armed or self.b.mode != 'GUIDED' or self.abort_mode:
                if self.owner:
                    self.stop('; '.join(reasons) or 'Estado incompatible')
                return
            run = packet['run']
            serial = packet['serial']
            if not isinstance(run, str) or not run or len(run) > 80 or type(serial) is not int:
                raise ValueError('Identificador inválido')
            if self.owner is not None and self.owner != run:
                return
            if serial <= self.serial:
                return
            vn, ve, vd, yaw = [number(v) for v in packet['command']]
            if math.hypot(vn, ve) > 0.200001 or abs(vd) > 0.100001 or abs(yaw) > math.pi + 0.001:
                raise ValueError('Orden fuera de límites')
            self.owner, self.serial = run, serial
            self.last_command = (vn, ve, vd, yaw)
            self.last_command_at = self.clock()
            self.last_reason = 'Envío GUIDED activo'
        except (ValueError, TypeError, KeyError):
            if self.owner:
                self.stop('Paquete de control mal formado')

    def action(self, msg):
        try:
            packet = json.loads(msg.data)
            if not isinstance(packet, dict) or not self.ticket_ok(packet):
                return
            action = packet.get('action')
            if action in ('cancel', 'complete', 'manual'):
                # Cancel is valid even if telemetry is degraded.
                fallback = 'MANUAL' if action == 'manual' else self.holding_mode()
                self.stop('Fin/cancelación solicitada: ' + action, fallback)
                if action == 'manual' and self.enabled and self.b.connected:
                    self.set_mode('MANUAL')
            elif action == 'guided':
                reasons = self.output_reasons()
                if not reasons and not self.owner:
                    self.set_mode('GUIDED')
                else:
                    self.last_reason = '; '.join(reasons) or 'Ya hay ejecución'
        except (ValueError, TypeError, KeyError):
            return

    def set_mode(self, mode):
        ids = {name: mode_id for mode_id, name in self.b.mode_name_by_id.items()}
        self.b.master.mav.set_mode_send(self.b.target_system_id, 1, ids[mode])

    def send(self, command):
        vn, ve, vd, yaw = command
        # Position and acceleration ignored; velocity XYZ and yaw active.
        # Bits: XYZ 1|2|4, acceleration 64|128|256, yaw rate 2048 => 2503.
        self.b.master.mav.set_position_target_local_ned_send(
            0, self.b.target_system_id, self.b.target_component_id, 1, 2503,
            0.0, 0.0, 0.0, vn, ve, vd, 0.0, 0.0, 0.0, yaw, 0.0)

    def holding_mode(self):
        if not self.health():
            return 'POSHOLD'
        flags = self.get('ekf') or 0
        if self.get('pressure') is not None and flags & 1 and flags & 32:
            return 'ALT_HOLD'
        return 'MANUAL'

    def send_stop(self):
        yaw = self.get('yaw')
        if yaw is not None:
            self.send((0.0, 0.0, 0.0, yaw))
        else:
            # Without heading, explicitly command zero yaw rate rather than an old heading.
            self.b.master.mav.set_position_target_local_ned_send(
                0, self.b.target_system_id, self.b.target_component_id, 1, 1479,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    def stop(self, reason, fallback=None):
        owned = self.owner is not None
        self.owner = None
        self.last_command = None
        self.serial = -1
        self.permit = uuid.uuid4().hex
        self.last_reason = reason
        if owned and self.enabled and self.b.mode == 'GUIDED':
            self.abort_mode = fallback or self.holding_mode()
            self.abort_until = self.clock() + 3.0
            self.next_mode_request = 0.0

    def requests(self):
        if not self.b.connected or self.clock() < self.next_requests:
            return
        self.next_requests = self.clock() + 5.0
        mav = self.b.master.mav
        for msg_id, hz in ((32, 10), (30, 10), (137, 5), (193, 5), (1, 2), (33, 5)):
            mav.command_long_send(self.b.target_system_id, self.b.target_component_id,
                                  511, 0, msg_id, int(1e6 / hz), 0, 0, 0, 0, 0)
        for msg_id in (148, 49):
            mav.command_long_send(self.b.target_system_id, self.b.target_component_id,
                                  512, 0, msg_id, 0, 0, 0, 0, 0, 0)
        for param in ('FS_PILOT_INPUT', 'FS_PILOT_TIMEOUT', 'SYSID_MYGCS'):
            mav.param_request_read_send(self.b.target_system_id, self.b.target_component_id,
                                       param.encode('ascii'), -1)

    def tick(self):
        if not self.monitor:
            return
        now = self.clock()
        b = self.b
        try:
            self.requests()
            if b.mode != self.mode_at_last_tick or b.armed != self.armed_at_last_tick:
                self.stop('Cambio de modo/armado; se requiere nueva ejecución')
                self.mode_at_last_tick, self.armed_at_last_tick = b.mode, b.armed
            if self.owner:
                reasons = self.output_reasons()
                if not b.armed or b.mode != 'GUIDED':
                    reasons.append('El ROV dejó GUIDED o se desarmó')
                if now - self.last_command_at > 0.4:
                    reasons.append('Ejecutor sin órdenes recientes')
                if reasons:
                    self.stop('; '.join(reasons), 'MANUAL' if b.deadman else self.holding_mode())
            heartbeat_fresh = b.last_heartbeat_time is not None and now - b.last_heartbeat_time < 1.5
            if self.enabled and b.connected and heartbeat_fresh and b.mode == 'GUIDED':
                # Neutral pilot input is required by ArduSub's pilot watchdog.
                # No joystick axes are passed through while GUIDED owns motion.
                b.master.mav.manual_control_send(b.target_system_id, 0, 0, 500, 0, 0)
                if self.owner:
                    self.send(self.last_command)
                elif self.abort_mode:
                    self.send_stop()
                    if now >= self.next_mode_request:
                        self.set_mode(self.abort_mode)
                        self.next_mode_request = now + 0.5
                    if now > self.abort_until:
                        self.abort_mode = 'MANUAL'
                        self.last_reason = 'Sin confirmación de parada: solicitando MANUAL'
            elif self.abort_mode:
                self.abort_mode = None
            token = uuid.uuid4().hex
            self.tokens.append((token, now))
            packet = {
                'session': self.session, 'nav_epoch': self.nav_epoch,
                'permit': self.permit, 'ticket': token, 'enabled': self.enabled,
                'armed': b.armed, 'mode': b.mode, 'connected': b.connected,
                'health': self.health(), 'blockers': self.output_reasons(),
                'reason': self.last_reason, 'owner': self.owner,
                'version': self.version, 'stopping': self.abort_mode,
                'samples': {k: self.get(k, 1.5 if k == 'voltage' else 0.8) for k in self.samples},
            }
            out = String()
            out.data = json.dumps(packet, allow_nan=False)
            self.status_pub.publish(out)
        except Exception as exc:
            self.stop('Error del transporte: ' + str(exc))
            b.get_logger().error(self.last_reason)

    def shutdown(self):
        if self.enabled and self.b.connected and self.owner and self.b.mode == 'GUIDED':
            self.send_stop()
            self.set_mode(self.holding_mode())
        self.stop('Puente cerrado')
