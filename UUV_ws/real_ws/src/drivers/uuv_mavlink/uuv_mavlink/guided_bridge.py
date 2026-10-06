"""GUIDED transport with a local watchdog, single-owner lease and fresh telemetry."""

from collections import deque
import json
import math
from queue import Empty, Full, Queue
import socket
from threading import Event, Thread
import time
from urllib.request import urlopen
import uuid

from std_msgs.msg import String

from .sequence_core import number


GUIDED_ENTRY_MAX_SPEED_MPS = 0.15
GUIDED_HOLD_MAX_HORIZONTAL_ERROR_M = 0.20
GUIDED_HOLD_MAX_VERTICAL_ERROR_M = 0.12
GUIDED_HOLD_MAX_YAW_ERROR_RAD = math.radians(15.0)


class GuidedBridge:
    def __init__(self, bridge, clock=time.monotonic):
        self.b = bridge
        self.clock = clock
        defaults = {
            'sequence_monitor_enabled': False,
            'enable_sequence_output': False,
            'dvl_source_system': 255,
            'dvl_source_component': 0,
            'dvl_min_confidence': 50.0,
            'dvl_rest_base_url': 'http://192.168.2.2/mavlink2rest/mavlink',
            'dvl_rest_poll_period_s': 0.50,
            'dvl_rest_timeout_s': 0.80,
            'dvl_tcp_host': '192.168.194.95',
            'dvl_tcp_port': 16171,
            'navigation_sample_timeout_s': 2.0,
            'sequence_heartbeat_timeout_s': 2.0,
        }
        for name, value in defaults.items():
            bridge.declare_parameter(name, value)
        self.monitor = bool(bridge.get_parameter('sequence_monitor_enabled').value)
        self.enabled = bool(bridge.get_parameter('enable_sequence_output').value)
        self.dvl_ids = (
            int(bridge.get_parameter('dvl_source_system').value),
            int(bridge.get_parameter('dvl_source_component').value),
        )
        self.min_confidence = float(bridge.get_parameter('dvl_min_confidence').value)
        self.dvl_rest_base = str(bridge.get_parameter('dvl_rest_base_url').value).rstrip('/')
        self.dvl_rest_period = float(bridge.get_parameter('dvl_rest_poll_period_s').value)
        self.dvl_rest_timeout = float(bridge.get_parameter('dvl_rest_timeout_s').value)
        self.dvl_tcp_host = str(bridge.get_parameter('dvl_tcp_host').value).strip()
        self.dvl_tcp_port = int(bridge.get_parameter('dvl_tcp_port').value)
        self.navigation_sample_timeout = float(
            bridge.get_parameter('navigation_sample_timeout_s').value
        )
        self.sequence_heartbeat_timeout = float(
            bridge.get_parameter('sequence_heartbeat_timeout_s').value
        )
        if self.dvl_rest_base and (
            not 0.05 <= self.dvl_rest_period <= 0.5 or not 0.05 <= self.dvl_rest_timeout <= 1.0
        ):
            raise ValueError('Intervalos REST del DVL fuera de límites seguros')
        if self.dvl_tcp_host and not 1 <= self.dvl_tcp_port <= 65535:
            raise ValueError('Puerto TCP del DVL fuera de rango')
        if not 1.0 <= self.navigation_sample_timeout <= 2.5:
            raise ValueError('Caducidad de telemetría fuera de límites seguros')
        if not 1.5 <= self.sequence_heartbeat_timeout <= 3.0:
            raise ValueError('Caducidad de heartbeat fuera de límites seguros')
        if self.enabled and (
            not self.monitor or not bridge.enable_command_output or not bridge.require_manual_mode
        ):
            raise ValueError('GUIDED necesita monitor, salida manual y require_manual_mode=true')
        self.session = uuid.uuid4().hex
        self.nav_epoch = 0
        self.permit = uuid.uuid4().hex
        self.samples = {}
        self.source_times = {}
        self.params = {}
        self.dvl_input = None
        self.dvl_rest_marker = None
        self.dvl_rest_queue = Queue(maxsize=1)
        self.dvl_rest_stop = Event()
        self.dvl_rest_thread = None
        self.dvl_tcp_queue = Queue(maxsize=1)
        self.dvl_tcp_stop = Event()
        self.dvl_tcp_thread = None
        self.dvl_tcp_seen = False
        self.dvl_tcp_valid = None
        self.dvl_tcp_at = 0.0
        self.dvl_altitude = None
        self.version = None
        self.origin = None
        self.tokens = deque(maxlen=30)
        self.owner = None
        self.owner_limits = None
        self.last_command = None
        self.last_command_at = 0.0
        self.hold_target = None
        self.holding_completed_goal = False
        self.guided_request_at = None
        self.guided_settle_started = None
        self.guided_entry_reference = None
        self.serial = -1
        self.mode_at_last_tick = None
        self.armed_at_last_tick = None
        self.last_reason = (
            'Salida de secuencia deshabilitada' if not self.enabled else 'Esperando validación'
        )
        self.abort_mode = None
        self.abort_until = 0.0
        self.next_mode_request = 0.0
        self.next_requests = 0.0
        self.status_pub = bridge.create_publisher(String, '/uuv/sequence/bridge_status', 1)
        bridge.create_subscription(String, '/uuv/sequence/setpoint', self.receive, 1)
        bridge.create_subscription(String, '/uuv/sequence/transport_action', self.action, 10)
        self.timer = bridge.create_timer(0.05, self.tick)
        if self.monitor and self.dvl_rest_base:
            self.dvl_rest_thread = Thread(
                target=self._dvl_rest_loop, name='uuv-dvl-rest', daemon=True
            )
            self.dvl_rest_thread.start()
        if self.monitor and self.dvl_tcp_host:
            self.dvl_tcp_thread = Thread(
                target=self._dvl_tcp_loop, name='uuv-dvl-tcp', daemon=True
            )
            self.dvl_tcp_thread.start()

    def put(self, key, value, source_time=None):
        now = self.clock()
        if source_time is not None:
            previous = self.source_times.get(key)
            if previous is not None and source_time <= previous:
                if source_time < previous - 1000:
                    if key == 'dvl':
                        # El DVL es diagnóstico. Un reinicio de su reloj no debe
                        # retirar la autoridad de movimiento a ArduSub.
                        self.samples.pop('dvl', None)
                        self.source_times.pop('dvl', None)
                        self.last_reason = (
                            'DVL reiniciado; diagnóstico temporalmente no disponible'
                        )
                        return
                    else:
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
        self.hold_target = None
        self.clear_guided_entry()
        self.next_requests = 0.0
        self.stop(reason, fallback='MANUAL')

    def get(self, key, timeout=0.8):
        item = self.samples.get(key)
        if item is None or not 0 <= self.clock() - item[1] < timeout:
            return None
        return item[0]

    def accept_dvl(
        self,
        confidence,
        position_delta,
        angle_delta,
        time_delta_usec,
        source_time=None,
        transport=None,
    ):
        confidence = number(confidence)
        values = list(position_delta) + list(angle_delta)
        for value in values:
            number(value)
        delta = int(time_delta_usec)
        if not 0 < delta <= 1000000 or not 0 <= confidence <= 100:
            return False
        if source_time is not None:
            source_time = int(source_time)
            if source_time < 0:
                return False
            # Water Linked BlueOS v1.0.10 sends time_usec=0 for every delta.
            # Zero supplies no acquisition timestamp: freshness then means
            # local packet arrival, not measurement age or replay protection.
            # Positive timestamps retain duplicate/clock-reset checks.
            source_time = source_time if source_time > 0 else None
        self.put('dvl', confidence, source_time)
        self.dvl_input = transport
        return True

    def observe_rest_dvl(self, payload):
        """Validate one Mavlink2Rest snapshot; only an advancing counter is live."""
        try:
            message = payload['message']
            status = payload['status']['time']
            if message.get('type') != 'VISION_POSITION_DELTA':
                return
            counter = status['counter']
            updated = status['last_update']
            if (
                type(counter) is not int
                or counter < 0
                or not isinstance(updated, str)
                or not updated
            ):
                return
            marker = (counter, updated)
            previous = self.dvl_rest_marker
            self.dvl_rest_marker = marker
            # A single cached HTTP response is not proof of a live DVL. Require
            # a second, newer sample before satisfying the navigation guard.
            if previous is None:
                return
            if counter < previous[0]:
                self.samples.pop('dvl', None)
                self.source_times.pop('dvl', None)
                self.last_reason = 'Reinicio del contador DVL; movimiento no interrumpido'
                return
            if counter == previous[0] or updated == previous[1]:
                return
            self.accept_dvl(
                message['confidence'],
                message['position_delta'],
                message['angle_delta'],
                message['time_delta_usec'],
                message.get('time_usec'),
                'mavlink2rest',
            )
        except (KeyError, TypeError, ValueError, OverflowError):
            return

    def drain_rest_dvl(self):
        while True:
            try:
                payload = self.dvl_rest_queue.get_nowait()
            except Empty:
                return
            self.observe_rest_dvl(payload)

    def _queue_rest_dvl(self, payload):
        try:
            self.dvl_rest_queue.put_nowait(payload)
        except Full:
            try:
                self.dvl_rest_queue.get_nowait()
            except Empty:
                pass
            try:
                self.dvl_rest_queue.put_nowait(payload)
            except Full:
                pass

    def _dvl_rest_loop(self):
        system, component = self.dvl_ids
        url = (
            f'{self.dvl_rest_base}/vehicles/{system}/components/{component}'
            '/messages/VISION_POSITION_DELTA'
        )
        while not self.dvl_rest_stop.is_set():
            started = time.monotonic()
            try:
                with urlopen(url, timeout=self.dvl_rest_timeout) as response:
                    if getattr(response, 'status', 200) != 200:
                        raise ValueError('HTTP DVL no disponible')
                    self._queue_rest_dvl(json.load(response))
            except Exception:
                # The normal freshness guard reports loss of data. Network
                # failures must never block this thread or the ROS watchdog.
                pass
            elapsed = time.monotonic() - started
            self.dvl_rest_stop.wait(max(0.0, self.dvl_rest_period - elapsed))

    def _queue_tcp_dvl(self, payload):
        try:
            self.dvl_tcp_queue.put_nowait(payload)
        except Full:
            try:
                self.dvl_tcp_queue.get_nowait()
            except Empty:
                pass
            try:
                self.dvl_tcp_queue.put_nowait(payload)
            except Full:
                pass

    def _dvl_tcp_loop(self):
        """Read the A50 persistent JSON stream without opening a socket per sample."""
        while not self.dvl_tcp_stop.is_set():
            try:
                with socket.create_connection(
                    (self.dvl_tcp_host, self.dvl_tcp_port), timeout=1.0
                ) as stream:
                    stream.settimeout(0.5)
                    buffer = b''
                    while not self.dvl_tcp_stop.is_set():
                        try:
                            chunk = stream.recv(65536)
                        except socket.timeout:
                            continue
                        if not chunk:
                            break
                        buffer += chunk
                        while b'\n' in buffer:
                            line, buffer = buffer.split(b'\n', 1)
                            try:
                                payload = json.loads(line)
                            except (ValueError, UnicodeDecodeError):
                                continue
                            if payload.get('type') == 'velocity':
                                self._queue_tcp_dvl(payload)
            except OSError:
                pass
            self.dvl_tcp_stop.wait(0.5)

    def drain_tcp_dvl(self):
        while True:
            try:
                payload = self.dvl_tcp_queue.get_nowait()
            except Empty:
                return
            try:
                valid = payload['velocity_valid']
                altitude = number(payload['altitude'])
                if type(valid) is not bool:
                    continue
                self.dvl_tcp_seen = True
                self.dvl_tcp_valid = valid
                self.dvl_tcp_at = self.clock()
                self.dvl_altitude = altitude if altitude >= 0 else None
            except (KeyError, TypeError, ValueError):
                continue

    def observe(self, msg):
        if not self.monitor:
            return
        kind = msg.get_type()
        source = (msg.get_srcSystem(), msg.get_srcComponent())
        try:
            if kind == 'VISION_POSITION_DELTA' and source == self.dvl_ids:
                self.accept_dvl(
                    msg.confidence,
                    msg.position_delta,
                    msg.angle_delta,
                    msg.time_delta_usec,
                    msg.time_usec,
                    'udp',
                )
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
                name = (
                    msg.param_id.decode(errors='replace')
                    if isinstance(msg.param_id, bytes)
                    else str(msg.param_id)
                )
                self.params[name.rstrip('\x00')] = (number(msg.param_value), self.clock())
        except (ValueError, TypeError, AttributeError):
            self.last_reason = 'Telemetría mal formada: ' + kind

    def health(self):
        """Return sensor diagnostics; these do not own motion authority."""
        reasons = []
        for key in ('position', 'velocity', 'yaw', 'pressure', 'ekf', 'dvl', 'voltage'):
            timeout = 1.5 if key == 'dvl' else self.navigation_sample_timeout
            if self.get(key, timeout) is None:
                reasons.append('Sin datos recientes: ' + key)
        flags = self.get('ekf', self.navigation_sample_timeout) or 0
        if (flags & 7) != 7 or not flags & (8 | 16) or flags & (128 | 1024 | 32768):
            reasons.append('EKF no válido para navegar')
        confidence = self.get('dvl')
        if confidence is not None and confidence < self.min_confidence:
            reasons.append('Confianza del mensaje DVL insuficiente')
        if self.dvl_tcp_seen:
            if self.clock() - self.dvl_tcp_at >= 0.5:
                reasons.append('Sin estado directo reciente del DVL')
            elif self.dvl_tcp_valid is not True:
                reasons.append('DVL reporta velocidad inválida')
        pressure = self.get('pressure')
        if pressure is not None and not 500 <= pressure <= 1500:
            reasons.append('Presión fuera del rango de esta prueba de piscina')
        return reasons

    def control_health(self):
        """Faults that really prevent delivery of a controlled setpoint."""
        b = self.b
        reasons = []
        if (
            not b.connected
            or b.last_heartbeat_time is None
            or self.clock() - b.last_heartbeat_time >= self.sequence_heartbeat_timeout
        ):
            reasons.append('Sin heartbeat reciente')
        pressure = self.get('pressure', self.navigation_sample_timeout)
        if pressure is not None and not 500 <= pressure <= 1500:
            reasons.append('Presión fuera del rango de esta prueba de piscina')
        return reasons

    def output_reasons(self):
        reasons = self.control_health()
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
        if self.guided_request_at is not None:
            reasons.append('Esperando confirmación real de GUIDED')
        elif self.guided_settle_started is not None:
            reasons.append('Estabilizando entrada a GUIDED')
        return reasons

    def ticket_ok(self, packet):
        if packet.get('session') != self.session or packet.get('permit') != self.permit:
            return False
        return any(
            token == packet.get('ticket') and 0 <= self.clock() - stamp < 0.4
            for token, stamp in self.tokens
        )

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
            declared = tuple(
                number(packet['limits'][key])
                for key in ('horizontal_speed', 'vertical_speed', 'yaw_speed')
            )
            if any(value <= 0 for value in declared):
                raise ValueError('Límites de movimiento no positivos')
            if self.owner is not None and declared != self.owner_limits:
                raise ValueError('Límites modificados durante la ejecución')
            horizontal, vertical, _ = declared
            if (
                math.hypot(vn, ve) > horizontal + 1e-6
                or abs(vd) > vertical + 1e-6
                or abs(yaw) > math.pi + 0.001
            ):
                raise ValueError('Orden fuera de límites')
            self.owner, self.serial = run, serial
            self.owner_limits = declared
            self.last_command = (vn, ve, vd, yaw)
            self.last_command_at = self.clock()
            self.holding_completed_goal = False
            # The entry guard belongs to the idle hold.  Once a fresh command
            # owner is accepted, the executor's limits and watchdog take over.
            self.clear_guided_entry()
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
            if action == 'complete' and packet.get('hold_target') is not None:
                if self.complete_hold(packet['hold_target']):
                    return
            if action in ('cancel', 'complete', 'manual'):
                # Cancel is valid even if telemetry is degraded.
                self.clear_guided_entry()
                fallback = 'MANUAL' if action == 'manual' else self.holding_mode()
                self.stop('Fin/cancelación solicitada: ' + action, fallback)
                if action == 'manual' and self.enabled and self.b.connected:
                    self.set_mode('MANUAL')
            elif action == 'guided':
                reasons = self.output_reasons()
                if self.b.mode != 'MANUAL':
                    reasons.append('La entrada segura a GUIDED debe comenzar en MANUAL')
                if not reasons and not self.owner:
                    if self.capture_hold_target():
                        fault = self.guided_request_fault()
                        if fault:
                            self.hold_target = None
                            self.last_reason = 'GUIDED rechazado: ' + fault
                        else:
                            # Remember the MANUAL pose only as a transition
                            # guard. Never expose it as a GUIDED position target:
                            # the vehicle may move slightly before the mode
                            # heartbeat arrives, and ArduSub may still retain a
                            # target from an earlier GUIDED session.
                            manual_reference = self.hold_target
                            self.hold_target = None
                            self.guided_request_at = self.clock()
                            self.guided_settle_started = None
                            self.guided_entry_reference = manual_reference

                            # Preload a neutral velocity command, request the
                            # mode, then immediately overwrite any retained
                            # GUIDED target with more neutral commands. Position
                            # hold is enabled only after the GUIDED heartbeat and
                            # a fresh pose/velocity sample have been received.
                            self.send_zero_velocity()
                            self.send_zero_velocity()
                            self.set_mode('GUIDED')
                            self.send_zero_velocity()
                            self.send_zero_velocity()
                            self.last_reason = (
                                'GUIDED solicitado con frenado neutro; '
                                'esperando confirmación para capturar la pose actual'
                            )
                    else:
                        self.last_reason = 'GUIDED rechazado: posición o rumbo no disponibles'
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
            0,
            self.b.target_system_id,
            self.b.target_component_id,
            1,
            2503,
            0.0,
            0.0,
            0.0,
            vn,
            ve,
            vd,
            0.0,
            0.0,
            0.0,
            yaw,
            0.0,
        )

    def holding_mode(self):
        if not self.control_health() and not self.health():
            return 'POSHOLD'
        flags = self.get('ekf') or 0
        if self.get('pressure') is not None and flags & 1 and flags & 32:
            return 'ALT_HOLD'
        return 'MANUAL'

    def complete_hold(self, target):
        """Retain a close final NED target after a healthy positional run."""
        if (
            not self.owner or not self.enabled or not self.b.connected
            or not self.b.armed or self.b.mode != 'GUIDED'
            or self.abort_mode or self.output_reasons() or self.completed_hold_health()
        ):
            return False
        position = self.get('position', min(0.5, self.navigation_sample_timeout))
        yaw = self.get('yaw', min(0.5, self.navigation_sample_timeout))
        if position is None or yaw is None:
            return False
        try:
            if not isinstance(target, (list, tuple)) or len(target) != 4:
                return False
            n, e, d, heading = (number(v) for v in target)
            heading_error = abs((heading - yaw + math.pi) % (2 * math.pi) - math.pi)
            # Accept only a nearby completion correction, never a new distant
            # waypoint embedded in a completion message.
            if (
                abs(heading) > math.pi + 0.001
                or math.hypot(n - position[0], e - position[1]) > 0.25
                or abs(d - position[2]) > 0.15
                or heading_error > math.radians(10)
            ):
                return False
        except (TypeError, ValueError, IndexError):
            return False
        self.hold_target = (n, e, d, heading)
        self.holding_completed_goal = True
        self.owner = None
        self.owner_limits = None
        self.last_command = None
        self.serial = -1
        self.permit = uuid.uuid4().hex
        self.clear_guided_entry()
        self.last_reason = 'Secuencia completada; manteniendo objetivo final en GUIDED'
        return True

    def completed_hold_health(self):
        """Check the navigation needed by final hold, not DVL diagnostics."""
        reasons = [
            'Sin datos recientes para retención: ' + key
            for key in ('position', 'velocity', 'yaw', 'pressure', 'ekf')
            if self.get(key, min(0.8, self.navigation_sample_timeout)) is None
        ]
        flags = self.get('ekf', min(0.8, self.navigation_sample_timeout)) or 0
        if (flags & 7) != 7 or not flags & (8 | 16) or flags & (128 | 1024 | 32768):
            reasons.append('EKF no válido para retener posición')
        return reasons

    def capture_hold_target(self):
        position = self.get('position', self.navigation_sample_timeout)
        yaw = self.get('yaw', self.navigation_sample_timeout)
        if position is None or yaw is None or len(position) != 3:
            self.hold_target = None
            return False
        try:
            self.hold_target = tuple(number(v) for v in position) + (number(yaw),)
        except (ValueError, TypeError):
            self.hold_target = None
            return False
        return True

    def hold_target_text(self):
        if self.hold_target is None:
            return 'objetivo no disponible'
        n, e, d, yaw = self.hold_target
        return f'N={n:.3f} m, E={e:.3f} m, D={d:.3f} m, yaw={math.degrees(yaw):.2f}°'

    def clear_guided_entry(self):
        self.guided_request_at = None
        self.guided_settle_started = None
        self.guided_entry_reference = None

    def guided_request_fault(self):
        """Reject mode entry if the vehicle is already translating."""
        velocity = self.get('velocity', self.navigation_sample_timeout)
        if velocity is None or len(velocity) != 3:
            return 'velocidad no disponible'
        try:
            horizontal_speed = math.hypot(number(velocity[0]), number(velocity[1]))
            vertical_speed = abs(number(velocity[2]))
        except (ValueError, TypeError):
            return 'velocidad no válida'
        if horizontal_speed > GUIDED_ENTRY_MAX_SPEED_MPS:
            return f'ROV en movimiento horizontal: {horizontal_speed:.3f} m/s'
        if vertical_speed > GUIDED_ENTRY_MAX_SPEED_MPS:
            return f'ROV en movimiento vertical: {vertical_speed:.3f} m/s'
        return None

    def confirm_guided_entry(self, now):
        """Capture a fresh stationary pose only after GUIDED is confirmed."""
        if not self.capture_hold_target():
            self.abort_guided_entry('posición o rumbo no disponibles', now)
            return False
        fault = self.guided_request_fault()
        if fault:
            self.abort_guided_entry(fault, now)
            return False
        self.guided_request_at = None
        self.guided_settle_started = now
        self.guided_entry_reference = self.hold_target
        self.last_reason = (
            'GUIDED confirmado; pose actual capturada y retenida ('
            + self.hold_target_text()
            + ')'
        )
        return True

    def guided_entry_fault(self):
        """Detect runaway motion, while allowing normal small hold corrections."""
        if self.guided_entry_reference is None:
            return None
        position = self.get('position', self.navigation_sample_timeout)
        velocity = self.get('velocity', self.navigation_sample_timeout)
        yaw = self.get('yaw', self.navigation_sample_timeout)
        if (
            position is None
            or velocity is None
            or yaw is None
            or len(position) != 3
            or len(velocity) != 3
        ):
            return 'Telemetría de transición no disponible'
        try:
            horizontal_error = math.hypot(
                number(position[0]) - self.guided_entry_reference[0],
                number(position[1]) - self.guided_entry_reference[1],
            )
            depth_error = abs(number(position[2]) - self.guided_entry_reference[2])
            horizontal_speed = math.hypot(number(velocity[0]), number(velocity[1]))
            vertical_speed = abs(number(velocity[2]))
            yaw_error = abs(
                (number(yaw) - self.guided_entry_reference[3] + math.pi) % (2.0 * math.pi)
                - math.pi
            )
        except (ValueError, TypeError):
            return 'Telemetría de transición no válida'
        if horizontal_error > GUIDED_HOLD_MAX_HORIZONTAL_ERROR_M:
            return f'Desviación horizontal durante entrada GUIDED: {horizontal_error:.3f} m'
        if depth_error > GUIDED_HOLD_MAX_VERTICAL_ERROR_M:
            return f'Desviación vertical durante entrada GUIDED: {depth_error:.3f} m'
        if yaw_error > GUIDED_HOLD_MAX_YAW_ERROR_RAD:
            return f'Desviación de rumbo durante entrada GUIDED: {math.degrees(yaw_error):.1f}°'
        if horizontal_speed > GUIDED_ENTRY_MAX_SPEED_MPS:
            return f'Velocidad horizontal durante entrada GUIDED: {horizontal_speed:.3f} m/s'
        if vertical_speed > GUIDED_ENTRY_MAX_SPEED_MPS:
            return f'Velocidad vertical durante entrada GUIDED: {vertical_speed:.3f} m/s'
        return None

    def abort_guided_entry(self, reason, now):
        self.clear_guided_entry()
        # Never command the pose that caused the transition fault while the
        # MANUAL fallback is still pending. Zero velocity is the only safe
        # transient command after an entry abort.
        self.hold_target = None
        self.last_reason = 'GUIDED abortado: ' + reason
        self.abort_mode = 'MANUAL'
        self.abort_until = now + 3.0
        self.next_mode_request = 0.0

    def send_hold(self):
        if self.hold_target is None and not self.capture_hold_target():
            return False
        n, e, d, yaw = self.hold_target
        # Stream one coherent position/yaw target. Repeated velocity-to-position
        # alternation must not replace the reference on every hold cycle.
        # 2552: use XYZ position and yaw; ignore velocity, acceleration and
        #       yaw rate.
        self.b.master.mav.set_position_target_local_ned_send(
            0,
            self.b.target_system_id,
            self.b.target_component_id,
            1,
            2552,
            n,
            e,
            d,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            yaw,
            0.0,
        )
        return True

    def send_stop(self):
        if not self.send_hold():
            # Without a trustworthy pose, command zero velocity and zero yaw rate.
            self.send_zero_velocity()

    def send_zero_velocity(self):
        self.b.master.mav.set_position_target_local_ned_send(
            0,
            self.b.target_system_id,
            self.b.target_component_id,
            1,
            1479,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )

    def stop(self, reason, fallback=None):
        owned = self.owner is not None or self.holding_completed_goal
        self.holding_completed_goal = False
        self.owner = None
        self.owner_limits = None
        self.last_command = None
        self.serial = -1
        self.permit = uuid.uuid4().hex
        self.last_reason = reason
        if owned and self.enabled and self.b.mode == 'GUIDED':
            # Freeze the position reached at cancellation instead of retaining
            # an old pre-run target while the fallback mode is requested.
            self.capture_hold_target()
            self.abort_mode = fallback or self.holding_mode()
            self.abort_until = self.clock() + 3.0
            self.next_mode_request = 0.0

    def requests(self):
        if not self.b.connected or self.clock() < self.next_requests:
            return
        self.next_requests = self.clock() + 5.0
        mav = self.b.master.mav
        for msg_id, hz in ((32, 10), (30, 10), (137, 5), (193, 5), (1, 2), (33, 5)):
            mav.command_long_send(
                self.b.target_system_id,
                self.b.target_component_id,
                511,
                0,
                msg_id,
                int(1e6 / hz),
                0,
                0,
                0,
                0,
                0,
            )
        for msg_id in (148, 49):
            mav.command_long_send(
                self.b.target_system_id,
                self.b.target_component_id,
                512,
                0,
                msg_id,
                0,
                0,
                0,
                0,
                0,
                0,
            )
        for param in ('FS_PILOT_INPUT', 'FS_PILOT_TIMEOUT', 'SYSID_MYGCS'):
            mav.param_request_read_send(
                self.b.target_system_id, self.b.target_component_id, param.encode('ascii'), -1
            )

    def tick(self):
        if not self.monitor:
            return
        now = self.clock()
        b = self.b
        try:
            self.drain_rest_dvl()
            self.drain_tcp_dvl()
            self.requests()
            previous_mode = self.mode_at_last_tick
            if b.mode != previous_mode or b.armed != self.armed_at_last_tick:
                self.stop('Cambio de modo/armado; se requiere nueva ejecución')
                self.mode_at_last_tick, self.armed_at_last_tick = b.mode, b.armed
                if b.mode != 'GUIDED' and self.guided_request_at is None:
                    self.guided_settle_started = None
                    self.guided_entry_reference = None
                elif (
                    previous_mode is not None
                    and b.mode == 'GUIDED'
                    and self.guided_request_at is None
                    and not self.owner
                    and not self.abort_mode
                ):
                    # An external mode change bypasses the neutral-command
                    # handshake. Reject it instead of letting ArduSub act on a
                    # target retained from an earlier GUIDED session.
                    self.abort_guided_entry(
                        'entrada externa sin protocolo seguro; usa la interfaz de secuencias',
                        now,
                    )
                    self.send_zero_velocity()
                    self.set_mode('MANUAL')
                    self.next_mode_request = now + 0.5
            if self.guided_request_at is not None:
                # While the mode heartbeat is pending, stream only a neutral
                # velocity target. No position from this or an earlier entry is
                # allowed to become active during the transition.
                self.send_zero_velocity()
                self.send_zero_velocity()
                # The MANUAL pose is only a diagnostic reference. Small drift
                # is expected in water and must not force repeated attempts;
                # only an unsafe measured speed blocks the transition. The
                # actual hold pose is recaptured after GUIDED is confirmed.
                fault = self.guided_request_fault()
                if fault:
                    self.abort_guided_entry(fault, now)
                    self.send_zero_velocity()
                    self.set_mode('MANUAL')
                    self.next_mode_request = now + 0.5
                elif b.mode == 'GUIDED':
                    if self.confirm_guided_entry(now):
                        # Start one coherent position/yaw hold from the fresh
                        # post-confirmation pose. Do not alternate it with a
                        # velocity target after this point.
                        self.send_hold()
                        self.send_hold()
                    else:
                        self.send_zero_velocity()
                        self.set_mode('MANUAL')
                        self.next_mode_request = now + 0.5
                elif now - self.guided_request_at > 2.0:
                    self.abort_guided_entry('GUIDED no confirmado dentro de 2 s', now)
                    self.send_zero_velocity()
                    self.set_mode('MANUAL')
                    self.next_mode_request = now + 0.5
            if self.owner:
                reasons = self.output_reasons()
                if not b.armed or b.mode != 'GUIDED':
                    reasons.append('El ROV dejó GUIDED o se desarmó')
                if now - self.last_command_at > 0.4:
                    reasons.append('Ejecutor sin órdenes recientes')
                if reasons:
                    self.stop('; '.join(reasons), 'MANUAL' if b.deadman else self.holding_mode())
            elif self.holding_completed_goal:
                reasons = self.output_reasons() + self.completed_hold_health()
                if reasons:
                    self.stop('; '.join(reasons), 'MANUAL' if b.deadman else self.holding_mode())
            heartbeat_fresh = (
                b.last_heartbeat_time is not None and now - b.last_heartbeat_time < 1.5
            )
            if self.enabled and b.connected and heartbeat_fresh and b.mode == 'GUIDED':
                # Neutral pilot input is required by ArduSub's pilot watchdog.
                # No joystick axes are passed through while GUIDED owns motion.
                b.master.mav.manual_control_send(b.target_system_id, 0, 0, 500, 0, 0)
                if self.owner:
                    self.send(self.last_command)
                elif self.abort_mode:
                    # During a fallback, never pull the vehicle back toward a
                    # position target. Brake translation/yaw until MANUAL (or
                    # the selected fallback mode) is confirmed.
                    self.send_zero_velocity()
                    if now >= self.next_mode_request:
                        self.set_mode(self.abort_mode)
                        self.next_mode_request = now + 0.5
                    if now > self.abort_until:
                        self.abort_mode = 'MANUAL'
                        self.last_reason = 'Sin confirmación de parada: solicitando MANUAL'
                else:
                    # GUIDED sin ejecución debe tener siempre un setpoint definido.
                    # Mantiene la posición N/E/D capturada y el rumbo actual.
                    self.send_stop()
                    if self.guided_entry_reference is not None:
                        fault = self.guided_entry_fault()
                        if fault:
                            self.abort_guided_entry(fault, now)
                            self.set_mode('MANUAL')
                            self.next_mode_request = now + 0.5
                        elif (
                            self.guided_settle_started is not None
                            and now - self.guided_settle_started >= 5.0
                        ):
                            self.guided_settle_started = None
                            self.last_reason = 'GUIDED estable; listo para iniciar'
            elif self.abort_mode:
                self.abort_mode = None
            token = uuid.uuid4().hex
            self.tokens.append((token, now))
            packet = {
                'session': self.session,
                'nav_epoch': self.nav_epoch,
                'permit': self.permit,
                'ticket': token,
                'enabled': self.enabled,
                'armed': b.armed,
                'mode': b.mode,
                'connected': b.connected,
                'health': self.control_health(),
                'diagnostics': self.health(),
                'blockers': self.output_reasons(),
                'reason': self.last_reason,
                'owner': self.owner,
                'active_motion_limits': self.owner_limits,
                'paused': False,
                'pause_elapsed': None,
                'pause_reason': None,
                'dvl_velocity_valid': self.dvl_tcp_valid if self.dvl_tcp_seen else None,
                'dvl_altitude': self.dvl_altitude,
                'version': self.version,
                'stopping': self.abort_mode,
                'guided_entry_pending': self.guided_request_at is not None,
                'guided_settling': self.guided_settle_started is not None,
                'hold_target': list(self.hold_target) if self.hold_target is not None else None,
                'holding_completed_goal': self.holding_completed_goal,
                'controller_revision': 'guided_precision_v2',
                'dvl_input': self.dvl_input,
                'samples': {
                    k: self.get(k, 1.5 if k == 'dvl' else self.navigation_sample_timeout)
                    for k in self.samples
                },
            }
            out = String()
            out.data = json.dumps(packet, allow_nan=False)
            self.status_pub.publish(out)
        except Exception as exc:
            self.stop('Error del transporte: ' + str(exc))
            b.get_logger().error(self.last_reason)

    def shutdown(self):
        if (
            self.enabled and self.b.connected
            and (self.owner or self.holding_completed_goal) and self.b.mode == 'GUIDED'
        ):
            self.send_stop()
            self.set_mode(self.holding_mode())
        self.dvl_rest_stop.set()
        if self.dvl_rest_thread is not None:
            self.dvl_rest_thread.join(timeout=self.dvl_rest_timeout + 0.2)
        self.dvl_tcp_stop.set()
        if self.dvl_tcp_thread is not None:
            self.dvl_tcp_thread.join(timeout=1.0)
        self.stop('Puente cerrado')
