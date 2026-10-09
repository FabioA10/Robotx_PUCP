"""Restricted MAVLink parameter read/write bridge with verified rollback."""

from datetime import datetime, timezone
import json
import time

from std_msgs.msg import String

from .parameter_protocol import (
    normalize_observed_parameter_values,
    normalize_parameter_values,
    parameter_name,
    PARAMETER_NAMES,
    parameter_sets_match,
    parameter_type,
    READ_ONLY_PARAMETER_NAMES,
    REQUEST_PARAMETER_NAMES,
    write_blockers,
)


class AutopilotParameterManager:
    """Expose five bounded ArduSub parameters through a guarded ROS protocol."""

    def __init__(self, bridge, clock=time.monotonic):
        self.bridge = bridge
        self.clock = clock
        bridge.declare_parameter('enable_parameter_write', False)
        self.write_enabled = bool(bridge.get_parameter('enable_parameter_write').value)
        self.publisher = bridge.create_publisher(String, '/uuv/parameters/status', 10)
        bridge.create_subscription(
            String,
            '/uuv/parameters/request',
            self.receive_request,
            10,
        )
        self.values = {}
        self.value_times = {}
        self.value_types = {}
        self.read_only_values = {}
        self.read_only_value_times = {}
        self.last_snapshot = None
        self.last_result = 'idle'
        self.last_request_id = None
        self.note = 'Esperando conexión para leer los parámetros del vehículo.'
        self.pending_apply = None
        self.transaction = None
        self.last_publish = 0.0
        self.was_connected = False
        self.timer = bridge.create_timer(0.10, self.tick)

    def _publish(self, force=False):
        now = self.clock()
        if not force and now - self.last_publish < 0.50:
            return
        blockers = self.blockers()
        payload = {
            'connected': bool(self.bridge.connected),
            'armed': bool(self.bridge.armed),
            'mode': str(self.bridge.mode),
            'write_enabled': self.write_enabled,
            'safe_to_write': not blockers,
            'blockers': blockers,
            'busy': self.pending_apply is not None or self.transaction is not None,
            'phase': self._phase(),
            'result': self.last_result,
            'request_id': self.last_request_id,
            'note': self.note,
            'values': {
                name: self.values[name]
                for name in PARAMETER_NAMES
                if name in self.values
            },
            'parameter_types': {
                name: self.value_types[name]
                for name in PARAMETER_NAMES
                if name in self.value_types
            },
            'read_only_values': {
                name: self.read_only_values[name]
                for name in READ_ONLY_PARAMETER_NAMES
                if name in self.read_only_values
            },
            'complete': all(
                name in self.values and name in self.value_types
                for name in PARAMETER_NAMES
            ),
            'monitor_complete': all(
                name in self.read_only_values for name in READ_ONLY_PARAMETER_NAMES
            ),
            'snapshot': self.last_snapshot,
            'snapshot_captured_at': getattr(self, 'snapshot_captured_at', None),
            'vehicle': self._vehicle_identity(),
        }
        message = String()
        message.data = json.dumps(payload, allow_nan=False, sort_keys=True)
        self.publisher.publish(message)
        self.last_publish = now

    def _phase(self):
        if self.pending_apply is not None:
            return 'snapshot'
        if self.transaction is not None:
            return self.transaction['phase']
        return 'idle'

    def _heartbeat_fresh(self):
        heartbeat = self.bridge.last_heartbeat_time
        return (
            heartbeat is not None
            and self.clock() - heartbeat < self.bridge.heartbeat_timeout
        )

    def _vehicle_identity(self):
        guided = getattr(self.bridge, 'guided', None)
        version = getattr(guided, 'version', None) if guided is not None else None
        return {
            'system_id': int(self.bridge.target_system_id),
            'component_id': int(self.bridge.target_component_id),
            'firmware': (
                'ArduSub ' + '.'.join(str(part) for part in version)
                if isinstance(version, (list, tuple)) and len(version) >= 3
                else 'ArduSub no confirmado'
            ),
        }

    def _sequence_active(self):
        guided = getattr(self.bridge, 'guided', None)
        if guided is None:
            return False
        return bool(
            getattr(guided, 'owner', False)
            or getattr(guided, 'holding_completed_goal', False)
            or getattr(guided, 'guided_request_at', None) is not None
            or getattr(guided, 'abort_mode', None)
        )

    def blockers(self):
        """Return current independent write blockers."""
        reasons = write_blockers(
            write_enabled=self.write_enabled,
            connected=bool(self.bridge.connected),
            heartbeat_fresh=self._heartbeat_fresh(),
            armed=bool(self.bridge.armed),
            mode=str(self.bridge.mode),
            deadman=bool(self.bridge.deadman),
            sequence_active=self._sequence_active(),
        )
        guided = getattr(self.bridge, 'guided', None)
        version = getattr(guided, 'version', None) if guided is not None else None
        if version != [4, 5, 7]:
            reasons.append('firmware ArduSub 4.5.7 todavía no confirmado')
        return reasons

    def _request_all(self):
        for name in REQUEST_PARAMETER_NAMES:
            self.bridge.master.mav.param_request_read_send(
                self.bridge.target_system_id,
                self.bridge.target_component_id,
                name.encode('ascii'),
                -1,
            )

    def _clear_observations(self):
        """Prevent a reconnect from presenting values read from an older link."""
        self.values.clear()
        self.value_times.clear()
        self.value_types.clear()
        self.read_only_values.clear()
        self.read_only_value_times.clear()

    def receive_request(self, message):
        """Accept read/apply/restore requests from the operator dashboard."""
        try:
            packet = json.loads(message.data)
            if not isinstance(packet, dict):
                raise ValueError('la solicitud no es un objeto')
            action = str(packet.get('action', '')).strip().lower()
            request_id = str(packet.get('request_id', '')).strip()
            if not request_id:
                raise ValueError('falta request_id')
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            self.last_result = 'rejected'
            self.note = f'Solicitud de parámetros inválida: {exc}'
            self._publish(True)
            return

        if self.pending_apply is not None or self.transaction is not None:
            self.last_result = 'rejected'
            self.note = 'Ya existe una operación de parámetros en curso.'
            self._publish(True)
            return

        self.last_request_id = request_id
        if action == 'read':
            self.last_result = 'reading'
            self.note = 'Leyendo los cinco parámetros directamente del autopiloto…'
            self._request_all()
            self._publish(True)
            return
        if action not in ('apply', 'restore'):
            self.last_result = 'rejected'
            self.note = f'Acción de parámetros no permitida: {action or "vacía"}.'
            self._publish(True)
            return

        reasons = self.blockers()
        if reasons:
            self.last_result = 'rejected'
            self.note = 'Aplicación bloqueada: ' + '; '.join(reasons) + '.'
            self._publish(True)
            return
        try:
            requested = packet.get('values')
            if action == 'restore':
                if packet.get('vehicle') != self._vehicle_identity():
                    raise ValueError(
                        'el respaldo no pertenece al vehículo y firmware conectados'
                    )
                if requested is None:
                    requested = self.last_snapshot
            target = (
                normalize_observed_parameter_values(requested)
                if action == 'restore'
                else normalize_parameter_values(requested)
            )
        except ValueError as exc:
            self.last_result = 'rejected'
            self.note = f'Valores rechazados: {exc}.'
            self._publish(True)
            return

        now = self.clock()
        self.pending_apply = {
            'action': action,
            'request_id': request_id,
            'target': target,
            'started': now,
            'seen': set(),
        }
        self.last_result = 'reading_snapshot'
        self.note = 'Capturando una lectura reciente antes de modificar el vehículo…'
        self._request_all()
        self._publish(True)

    def observe(self, message):
        """Consume PARAM_VALUE readbacks from the configured autopilot."""
        if message.get_type() != 'PARAM_VALUE':
            return
        name = parameter_name(message.param_id)
        if name not in REQUEST_PARAMETER_NAMES:
            return
        now = self.clock()
        try:
            received_type = parameter_type(int(message.param_type))
        except (AttributeError, TypeError, ValueError):
            self.last_result = 'rejected'
            self.note = f'{name}: el autopiloto no informó un tipo MAVLink válido.'
            self._publish(True)
            return

        if name in PARAMETER_NAMES:
            self.values[name] = float(message.param_value)
            self.value_times[name] = now
            self.value_types[name] = received_type
        else:
            self.read_only_values[name] = float(message.param_value)
            self.read_only_value_times[name] = now
            self._publish()
            return

        if self.pending_apply is not None and now >= self.pending_apply['started']:
            self.pending_apply['seen'].add(name)

        transaction = self.transaction
        if transaction is not None:
            phase = transaction['phase']
            if phase in ('verify_apply', 'verify_rollback'):
                if now >= transaction['verify_started']:
                    transaction['verify_seen'].add(name)
            elif name == transaction['order'][transaction['index']]:
                target = transaction['target'][name]
                if abs(self.values[name] - target) <= 0.05:
                    transaction['index'] += 1
                    transaction['retries'] = 0
                    if transaction['index'] >= len(transaction['order']):
                        verification = (
                            'verify_apply'
                            if phase == 'apply'
                            else 'verify_rollback'
                        )
                        self._begin_verification(verification)
                    else:
                        self._send_current()
        self._publish()

    def _start_apply(self):
        pending = self.pending_apply
        try:
            snapshot = normalize_observed_parameter_values(self.values)
            types = {
                name: parameter_type(self.value_types[name])
                for name in PARAMETER_NAMES
            }
        except (KeyError, ValueError) as exc:
            self.pending_apply = None
            self.last_result = 'rejected'
            self.note = f'No se pudo crear un respaldo seguro: {exc}.'
            self._publish(True)
            return
        self.pending_apply = None
        self.transaction = {
            'action': pending['action'],
            'request_id': pending['request_id'],
            'phase': 'apply',
            'target': pending['target'],
            'snapshot': snapshot,
            'order': list(PARAMETER_NAMES),
            'index': 0,
            'retries': 0,
            'sent_at': 0.0,
            'types': types,
        }
        self.last_result = 'applying'
        self.note = 'Aplicando un parámetro a la vez y esperando confirmación…'
        self._send_current()
        self._publish(True)

    def _send_current(self):
        transaction = self.transaction
        if transaction is None:
            return
        name = transaction['order'][transaction['index']]
        value = transaction['target'][name]
        self.bridge.master.mav.param_set_send(
            self.bridge.target_system_id,
            self.bridge.target_component_id,
            name.encode('ascii'),
            float(value),
            transaction['types'][name],
        )
        transaction['sent_at'] = self.clock()
        transaction['retries'] += 1

    def _begin_verification(self, phase):
        self.transaction['phase'] = phase
        self.transaction['verify_started'] = self.clock()
        self.transaction['verify_seen'] = set()
        self.note = 'Verificando por lectura independiente los cinco valores…'
        self._request_all()
        self._publish(True)

    def _begin_rollback(self, reason):
        if self.blockers():
            self._finish_partial(reason + ' No se revirtió porque cambió el estado seguro.')
            return
        transaction = self.transaction
        transaction['phase'] = 'rollback'
        transaction['target'] = transaction['snapshot']
        transaction['order'] = list(PARAMETER_NAMES)
        transaction['index'] = 0
        transaction['retries'] = 0
        transaction['sent_at'] = 0.0
        transaction['failure_reason'] = reason
        self.last_result = 'rolling_back'
        self.note = reason + ' Restaurando automáticamente la lectura previa…'
        self._send_current()
        self._publish(True)

    def _finish_success(self):
        transaction = self.transaction
        self.last_snapshot = dict(transaction['snapshot'])
        self.snapshot_captured_at = datetime.now(timezone.utc).isoformat(
            timespec='seconds'
        )
        self.last_request_id = transaction['request_id']
        action = transaction['action']
        self.transaction = None
        self.last_result = 'restored' if action == 'restore' else 'applied'
        self.note = (
            'Configuración anterior restaurada y verificada.'
            if action == 'restore'
            else 'Los cinco parámetros fueron aplicados y verificados.'
        )
        self._publish(True)

    def _finish_rollback(self):
        transaction = self.transaction
        reason = transaction.get('failure_reason', 'Falló la aplicación.')
        self.last_request_id = transaction['request_id']
        self.transaction = None
        self.last_result = 'rolled_back'
        self.note = reason + ' La configuración anterior fue restaurada y verificada.'
        self._publish(True)

    def _finish_partial(self, reason):
        request_id = None
        if self.transaction is not None:
            request_id = self.transaction.get('request_id')
        elif self.pending_apply is not None:
            request_id = self.pending_apply.get('request_id')
        self.pending_apply = None
        self.transaction = None
        self.last_request_id = request_id or self.last_request_id
        self.last_result = 'failed_partial'
        self.note = reason + ' Lee y comprueba los parámetros antes de armar.'
        self._publish(True)

    def _tick_pending(self, now):
        pending = self.pending_apply
        if pending is None:
            return
        if self.blockers():
            self.pending_apply = None
            self.last_result = 'rejected'
            self.note = 'La captura fue cancelada porque dejó de cumplirse el estado seguro.'
            self._publish(True)
            return
        if set(PARAMETER_NAMES).issubset(pending['seen']):
            self._start_apply()
            return
        if now - pending['started'] > 3.0:
            self.pending_apply = None
            self.last_result = 'rejected'
            self.note = 'No llegaron los cinco parámetros; no se modificó el vehículo.'
            self._publish(True)

    def _tick_transaction(self, now):
        transaction = self.transaction
        if transaction is None:
            return
        if self.blockers():
            self._finish_partial('La operación se detuvo al perderse el estado seguro.')
            return
        phase = transaction['phase']
        if phase in ('apply', 'rollback'):
            if now - transaction['sent_at'] <= 1.0:
                return
            if transaction['retries'] < 3:
                self._send_current()
                return
            name = transaction['order'][transaction['index']]
            reason = f'El autopiloto no confirmó {name}.'
            if phase == 'apply':
                self._begin_rollback(reason)
            else:
                self._finish_partial(reason + ' También falló la reversión automática.')
            return

        if not set(PARAMETER_NAMES).issubset(transaction['verify_seen']):
            if now - transaction['verify_started'] <= 3.0:
                return
            if phase == 'verify_apply':
                self._begin_rollback('La verificación final quedó incompleta.')
            else:
                self._finish_partial('No se pudo verificar la reversión automática.')
            return

        if parameter_sets_match(self.values, transaction['target']):
            if phase == 'verify_apply':
                self._finish_success()
            else:
                self._finish_rollback()
        elif phase == 'verify_apply':
            self._begin_rollback('La lectura final no coincide con lo solicitado.')
        else:
            self._finish_partial('La lectura final no coincide con el respaldo.')

    def tick(self):
        """Advance reads and transactions without blocking the MAVLink loop."""
        now = self.clock()
        connected = bool(self.bridge.connected and self._heartbeat_fresh())
        if connected and not self.was_connected:
            self._clear_observations()
            self.note = 'Conexión detectada; leyendo parámetros del vehículo…'
            self.last_result = 'reading'
            self._request_all()
            self._publish(True)
        elif not connected and self.was_connected:
            if self.pending_apply is not None or self.transaction is not None:
                self._finish_partial('Se perdió la conexión durante la operación.')
            else:
                self.note = 'Sin heartbeat reciente; escritura bloqueada.'
                self._publish(True)
            self._clear_observations()
        self.was_connected = connected
        self._tick_pending(now)
        self._tick_transaction(now)
        if (
            self.last_result == 'reading'
            and all(name in self.values for name in PARAMETER_NAMES)
        ):
            self.last_result = 'read'
            self.note = 'Parámetros actuales recibidos del autopiloto.'
            self._publish(True)
        self._publish()
