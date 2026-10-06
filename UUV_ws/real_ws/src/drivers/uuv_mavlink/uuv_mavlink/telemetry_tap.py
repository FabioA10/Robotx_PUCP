"""Salida de diagnóstico pasiva; fallos de registro no interrumpen MAVLink."""

import json
import math
from pathlib import Path
import time
import uuid

from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


TOPIC = '/uuv/diagnostics/mavlink'
RX_TYPES = {
    'HEARTBEAT', 'LOCAL_POSITION_NED', 'ATTITUDE', 'SCALED_PRESSURE2',
    'EKF_STATUS_REPORT', 'SERVO_OUTPUT_RAW', 'POSITION_TARGET_LOCAL_NED',
    'PID_TUNING', 'SYS_STATUS', 'VISION_POSITION_DELTA', 'GLOBAL_POSITION_INT',
    'STATUSTEXT', 'COMMAND_ACK', 'AUTOPILOT_VERSION', 'PARAM_VALUE',
}
TX_TYPES = {
    'SET_POSITION_TARGET_LOCAL_NED', 'MANUAL_CONTROL', 'SET_MODE',
    'COMMAND_LONG', 'RC_CHANNELS_OVERRIDE', 'PARAM_REQUEST_READ',
}


def clock_id():
    try:
        return Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    except OSError:
        return None


def sanitize(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, bytes):
        return value.decode(errors='replace')
    if isinstance(value, (list, tuple)):
        return [sanitize(v) for v in value]
    if isinstance(value, dict):
        return {k: sanitize(v) for k, v in value.items()}
    return value


class TelemetryTap:
    def __init__(self, node):
        self.node = node
        self.session = uuid.uuid4().hex
        self.clock = clock_id()
        self.serial = 0
        self.errors = 0
        self.tx_available = False
        qos = QoSProfile(depth=500, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.publisher = node.create_publisher(String, TOPIC, qos)

    def attach(self, mav):
        # El callback de pymavlink se ejecuta después de escribir el paquete.
        # No reemplazar otros observadores que pudieran existir.
        try:
            if getattr(mav, 'send_callback', None) is not None:
                raise ValueError('Existe otro callback de envío; se conserva')
            mav.set_send_callback(self.sent)
            self.tx_available = True
        except Exception as exc:
            self._warning('Diagnóstico TX no disponible: ' + str(exc))

    def _warning(self, text):
        try:
            self.node.get_logger().warning(text)
        except Exception:
            pass

    def receive(self, msg):
        self._observe('rx', msg)

    def sent(self, msg, *_args, **_kwargs):
        self._observe('tx', msg)

    def _observe(self, direction, msg):
        # Nunca dejar que una excepción de diagnóstico afecte al control.
        try:
            mono = time.monotonic_ns()
            wall = time.time_ns()
            if self.publisher.get_subscription_count() == 0:
                return
            kind = msg.get_type()
            if kind not in (RX_TYPES if direction == 'rx' else TX_TYPES):
                return
            fields = sanitize(msg.to_dict())
            source_times = {
                key: fields[key]
                for key in ('time_boot_ms', 'time_usec', 'time_us')
                if key in fields
            }
            self.serial += 1
            data = {
                'revision': 'guided_telemetry_tap_v1',
                'session': self.session, 'serial': self.serial,
                'clock_id': self.clock, 'host_monotonic_ns': mono,
                'host_time_ns': wall, 'direction': direction, 'message_type': kind,
                'source_system': msg.get_srcSystem(), 'source_component': msg.get_srcComponent(),
                'packet_sequence': msg.get_seq(), 'source_time_fields': source_times,
                'tx_hook_available': self.tx_available, 'tap_errors': self.errors,
                'message': fields,
            }
            out = String()
            out.data = json.dumps(data, ensure_ascii=False, allow_nan=False)
            self.publisher.publish(out)
        except Exception as exc:
            self.errors += 1
            if self.errors == 1:
                self._warning('Error de diagnóstico MAVLink; control continúa: ' + str(exc))
