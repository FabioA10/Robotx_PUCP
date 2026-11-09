#!/usr/bin/env python3
"""BlueOS-resident absolute waypoint executor using the proven V6 core."""

import asyncio
import copy
from datetime import datetime, timezone
import hashlib
import ipaddress
import json
import logging
import os
from pathlib import Path
import signal
import time

from aiohttp import ClientError, ClientSession, ClientTimeout, web, WSMsgType

from compat import BUS, install_ros_shims, Node, PARAMETER_OVERRIDES, String


AGENT_REVISION = 'absolute_embedded_link_v7'
MAVLINK2REST = 'http://127.0.0.1:6040'
LISTEN_HOST = '0.0.0.0'
LISTEN_PORT = 9132
DATA_ROOT = Path(os.environ.get('UUV_SEQUENCE_DATA', '/usr/blueos/userdata/uuv_sequence_agent'))
LOG_ROOT = DATA_ROOT / 'logs'
STATE_FILE = DATA_ROOT / 'last_plan.json'
ALLOWED_CLIENTS = (
    ipaddress.ip_network('127.0.0.0/8'),
    ipaddress.ip_network('192.168.2.0/24'),
)


install_ros_shims()
PARAMETER_OVERRIDES['uuv_sequence_executor'] = {
    'water_density': float(os.environ.get('UUV_WATER_DENSITY', '1000.0')),
    'event_log_enabled': True,
    'event_log_directory': str(LOG_ROOT),
}

from uuv_agent_core.guided_bridge import GuidedBridge  # noqa: E402
from uuv_agent_core.sequence_executor_node import SequenceExecutor  # noqa: E402


def finite(value):
    result = float(value)
    if not (-1.0e20 < result < 1.0e20):
        raise ValueError('Valor MAVLink no finito')
    return result


def bits(value):
    if isinstance(value, dict):
        return int(value.get('bits', 0))
    if isinstance(value, list):
        return 0
    return int(value)


def mav_string(value):
    if isinstance(value, str):
        return value.rstrip('\x00')
    if isinstance(value, (bytes, bytearray)):
        return bytes(value).decode('ascii', errors='replace').rstrip('\x00')
    if isinstance(value, list):
        if value and all(isinstance(item, int) for item in value):
            return bytes(value).decode('ascii', errors='replace').rstrip('\x00')
        return ''.join(str(item) for item in value).rstrip('\x00')
    return str(value)


def set_enum(message, key, enum_name):
    message[key] = {'type': enum_name} if isinstance(message.get(key), dict) else enum_name


def set_mask(message, key, mask):
    message[key] = {'bits': int(mask)} if isinstance(message.get(key), dict) else int(mask)


def set_fixed_string(message, key, value):
    current = message.get(key)
    if isinstance(current, list):
        if current and all(isinstance(item, int) for item in current):
            encoded = list(str(value).encode('ascii', errors='replace'))[:len(current)]
            message[key] = encoded + [0] * (len(current) - len(encoded))
        else:
            encoded = list(str(value))[:len(current)]
            message[key] = encoded + ['\x00'] * (len(current) - len(encoded))
    else:
        message[key] = str(value)


class JsonMavlinkMessage:
    """Attribute facade expected by GuidedBridge.observe()."""

    def __init__(self, payload):
        self.header = payload.get('header', {})
        self.message = payload.get('message', {})

    def get_type(self):
        return str(self.message.get('type', ''))

    def get_srcSystem(self):
        return int(self.header.get('system_id', -1))

    def get_srcComponent(self):
        return int(self.header.get('component_id', -1))

    def __getattr__(self, name):
        if name not in self.message:
            raise AttributeError(name)
        value = self.message[name]
        if name == 'param_id':
            return mav_string(value)
        if isinstance(value, dict) and 'bits' in value:
            return int(value['bits'])
        return value


class RestMav:
    """Synchronous-looking MAVLink sender backed by an async local REST queue."""

    COMMANDS = {
        511: 'MAV_CMD_SET_MESSAGE_INTERVAL',
        512: 'MAV_CMD_REQUEST_MESSAGE',
    }

    def __init__(self):
        self.templates = {}
        self.outbox = asyncio.Queue(maxsize=100)
        self.stream_outbox = asyncio.Queue(maxsize=2)
        self.last_error = None
        self.sent = 0
        self.failed = 0

    async def initialise(self, session):
        names = (
            'HEARTBEAT',
            'SET_MODE',
            'SET_POSITION_TARGET_LOCAL_NED',
            'MANUAL_CONTROL',
            'COMMAND_LONG',
            'PARAM_REQUEST_READ',
        )
        for name in names:
            async with session.get(
                MAVLINK2REST + '/v1/helper/mavlink',
                params={'name': name},
            ) as response:
                if response.status != 200:
                    raise RuntimeError(f'Helper MAVLink {name}: HTTP {response.status}')
                payload = await response.json()
            payload['header'].update(system_id=255, component_id=190, sequence=0)
            self.templates[name] = payload

    def packet(self, name):
        if name not in self.templates:
            raise RuntimeError('Plantilla MAVLink no disponible: ' + name)
        return copy.deepcopy(self.templates[name])

    def enqueue(self, packet):
        message_type = packet.get('message', {}).get('type')
        queue = (
            self.stream_outbox
            if message_type in ('SET_POSITION_TARGET_LOCAL_NED', 'MANUAL_CONTROL')
            else self.outbox
        )
        if queue is self.stream_outbox and queue.full():
            try:
                queue.get_nowait()
            except asyncio.QueueEmpty:
                pass
        try:
            queue.put_nowait(packet)
        except asyncio.QueueFull as exc:
            self.last_error = 'Cola MAVLink llena; control detenido'
            raise RuntimeError(self.last_error) from exc

    def discard_stale(self):
        """Never replay old motion or mode requests after a REST outage."""
        for queue in (self.outbox, self.stream_outbox):
            while True:
                try:
                    queue.get_nowait()
                except asyncio.QueueEmpty:
                    break

    def heartbeat_send(self):
        packet = self.packet('HEARTBEAT')
        message = packet['message']
        message['custom_mode'] = 0
        set_enum(message, 'mavtype', 'MAV_TYPE_GCS')
        set_enum(message, 'autopilot', 'MAV_AUTOPILOT_INVALID')
        set_mask(message, 'base_mode', 0)
        set_enum(message, 'system_status', 'MAV_STATE_ACTIVE')
        message['mavlink_version'] = 3
        self.enqueue(packet)

    def set_mode_send(self, target_system, base_mode, custom_mode):
        packet = self.packet('SET_MODE')
        message = packet['message']
        message['target_system'] = int(target_system)
        set_mask(message, 'base_mode', base_mode)
        message['custom_mode'] = int(custom_mode)
        self.enqueue(packet)

    def set_position_target_local_ned_send(
        self,
        time_boot_ms,
        target_system,
        target_component,
        coordinate_frame,
        type_mask,
        x,
        y,
        z,
        vx,
        vy,
        vz,
        afx,
        afy,
        afz,
        yaw,
        yaw_rate,
    ):
        packet = self.packet('SET_POSITION_TARGET_LOCAL_NED')
        message = packet['message']
        message.update(
            time_boot_ms=int(time_boot_ms),
            target_system=int(target_system),
            target_component=int(target_component),
            x=finite(x),
            y=finite(y),
            z=finite(z),
            vx=finite(vx),
            vy=finite(vy),
            vz=finite(vz),
            afx=finite(afx),
            afy=finite(afy),
            afz=finite(afz),
            yaw=finite(yaw),
            yaw_rate=finite(yaw_rate),
        )
        set_enum(message, 'coordinate_frame', 'MAV_FRAME_LOCAL_NED')
        set_mask(message, 'type_mask', type_mask)
        self.enqueue(packet)

    def manual_control_send(self, target, x, y, z, r, buttons):
        packet = self.packet('MANUAL_CONTROL')
        message = packet['message']
        message.update(
            target=int(target),
            x=int(x),
            y=int(y),
            z=int(z),
            r=int(r),
            buttons=int(buttons),
        )
        if 'buttons2' in message:
            message['buttons2'] = 0
        if 'enabled_extensions' in message:
            message['enabled_extensions'] = 0
        if 's' in message:
            message['s'] = 0
        if 't' in message:
            message['t'] = 0
        self.enqueue(packet)

    def command_long_send(
        self,
        target_system,
        target_component,
        command,
        confirmation,
        param1,
        param2,
        param3,
        param4,
        param5,
        param6,
        param7,
    ):
        packet = self.packet('COMMAND_LONG')
        message = packet['message']
        message.update(
            target_system=int(target_system),
            target_component=int(target_component),
            confirmation=int(confirmation),
            param1=finite(param1),
            param2=finite(param2),
            param3=finite(param3),
            param4=finite(param4),
            param5=finite(param5),
            param6=finite(param6),
            param7=finite(param7),
        )
        command_name = self.COMMANDS.get(int(command))
        if command_name is None:
            raise ValueError('COMMAND_LONG no permitido: ' + str(command))
        set_enum(message, 'command', command_name)
        self.enqueue(packet)

    def param_request_read_send(self, target_system, target_component, param_id, param_index):
        packet = self.packet('PARAM_REQUEST_READ')
        message = packet['message']
        message['target_system'] = int(target_system)
        message['target_component'] = int(target_component)
        set_fixed_string(message, 'param_id', mav_string(param_id))
        message['param_index'] = int(param_index)
        self.enqueue(packet)


class Master:
    def __init__(self, mav):
        self.mav = mav


class EmbeddedBridge(Node):
    MODE_NAME_BY_ID = {
        0: 'STABILIZE',
        1: 'ACRO',
        2: 'ALT_HOLD',
        3: 'AUTO',
        4: 'GUIDED',
        7: 'CIRCLE',
        9: 'SURFACE',
        16: 'POSHOLD',
        19: 'MANUAL',
        20: 'MOTOR_DETECT',
        21: 'SURFTRAK',
    }

    def __init__(self, mav):
        super().__init__('uuv_embedded_mavlink_bridge')
        self._parameters.update(
            sequence_monitor_enabled=True,
            enable_sequence_output=False,
            dvl_source_system=255,
            dvl_source_component=0,
            dvl_rest_base_url=MAVLINK2REST + '/v1/mavlink',
            dvl_rest_poll_period_s=0.20,
            dvl_rest_timeout_s=0.40,
            dvl_tcp_host='192.168.194.95',
            dvl_tcp_port=16171,
            navigation_sample_timeout_s=2.0,
            sequence_heartbeat_timeout_s=2.0,
        )
        self.target_system_id = 1
        self.target_component_id = 1
        self.source_system_id = 255
        self.mode_name_by_id = dict(self.MODE_NAME_BY_ID)
        self.mode = 'UNKNOWN'
        self.armed = False
        self.connected = False
        self.last_heartbeat_time = None
        self.enable_command_output = True
        self.require_manual_mode = True
        self.motors_enabled = False
        self.last_cmd_time = None
        self.deadman = False
        self.master = Master(mav)
        self.guided = GuidedBridge(self)

    def observe(self, payload):
        message = JsonMavlinkMessage(payload)
        if message.get_srcSystem() == self.target_system_id:
            if message.get_type() == 'HEARTBEAT' and message.get_srcComponent() == 1:
                self.last_heartbeat_time = time.monotonic()
                self.connected = True
                self.mode = self.mode_name_by_id.get(
                    int(message.message.get('custom_mode', -1)),
                    'UNKNOWN',
                )
                self.armed = bool(bits(message.message.get('base_mode', 0)) & 128)
        self.guided.observe(message)

    def maintain(self):
        now = time.monotonic()
        self.connected = (
            self.last_heartbeat_time is not None
            and now - self.last_heartbeat_time < 2.0
        )
        if self.guided.enabled:
            self.last_cmd_time = now


class Agent:
    def __init__(self):
        self.http = None
        self.mav = RestMav()
        self.bridge = None
        self.executor = None
        self.latest_status = None
        self.latest_bridge = None
        self.client_seen = 0.0
        self.client_name = None
        self.plan_hash = None
        self.last_plan = None
        self.started = time.monotonic()
        self.stop_event = asyncio.Event()
        self.logger = logging.getLogger('uuv_sequence_agent')

    async def initialise(self):
        DATA_ROOT.mkdir(parents=True, exist_ok=True)
        LOG_ROOT.mkdir(parents=True, exist_ok=True)
        timeout = ClientTimeout(total=1.0, connect=0.5)
        self.http = ClientSession(timeout=timeout)
        await self.mav.initialise(self.http)
        self.bridge = EmbeddedBridge(self.mav)
        self.executor = SequenceExecutor()
        BUS.subscribe('/uuv/sequence/status', self.capture_status)
        BUS.subscribe('/uuv/sequence/bridge_status', self.capture_bridge)

    def capture_status(self, message):
        try:
            self.latest_status = json.loads(message.data)
        except (TypeError, ValueError):
            pass

    def capture_bridge(self, message):
        try:
            self.latest_bridge = json.loads(message.data)
        except (TypeError, ValueError):
            pass

    def client_allowed(self, request):
        try:
            address = ipaddress.ip_address(request.remote or '')
        except ValueError:
            return False
        return any(address in network for network in ALLOWED_CLIENTS)

    async def json_body(self, request):
        if not self.client_allowed(request):
            raise web.HTTPForbidden(text='Cliente fuera de la red de control')
        if request.content_length is not None and request.content_length > 65536:
            raise web.HTTPRequestEntityTooLarge(max_size=65536, actual_size=request.content_length)
        try:
            payload = await request.json()
        except (json.JSONDecodeError, UnicodeDecodeError) as exc:
            raise web.HTTPBadRequest(text='JSON inválido') from exc
        if not isinstance(payload, dict):
            raise web.HTTPBadRequest(text='Se esperaba un objeto JSON')
        self.client_seen = time.monotonic()
        return payload

    async def health(self, _request):
        return web.json_response(
            {
                'ok': True,
                'revision': AGENT_REVISION,
                'uptime_s': round(time.monotonic() - self.started, 3),
                'mavlink_templates': sorted(self.mav.templates),
                'mavlink_sent': self.mav.sent,
                'mavlink_failed': self.mav.failed,
                'mavlink_error': self.mav.last_error,
                'mavlink_queue': self.mav.outbox.qsize(),
                'mavlink_stream_queue': self.mav.stream_outbox.qsize(),
            }
        )

    async def status(self, request):
        if not self.client_allowed(request):
            raise web.HTTPForbidden(text='Cliente fuera de la red de control')
        status = copy.deepcopy(self.latest_status or {})
        bridge = copy.deepcopy(self.latest_bridge or {})
        status.update(
            agent_revision=AGENT_REVISION,
            execution_host='blueos',
            agent_uptime_s=round(time.monotonic() - self.started, 3),
            plan_hash=self.plan_hash,
            bridge=bridge,
            link_policy={
                'normal_grace_s': 1.0,
                'after_grace': 'hold_guided_keep_plan',
                'resume': 'same_waypoint_after_ui_heartbeat',
            },
        )
        return web.json_response(status)

    async def claim(self, request):
        payload = await self.json_body(request)
        name = str(payload.get('client', ''))[:80]
        enable = payload.get('enable_output')
        if not name or type(enable) is not bool:
            raise web.HTTPBadRequest(text='Claim inválido')
        if not enable and self.executor.core.state == 'RUNNING':
            raise web.HTTPConflict(text='No se deshabilita salida durante una secuencia')
        self.client_name = name
        self.bridge.guided.enabled = enable
        self.bridge.motors_enabled = enable
        if enable:
            self.bridge.guided.last_reason = 'Agente embarcado habilitado; esperando validación'
        return web.json_response({'ok': True, 'enabled': enable})

    async def request(self, request):
        payload = await self.json_body(request)
        action = payload.get('action')
        if action == 'prepare':
            if payload.get('plan_mode') != 'absolute':
                raise web.HTTPBadRequest(text='El agente V7 admite solo puntos absolutos')
            steps = payload.get('steps')
            if not isinstance(steps, list) or any(
                not isinstance(step, dict) or step.get('kind') != 'absolute_waypoint'
                for step in steps
            ):
                raise web.HTTPBadRequest(text='Plan absoluto inválido')
            canonical = json.dumps(steps, sort_keys=True, separators=(',', ':'))
            self.plan_hash = hashlib.sha256(canonical.encode('utf-8')).hexdigest()
            self.last_plan = payload
            self.persist_plan()
        message = String()
        message.data = json.dumps(payload, allow_nan=False)
        BUS.publish('/uuv/sequence/request', message)
        return web.json_response({'ok': True, 'action': action, 'plan_hash': self.plan_hash})

    async def alive(self, request):
        payload = await self.json_body(request)
        message = String()
        message.data = json.dumps(payload, allow_nan=False)
        BUS.publish('/uuv/sequence/ui_alive', message)
        return web.json_response({'ok': True})

    def persist_plan(self):
        if self.last_plan is None:
            return
        payload = {
            'saved_utc': datetime.now(timezone.utc).isoformat(),
            'plan_hash': self.plan_hash,
            'request': self.last_plan,
        }
        temporary = STATE_FILE.with_suffix('.tmp')
        temporary.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False),
            encoding='utf-8',
        )
        os.replace(temporary, STATE_FILE)

    async def websocket_loop(self):
        pattern = (
            '^(HEARTBEAT|LOCAL_POSITION_NED|ATTITUDE|SCALED_PRESSURE2|'
            'SCALED_PRESSURE|EKF_STATUS_REPORT|AUTOPILOT_VERSION|PARAM_VALUE|'
            'SYS_STATUS|GPS_GLOBAL_ORIGIN|VISION_POSITION_DELTA)$'
        )
        while not self.stop_event.is_set():
            try:
                async with self.http.ws_connect(
                    MAVLINK2REST + '/v1/ws/mavlink',
                    params={'filter': pattern},
                    heartbeat=5.0,
                    receive_timeout=3.0,
                ) as socket:
                    async for message in socket:
                        if message.type == WSMsgType.TEXT:
                            try:
                                payload = json.loads(message.data)
                                self.bridge.observe(payload)
                            except (TypeError, ValueError, KeyError, AttributeError) as exc:
                                self.logger.warning('MAVLink JSON descartado: %s', exc)
                        elif message.type in (WSMsgType.CLOSED, WSMsgType.ERROR):
                            break
            except (ClientError, asyncio.TimeoutError, OSError) as exc:
                self.mav.last_error = 'WebSocket MAVLink: ' + str(exc)
                await asyncio.sleep(0.25)

    async def sender_loop(self):
        while not self.stop_event.is_set():
            try:
                packet = self.mav.outbox.get_nowait()
            except asyncio.QueueEmpty:
                try:
                    packet = self.mav.stream_outbox.get_nowait()
                except asyncio.QueueEmpty:
                    await asyncio.sleep(0.005)
                    continue
            try:
                async with self.http.post(
                    MAVLINK2REST + '/v1/mavlink',
                    json=packet,
                ) as response:
                    detail = await response.text()
                    if not 200 <= response.status < 300:
                        raise RuntimeError(f'HTTP {response.status}: {detail[:200]}')
                self.mav.sent += 1
                self.mav.last_error = None
            except (ClientError, asyncio.TimeoutError, OSError, RuntimeError) as exc:
                self.mav.failed += 1
                self.mav.last_error = 'Envío MAVLink: ' + str(exc)
                self.mav.discard_stale()

    async def heartbeat_loop(self):
        while not self.stop_event.is_set():
            try:
                self.mav.heartbeat_send()
            except (RuntimeError, ValueError) as exc:
                self.mav.last_error = str(exc)
            await asyncio.sleep(1.0)

    async def timer_loop(self):
        while not self.stop_event.is_set():
            self.bridge.maintain()
            BUS.spin_once()
            if (
                self.executor.core.state != 'RUNNING'
                and not self.bridge.guided.holding_completed_goal
                and self.client_seen
                and time.monotonic() - self.client_seen > 3.0
            ):
                self.bridge.guided.enabled = False
                self.bridge.motors_enabled = False
            await asyncio.sleep(0.01)

    async def run(self):
        await self.initialise()
        application = web.Application(client_max_size=65536)
        application.add_routes(
            [
                web.get('/health', self.health),
                web.get('/v1/status', self.status),
                web.post('/v1/claim', self.claim),
                web.post('/v1/request', self.request),
                web.post('/v1/alive', self.alive),
            ]
        )
        runner = web.AppRunner(application, access_log=None)
        await runner.setup()
        site = web.TCPSite(runner, LISTEN_HOST, LISTEN_PORT)
        await site.start()
        tasks = [
            asyncio.create_task(self.websocket_loop()),
            asyncio.create_task(self.sender_loop()),
            asyncio.create_task(self.heartbeat_loop()),
            asyncio.create_task(self.timer_loop()),
        ]
        self.logger.info('Agente %s escuchando en %s:%d', AGENT_REVISION, LISTEN_HOST, LISTEN_PORT)
        await self.stop_event.wait()
        for task in tasks:
            task.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)
        self.bridge.guided.shutdown()
        await runner.cleanup()
        await self.http.close()


async def async_main():
    agent = Agent()
    loop = asyncio.get_running_loop()
    for signum in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(signum, agent.stop_event.set)
    await agent.run()


def main():
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s %(levelname)s %(name)s: %(message)s',
    )
    asyncio.run(async_main())


if __name__ == '__main__':
    main()
