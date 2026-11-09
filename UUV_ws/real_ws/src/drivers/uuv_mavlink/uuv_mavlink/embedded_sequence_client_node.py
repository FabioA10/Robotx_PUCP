"""ROS 2 proxy for a sequence executor running inside BlueOS."""

import json
from queue import Empty, Full, Queue
from threading import Event, Lock, Thread
import time
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .embedded_protocol import (
    disconnected_status,
    initial_offline_status,
    normalise_agent_url,
)


class EmbeddedSequenceClient(Node):
    """Expose the existing sequence topics while BlueOS owns execution."""

    def __init__(self):
        super().__init__('uuv_embedded_sequence_client')
        self.declare_parameter('agent_url', 'http://192.168.2.2:9132')
        self.declare_parameter('enable_sequence_output', False)
        self.declare_parameter('request_timeout_s', 0.30)
        self.agent_url = normalise_agent_url(self.get_parameter('agent_url').value)
        self.output_enabled = bool(self.get_parameter('enable_sequence_output').value)
        self.timeout = float(self.get_parameter('request_timeout_s').value)
        if not 0.10 <= self.timeout <= 1.0:
            raise ValueError('request_timeout_s debe estar entre 0.10 y 1.0 s')

        self.status_pub = self.create_publisher(String, '/uuv/sequence/status', 1)
        self.bridge_pub = self.create_publisher(String, '/uuv/sequence/bridge_status', 1)
        self.create_subscription(String, '/uuv/sequence/request', self.on_request, 20)
        self.create_subscription(String, '/uuv/sequence/ui_alive', self.on_alive, 5)

        self.lock = Lock()
        self.status = initial_offline_status()
        self.alive = None
        self.requests = Queue(maxsize=50)
        self.stop_event = Event()
        self.worker = Thread(target=self.worker_loop, name='blueos-sequence-http', daemon=True)
        self.worker.start()
        self.timer = self.create_timer(0.05, self.publish_status)
        self.get_logger().info(
            f'Cliente de secuencias embarcadas: {self.agent_url}; '
            f'salida={self.output_enabled}'
        )

    @staticmethod
    def decode(message):
        payload = json.loads(message.data)
        if not isinstance(payload, dict):
            raise ValueError('Se esperaba un objeto JSON')
        return payload

    def on_request(self, message):
        try:
            payload = self.decode(message)
            try:
                self.requests.put_nowait(payload)
            except Full:
                if payload.get('action') in ('cancel', 'manual'):
                    try:
                        self.requests.get_nowait()
                    except Empty:
                        pass
                    self.requests.put_nowait(payload)
                else:
                    self.get_logger().error('Cola HTTP llena; solicitud descartada')
        except (TypeError, ValueError, json.JSONDecodeError):
            self.get_logger().warning('Solicitud de secuencia JSON inválida')

    def on_alive(self, message):
        try:
            payload = self.decode(message)
            with self.lock:
                self.alive = payload
        except (TypeError, ValueError, json.JSONDecodeError):
            pass

    def request_json(self, method, path, payload=None):
        body = None
        headers = {'Accept': 'application/json'}
        if payload is not None:
            body = json.dumps(payload, allow_nan=False).encode('utf-8')
            headers['Content-Type'] = 'application/json'
        request = Request(
            self.agent_url + path,
            data=body,
            headers=headers,
            method=method,
        )
        with urlopen(request, timeout=self.timeout) as response:
            if not 200 <= response.status < 300:
                raise OSError(f'HTTP {response.status}')
            raw = response.read(262144)
        result = json.loads(raw.decode('utf-8'))
        if not isinstance(result, dict):
            raise ValueError('Respuesta del agente no es un objeto')
        return result

    def mark_disconnected(self, error):
        with self.lock:
            self.status = disconnected_status(self.status, error)

    def worker_loop(self):
        next_poll = 0.0
        next_claim = 0.0
        next_alive = 0.0
        pending_request = None
        while not self.stop_event.is_set():
            now = time.monotonic()
            try:
                if now >= next_claim:
                    self.request_json(
                        'POST',
                        '/v1/claim',
                        {
                            'client': self.get_name(),
                            'enable_output': self.output_enabled,
                        },
                    )
                    next_claim = now + 1.0

                while True:
                    if pending_request is None:
                        try:
                            pending_request = self.requests.get_nowait()
                        except Empty:
                            break
                    self.request_json('POST', '/v1/request', pending_request)
                    pending_request = None

                if now >= next_alive:
                    with self.lock:
                        alive = self.alive
                        self.alive = None
                    if alive is not None:
                        self.request_json('POST', '/v1/alive', alive)
                    next_alive = now + 0.05

                if now >= next_poll:
                    status = self.request_json('GET', '/v1/status')
                    status['agent_link'] = True
                    status['execution_host'] = 'blueos'
                    with self.lock:
                        self.status = status
                    next_poll = now + 0.05
            except (HTTPError, URLError, OSError, ValueError, json.JSONDecodeError) as exc:
                self.mark_disconnected(exc)
                self.stop_event.wait(0.10)
            else:
                self.stop_event.wait(0.01)

    def publish_status(self):
        with self.lock:
            status = dict(self.status)
        message = String()
        message.data = json.dumps(status, allow_nan=False)
        self.status_pub.publish(message)
        bridge = String()
        bridge.data = json.dumps(status.get('bridge', status), allow_nan=False)
        self.bridge_pub.publish(bridge)

    def close(self):
        self.stop_event.set()
        self.worker.join(timeout=self.timeout + 0.5)


def main(args=None):
    rclpy.init(args=args)
    node = EmbeddedSequenceClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
