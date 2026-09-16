"""Default-disabled dry-run endpoint for future pool sequence execution.

It accepts a JSON plan from the pool dashboard only to validate it and publish
its state.  It intentionally sends no vehicle commands and must not be treated
as an autonomous controller.
"""

import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String

from uuv_dashboard.sequence_executor_model import ExecutionGuard, validate_steps


class SequenceExecutor(Node):
    def __init__(self):
        super().__init__('uuv_dry_run_sequence_executor')
        self.declare_parameter('enable_sequence_execution', False)
        self.enabled = bool(
            self.get_parameter('enable_sequence_execution').value)
        self.guard = ExecutionGuard()
        self.status_pub = self.create_publisher(
            String, '/uuv/sequence_dry_run/status', 10)
        self.create_subscription(
            String, '/uuv/sequence_dry_run/request', self.request_callback, 10)
        self.create_subscription(
            Empty, '/uuv/sequence_dry_run/cancel', self.cancel_callback, 10)
        self.timer = self.create_timer(0.1, self.watchdog)
        self.publish_status(
            'BLOQUEADO: ejecutor de secuencia en prueba seca; no hay salida MAVLink')

    def publish_status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def request_callback(self, msg):
        try:
            request = json.loads(msg.data)
            steps = validate_steps(request.get('steps'))
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            self.guard.cancel('Solicitud inválida: ' + str(exc))
            self.publish_status(self.guard.reason)
            return
        if not self.guard.start(time.monotonic(), self.enabled):
            self.publish_status(self.guard.reason)
            return
        # Keeping this log avoids a silent transition if a developer enables the
        # parameter by mistake before a vehicle transport has been reviewed.
        self.guard.cancel('Bloqueada: transporte MAVLink de secuencias no implementado')
        self.get_logger().warning(
            'Plan validado (%d pasos) sin enviar órdenes al vehículo', len(steps))
        self.publish_status(self.guard.reason)

    def cancel_callback(self, _msg):
        self.guard.cancel('Cancelada por la interfaz u operador')
        self.publish_status(self.guard.reason)

    def watchdog(self):
        if self.guard.watchdog(time.monotonic()):
            self.publish_status(self.guard.reason)


def main(args=None):
    rclpy.init(args=args)
    node = SequenceExecutor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
