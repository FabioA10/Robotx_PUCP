#!/usr/bin/env python3

import sys
import termios
import tty
import select
import time
import atexit

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class CameraKeyboardNode(Node):
    def __init__(self):
        super().__init__('camera_keyboard_node')

        self.pub = self.create_publisher(Float32, '/camera_tilt/cmd_vel', 10)

        self.cmd = 0.0
        self.last_key_time = 0.0

        # Mayor tiempo evita cortes cuando el autorepeat se retrasa un poco
        self.key_timeout = 0.45

        self.get_logger().info('Control de cámara:')
        self.get_logger().info('Mantener I = girar a un lado')
        self.get_logger().info('Mantener O = girar al otro lado')
        self.get_logger().info('Soltar tecla = detener')
        self.get_logger().info('Q = salir')

        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        atexit.register(self.restore_terminal)

        # Publica a 50 Hz
        self.timer = self.create_timer(0.02, self.loop)

    def loop(self):
        now = time.time()

        # Leer TODAS las teclas acumuladas en el buffer
        while select.select([sys.stdin], [], [], 0.0)[0]:
            key = sys.stdin.read(1).lower()

            if key == 'i':
                self.cmd = -1.0
                self.last_key_time = now

            elif key == 'o':
                self.cmd = 1.0
                self.last_key_time = now

            elif key == 'q':
                self.cmd = 0.0
                self.publish_cmd()
                self.restore_terminal()
                rclpy.shutdown()
                return

        # Si no llegan más letras por un rato, parar
        if now - self.last_key_time > self.key_timeout:
            self.cmd = 0.0

        self.publish_cmd()

    def publish_cmd(self):
        msg = Float32()
        msg.data = float(self.cmd)
        self.pub.publish(msg)

    def restore_terminal(self):
        try:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        except Exception:
            pass

    def destroy_node(self):
        try:
            self.cmd = 0.0
            self.publish_cmd()
            self.restore_terminal()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraKeyboardNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
