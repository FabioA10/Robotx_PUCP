import sys
import select
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class KeyboardTeleopNode(Node):
    def __init__(self):
        super().__init__('keyboard_teleop_node')

        self.pub = self.create_publisher(Twist, '/rov/cmd_vel', 10)

        self.linear_speed = 0.1
        self.angular_speed = 0.3
        self.timeout = 0.15

        self.running = True
        self.print_help()

    def print_help(self):
        print("\033c", end="")
        print("""
TELEOP ROV - /rov/cmd_vel

Movimiento:
  w: avanzar          s: retroceder
  a: lateral izq.     d: lateral der.
  f: subir            g: bajar
  e: girar izq.       r: girar der.

Control:
  espacio: detener
  q: salir

Velocidades:
  + / - : velocidad lineal
  n / m : velocidad angular

Modo:
  Si no presionas ninguna tecla, se envía STOP automáticamente.
""")
        print(f"linear_speed={self.linear_speed:.2f} m/s | angular_speed={self.angular_speed:.2f} rad/s")

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        key = ''

        try:
            tty.setraw(fd)
            ready, _, _ = select.select([sys.stdin], [], [], self.timeout)
            if ready:
                key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

        return key

    def publish_cmd(self, x=0.0, y=0.0, z=0.0, yaw=0.0):
        msg = Twist()
        msg.linear.x = float(x)
        msg.linear.y = float(y)
        msg.linear.z = float(z)
        msg.angular.z = float(yaw)
        self.pub.publish(msg)

    def spin_keyboard(self):
        try:
            while rclpy.ok() and self.running:
                key = self.get_key()

                if key == 'w':
                    self.publish_cmd(x=self.linear_speed)

                elif key == 's':
                    self.publish_cmd(x=-self.linear_speed)

                elif key == 'a':
                    self.publish_cmd(y=self.linear_speed)

                elif key == 'd':
                    self.publish_cmd(y=-self.linear_speed)

                elif key == 'f':
                    self.publish_cmd(z=-self.linear_speed)

                elif key == 'g':
                    self.publish_cmd(z=self.linear_speed)

                elif key == 'e':
                    self.publish_cmd(yaw=self.angular_speed)

                elif key == 'r':
                    self.publish_cmd(yaw=-self.angular_speed)
                    
                    

                elif key == '+':
                    self.linear_speed += 0.05
                    self.print_help()

                elif key == '-':
                    self.linear_speed = max(0.05, self.linear_speed - 0.05)
                    self.print_help()

                elif key == 'n':
                    self.angular_speed += 0.1
                    self.print_help()

                elif key == 'm':
                    self.angular_speed = max(0.1, self.angular_speed - 0.1)
                    self.print_help()

                elif key == ' ':
                    self.publish_cmd()

                elif key == 'q':
                    self.publish_cmd()
                    self.running = False

                else:
                    self.publish_cmd()

        except KeyboardInterrupt:
            self.publish_cmd()


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleopNode()
    node.spin_keyboard()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
