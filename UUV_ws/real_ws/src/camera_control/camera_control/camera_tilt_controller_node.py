#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from pymavlink import mavutil


class CameraTiltControllerNode(Node):
    def __init__(self):
        super().__init__('camera_tilt_controller_node')

        self.declare_parameter('mavlink_url', 'udpin:0.0.0.0:14550')
        self.declare_parameter('channel', 8)
        self.declare_parameter('neutral_pwm', 1500)
        self.declare_parameter('min_pwm', 1100)
        self.declare_parameter('max_pwm', 1900)
        self.declare_parameter('timeout_s', 0.5)

        self.mavlink_url = self.get_parameter('mavlink_url').value
        self.channel = int(self.get_parameter('channel').value)
        self.neutral_pwm = int(self.get_parameter('neutral_pwm').value)
        self.min_pwm = int(self.get_parameter('min_pwm').value)
        self.max_pwm = int(self.get_parameter('max_pwm').value)
        self.timeout_s = float(self.get_parameter('timeout_s').value)

        self.current_cmd = 0.0
        self.last_cmd_time = time.time()

        self.get_logger().info(f'Conectando MAVLink en {self.mavlink_url}')
        self.master = mavutil.mavlink_connection(self.mavlink_url)

        self.get_logger().info('Esperando heartbeat...')
        self.master.wait_heartbeat()
        self.get_logger().info('Conectado por MAVLink')

        self.sub = self.create_subscription(
            Float32,
            '/camera_tilt/cmd_vel',
            self.cmd_callback,
            10
        )

        self.timer = self.create_timer(0.1, self.send_loop)

    def cmd_callback(self, msg):
        cmd = float(msg.data)
        cmd = max(-1.0, min(1.0, cmd))

        self.current_cmd = cmd
        self.last_cmd_time = time.time()

        self.get_logger().info(f'Comando recibido: {cmd:.2f}')

    def cmd_to_pwm(self, cmd):
        if abs(cmd) < 0.05:
            return self.neutral_pwm

        if cmd > 0:
            return int(self.neutral_pwm + cmd * (self.max_pwm - self.neutral_pwm))
        else:
            return int(self.neutral_pwm + cmd * (self.neutral_pwm - self.min_pwm))

    def send_loop(self):
        if time.time() - self.last_cmd_time > self.timeout_s:
            cmd = 0.0
        else:
            cmd = self.current_cmd

        pwm = self.cmd_to_pwm(cmd)
        self.set_rc_channel_pwm(self.channel, pwm)

    def set_rc_channel_pwm(self, channel, pwm):
        rc = [65535] * 18
        rc[channel - 1] = pwm

        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            *rc
        )

    def destroy_node(self):
        try:
            self.get_logger().info('Parando cámara en PWM 1500')
            self.set_rc_channel_pwm(self.channel, self.neutral_pwm)
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = CameraTiltControllerNode()

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
