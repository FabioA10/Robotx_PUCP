import math
import random

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float32, String

from pymavlink import mavutil


class FakeDVLNode(Node):
    def __init__(self):
        super().__init__('fake_dvl_node')

        self.dvl_pub = self.create_publisher(TwistStamped, '/dvl/twist', 10)
        self.alt_pub = self.create_publisher(Float32, '/dvl/altitude', 10)
        self.status_pub = self.create_publisher(String, '/dvl/status', 10)

        # Parámetros tipo A50
        self.min_altitude = 0.05      # 5 cm
        self.max_altitude = 50.0      # 50 m
        self.max_velocity = 3.75      # m/s
        self.noise_std = 0.005        # ruido simple en m/s
        self.rate_hz = 10.0           # dentro del rango 4-15 Hz del A50

        self.get_logger().info('Conectando fake DVL por MAVLink en puerto 14551...')
        self.master = mavutil.mavlink_connection('udpin:127.0.0.1:14551')
        self.master.wait_heartbeat()
        self.get_logger().info('Fake DVL conectado al vehículo')

        self.timer = self.create_timer(1.0 / self.rate_hz, self.timer_callback)

    def saturate(self, value, limit):
        return max(min(value, limit), -limit)

    def add_noise(self, value):
        return value + random.gauss(0.0, self.noise_std)

    def compute_quality(self, altitude):
        if altitude < self.min_altitude or altitude > self.max_altitude:
            return 0.0

        # Calidad alta en rango medio, menor cerca de límites
        mid = (self.min_altitude + self.max_altitude) / 2.0
        distance_from_mid = abs(altitude - mid)
        max_distance = (self.max_altitude - self.min_altitude) / 2.0

        quality = 1.0 - 0.5 * (distance_from_mid / max_distance)
        return max(0.0, min(1.0, quality))

    def timer_callback(self):
        msg = self.master.recv_match(type='LOCAL_POSITION_NED', blocking=False)

        if msg is None:
            return

        # LOCAL_POSITION_NED:
        # vx, vy, vz en m/s
        # z positivo hacia abajo en NED
        altitude = float(msg.z)

        quality = self.compute_quality(altitude)
        bottom_lock = quality > 0.2

        vx = self.saturate(float(msg.vx), self.max_velocity)
        vy = self.saturate(float(msg.vy), self.max_velocity)
        vz = self.saturate(float(msg.vz), self.max_velocity)

        if bottom_lock:
            vx = self.add_noise(vx)
            vy = self.add_noise(vy)
            vz = self.add_noise(vz)
        else:
            vx = 0.0
            vy = 0.0
            vz = 0.0

        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = 'dvl_link'
        twist_msg.twist.linear.x = vx
        twist_msg.twist.linear.y = vy
        twist_msg.twist.linear.z = vz

        self.dvl_pub.publish(twist_msg)

        altitude_msg = Float32()
        altitude_msg.data = altitude
        self.alt_pub.publish(altitude_msg)

        status_msg = String()
        status_msg.data = (
            f'bottom_lock={bottom_lock}, '
            f'quality={quality:.2f}, '
            f'altitude={altitude:.2f}, '
            f'valid={bottom_lock}'
        )
        self.status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeDVLNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
