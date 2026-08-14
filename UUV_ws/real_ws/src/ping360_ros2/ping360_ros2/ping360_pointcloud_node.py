#!/usr/bin/env python3

import math
import struct

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

from brping import Ping360


class Ping360PointCloudNode(Node):
    def __init__(self):
        super().__init__('ping360_pointcloud_node')

        self.declare_parameter('host', '192.168.2.2')
        self.declare_parameter('port', 9092)
        self.declare_parameter('frame_id', 'ping360_link')

        self.declare_parameter('range_m', 5.0)
        self.declare_parameter('number_of_samples', 400)
        self.declare_parameter('threshold', 1)

        self.declare_parameter('gain_setting', 1)          # 0 bajo, 1 normal, 2 alto
        self.declare_parameter('transmit_frequency', 750)  # kHz
        self.declare_parameter('transmit_duration', 80)    # us
        self.declare_parameter('angle_step', 2)            # 1 grad = 0.9 grados

        self.host = self.get_parameter('host').value
        self.port = int(self.get_parameter('port').value)
        self.frame_id = self.get_parameter('frame_id').value

        self.range_m = float(self.get_parameter('range_m').value)
        self.number_of_samples = int(self.get_parameter('number_of_samples').value)
        self.threshold = int(self.get_parameter('threshold').value)

        self.gain_setting = int(self.get_parameter('gain_setting').value)
        self.transmit_frequency = int(self.get_parameter('transmit_frequency').value)
        self.transmit_duration = int(self.get_parameter('transmit_duration').value)
        self.angle_step = int(self.get_parameter('angle_step').value)

        self.speed_of_sound = 1500.0  # m/s aproximado en agua
        self.sample_period = self.compute_sample_period(
            self.range_m,
            self.number_of_samples
        )

        self.angle = 0
        self.scan_memory = {}

        self.pub = self.create_publisher(PointCloud2, '/ping360/points', 10)

        self.get_logger().info(f'Conectando al Ping360 por UDP {self.host}:{self.port}')

        self.ping = Ping360()
        self.ping.connect_udp(self.host, self.port)

        if self.ping.initialize() is False:
            self.get_logger().error('No se pudo inicializar el Ping360')
            raise RuntimeError('Ping360 initialization failed')

        self.get_logger().info('Ping360 inicializado correctamente')

        self.configure_sonar()

        self.timer = self.create_timer(0.15, self.scan_once)

    def compute_sample_period(self, range_m, number_of_samples):
        """
        sample_period usa unidades de 25 ns.
        El sonar mide ida y vuelta, por eso se usa 2*range/c.
        """
        sample_period = int((2.0 * range_m) / (self.speed_of_sound * 25e-9 * number_of_samples))
        return max(80, min(sample_period, 40000))

    def configure_sonar(self):
        self.get_logger().info(
            f'Configurando Ping360: range={self.range_m} m, '
            f'samples={self.number_of_samples}, '
            f'sample_period={self.sample_period}, '
            f'gain={self.gain_setting}, '
            f'threshold={self.threshold}'
        )

        self.ping.set_gain_setting(self.gain_setting)
        self.ping.set_transmit_frequency(self.transmit_frequency)
        self.ping.set_transmit_duration(self.transmit_duration)
        self.ping.set_sample_period(self.sample_period)
        self.ping.set_number_of_samples(self.number_of_samples)

    def scan_once(self):
        msg = self.ping.transmitAngle(self.angle)

        if msg is None:
            self.get_logger().warn('No llegó respuesta del Ping360')
            return

        # A veces BlueOS/brping devuelve mensajes internos que no contienen medición sonar.
        # Solo procesamos mensajes que realmente tienen angle y data.
        if not hasattr(msg, 'data') or not hasattr(msg, 'angle'):
            mid = getattr(msg, 'msg_id', getattr(msg, 'message_id', 'unknown'))
            self.get_logger().debug(f'Ignorando mensaje sin data. msg_id={mid}')

            self.angle += self.angle_step
            if self.angle >= 400:
                self.angle = 0

            return

        data = list(msg.data)

        if len(data) == 0:
            return

        angle_grad = msg.angle
        angle_rad = (angle_grad / 400.0) * 2.0 * math.pi

        points_this_angle = []

        for i, intensity in enumerate(data):
            intensity = int(intensity)

            if intensity < self.threshold:
                continue

            r = self.index_to_range(i)

            x = r * math.cos(angle_rad)
            y = r * math.sin(angle_rad)
            z = 0.0

            points_this_angle.append((x, y, z, float(intensity)))

        self.scan_memory[angle_grad] = points_this_angle

        all_points = []
        for pts in self.scan_memory.values():
            all_points.extend(pts)

        if all_points:
            cloud = self.create_pointcloud2(all_points)
            self.pub.publish(cloud)

        self.angle += self.angle_step
        if self.angle >= 400:
            self.angle = 0

    def index_to_range(self, index):
        dt = self.sample_period * 25e-9
        distance = (index * dt * self.speed_of_sound) / 2.0
        return distance

    def create_pointcloud2(self, points):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        data = b''.join([struct.pack('ffff', *p) for p in points])

        cloud = PointCloud2()
        cloud.header = header
        cloud.height = 1
        cloud.width = len(points)
        cloud.fields = fields
        cloud.is_bigendian = False
        cloud.point_step = 16
        cloud.row_step = cloud.point_step * len(points)
        cloud.data = data
        cloud.is_dense = True

        return cloud

    def destroy_node(self):
        try:
            self.get_logger().info('Apagando motor del Ping360')
            self.ping.control_motor_off()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Ping360PointCloudNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
