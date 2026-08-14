import math
import random

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from pymavlink import mavutil


class FakePing360Node(Node):
    def __init__(self):
        super().__init__('fake_ping360_node')

        self.scan_pub = self.create_publisher(LaserScan, '/ping360/scan', 10)
        self.status_pub = self.create_publisher(String, '/ping360/status', 10)

        # Parámetros aproximados del Ping360
        self.max_range = 50.0
        self.min_range = 0.75
        self.angle_increment_deg = 1.0
        self.noise_std = 0.06
        self.scan_rate_hz = 10.0

        # Obstáculos simulados: (x, y, radio, reflectividad)
        self.obstacles = [
            (6.0, 0.0, 1.2, 0.95),    # caja frontal en Gazebo
            (8.0, -4.0, 3.0, 0.85),   # pared derecha aproximada
            (4.0, 5.0, 1.5, 0.80),    # caja izquierda
        ]
        
        
        self.get_logger().info('Conectando fake Ping360 por MAVLink en puerto 14553...')
        self.master = mavutil.mavlink_connection('udpin:127.0.0.1:14553')
        self.master.wait_heartbeat()
        self.get_logger().info('Fake Ping360 conectado al vehículo')

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.timer = self.create_timer(1.0 / self.scan_rate_hz, self.timer_callback)

    def update_pose(self):
        while True:
            msg = self.master.recv_match(blocking=False)
            if msg is None:
                break

            msg_type = msg.get_type()

            if msg_type == 'LOCAL_POSITION_NED':
                self.x = float(msg.x)
                self.y = float(msg.y)

            elif msg_type == 'ATTITUDE':
                self.yaw = float(msg.yaw)

    def ray_circle_intersection(self, angle):
        best_range = self.max_range
        best_reflectivity = 0.0

        ray_dx = math.cos(angle)
        ray_dy = math.sin(angle)

        for ox, oy, radius, reflectivity in self.obstacles:
            cx = ox - self.x
            cy = oy - self.y

            projection = cx * ray_dx + cy * ray_dy

            if projection < self.min_range:
                continue

            closest_x = projection * ray_dx
            closest_y = projection * ray_dy

            dist_sq = (cx - closest_x) ** 2 + (cy - closest_y) ** 2

            if dist_sq <= radius ** 2:
                offset = math.sqrt(radius ** 2 - dist_sq)
                hit_range = projection - offset

                if self.min_range <= hit_range < best_range:
                    best_range = hit_range
                    best_reflectivity = reflectivity

        if best_range < self.max_range:
            best_range += random.gauss(0.0, self.noise_std)
            best_range = max(self.min_range, min(best_range, self.max_range))

        return best_range, best_reflectivity

    def compute_intensity(self, distance, reflectivity):
        if distance >= self.max_range:
            return 0.0

        range_factor = max(0.0, 1.0 - distance / self.max_range)
        intensity = reflectivity * range_factor
        intensity += random.gauss(0.0, 0.03)

        return max(0.0, min(1.0, intensity))

    def timer_callback(self):
        self.update_pose()

        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'ping360_link'

        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = math.radians(self.angle_increment_deg)
        scan.time_increment = 0.0
        scan.scan_time = 1.0 / self.scan_rate_hz
        scan.range_min = self.min_range
        scan.range_max = self.max_range

        ranges = []
        intensities = []
        detections = 0

        num_samples = int((scan.angle_max - scan.angle_min) / scan.angle_increment)

        for i in range(num_samples):
            local_angle = scan.angle_min + i * scan.angle_increment
            world_angle = self.yaw + local_angle

            distance, reflectivity = self.ray_circle_intersection(world_angle)
            intensity = self.compute_intensity(distance, reflectivity)

            if distance < self.max_range:
                detections += 1

            ranges.append(distance)
            intensities.append(intensity)

        scan.ranges = ranges
        scan.intensities = intensities

        self.scan_pub.publish(scan)

        status = String()
        status.data = (
            f'valid=True, '
            f'range_min={self.min_range:.2f}, '
            f'range_max={self.max_range:.1f}, '
            f'samples={num_samples}, '
            f'detections={detections}, '
            f'pose_x={self.x:.2f}, '
            f'pose_y={self.y:.2f}, '
            f'yaw_deg={math.degrees(self.yaw):.1f}'
        )
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = FakePing360Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
