import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

from std_msgs.msg import Float32


class PipelineDetectorNode(Node):
    def __init__(self):
        super().__init__('pipeline_detector_node')

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/ping360/real_scan',
            self.scan_callback,
            10
        )
        
        
        self.distance_pub = self.create_publisher(
            Float32,
            '/pipeline/distance',
            10
        )

        self.bearing_pub = self.create_publisher(
            Float32,
            '/pipeline/bearing',
            10
        )

        self.heading_pub = self.create_publisher(
            Float32,
            '/pipeline/heading',
            10
        )
        
        

        self.status_pub = self.create_publisher(
            String,
            '/pipeline/status',
            10
        )

        self.marker_pub = self.create_publisher(
            Marker,
            '/pipeline/debug_marker',
            10
        )

        self.min_valid_range = 0.75
        self.max_detect_range = 15.0

        self.get_logger().info('Pipeline detector iniciado. Escuchando /ping360/real_scan')

    def scan_callback(self, msg):
        points = []

        angle = msg.angle_min

        for r in msg.ranges:
            if math.isfinite(r):
                if self.min_valid_range <= r <= self.max_detect_range:
                    x = r * math.cos(angle)
                    y = r * math.sin(angle)
                    points.append((x, y, r, angle))

            angle += msg.angle_increment

        status = String()

        if len(points) < 5:
            status.data = 'PIPELINE_NOT_DETECTED'
            self.status_pub.publish(status)
            self.publish_marker([], msg.header.frame_id)
            return

        # Punto más cercano detectado
        closest = min(points, key=lambda p: p[2])
        cx, cy, cr, ca = closest

        # Filtrar puntos cercanos alrededor del objeto más cercano
        cluster = []
        for p in points:
            x, y, r, a = p
            if abs(r - cr) < 1.0:
                cluster.append((x, y))

        if len(cluster) < 5:
            status.data = f'OBJECT_DETECTED range={cr:.2f} angle={math.degrees(ca):.1f}'
            self.status_pub.publish(status)
            self.publish_marker([(cx, cy)], msg.header.frame_id)
            return

        # Centroide del cluster
        mx = sum(p[0] for p in cluster) / len(cluster)
        my = sum(p[1] for p in cluster) / len(cluster)
        
                # Estimar orientación de la tubería usando PCA 2D simple
        sxx = 0.0
        syy = 0.0
        sxy = 0.0

        for x, y in cluster:
            dx = x - mx
            dy = y - my
            sxx += dx * dx
            syy += dy * dy
            sxy += dx * dy

        heading = 0.5 * math.atan2(2.0 * sxy, sxx - syy)
        

        distance = math.sqrt(mx * mx + my * my)
        bearing = math.atan2(my, mx)
        
        
        distance_msg = Float32()
        distance_msg.data = float(distance)

        bearing_msg = Float32()
        bearing_msg.data = float(math.degrees(bearing))

        heading_msg = Float32()
        heading_msg.data = float(math.degrees(heading))

        self.distance_pub.publish(distance_msg)
        self.bearing_pub.publish(bearing_msg)
        self.heading_pub.publish(heading_msg)
        
        

        status.data = (
            f'PIPELINE_CANDIDATE '
            f'distance={distance:.2f} '
            f'bearing_deg={math.degrees(bearing):.1f} '
            f'heading_deg={math.degrees(heading):.1f} '
            f'points={len(cluster)}'
        )

        self.status_pub.publish(status)
        self.publish_marker(cluster, msg.header.frame_id)

    def publish_marker(self, points, frame_id):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = frame_id

        marker.ns = 'pipeline_detector'
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD

        marker.scale.x = 0.12
        marker.scale.y = 0.12

        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        for x, y in points:
            p = Point()
            p.x = float(x)
            p.y = float(y)
            p.z = 0.0
            marker.points.append(p)

        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = PipelineDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
