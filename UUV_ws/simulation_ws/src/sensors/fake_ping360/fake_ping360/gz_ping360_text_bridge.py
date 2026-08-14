import math
import subprocess

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import select


class GzPing360TextBridge(Node):
    def __init__(self):
        super().__init__('gz_ping360_text_bridge')

        self.pub = self.create_publisher(LaserScan, '/ping360/real_scan', 10)

        self.angle_min = -math.pi
        self.angle_max = math.pi
        self.angle_step = math.radians(1.0)
        self.range_min = 0.75
        self.range_max = 50.0
        self.count = 360
        self.ranges = []

        self.get_logger().info('Leyendo Gazebo topic /ping360/gz_scan...')
        self.process = subprocess.Popen(
            ['gz', 'topic', '-e', '-t', '/ping360/gz_scan'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1
        )

        self.timer = self.create_timer(0.01, self.read_gz_output)

    def read_gz_output(self):
        lines_read = 0
        max_lines_per_cycle = 3000

        while lines_read < max_lines_per_cycle:
            ready, _, _ = select.select([self.process.stdout], [], [], 0.0)

            if not ready:
                break

            line = self.process.stdout.readline()

            if not line:
                break

            lines_read += 1
            line = line.strip()

            if line.startswith('angle_min:'):
                self.angle_min = float(line.split(':')[1].strip())

            elif line.startswith('angle_max:'):
                self.angle_max = float(line.split(':')[1].strip())

            elif line.startswith('angle_step:'):
                self.angle_step = float(line.split(':')[1].strip())

            elif line.startswith('range_min:'):
                self.range_min = float(line.split(':')[1].strip())

            elif line.startswith('range_max:'):
                self.range_max = float(line.split(':')[1].strip())

            elif line.startswith('count:'):
                self.count = int(line.split(':')[1].strip())
                self.ranges = []

            elif line.startswith('ranges:'):
                value = line.split(':')[1].strip()

                if value == 'inf':
                    self.ranges.append(float('inf'))
                elif value == '-inf':
                    self.ranges.append(float('-inf'))
                else:
                    self.ranges.append(float(value))

                if len(self.ranges) >= self.count:
                    self.publish_scan()

    def publish_scan(self):
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'ping360_gz_link'

        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_step
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = self.range_min
        scan.range_max = self.range_max

        scan.ranges = self.ranges[:self.count]
        scan.intensities = [0.0 if math.isinf(r) else max(0.0, 1.0 - r / self.range_max)
                            for r in scan.ranges]

        self.pub.publish(scan)
        self.ranges = []


def main(args=None):
    rclpy.init(args=args)
    node = GzPing360TextBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
