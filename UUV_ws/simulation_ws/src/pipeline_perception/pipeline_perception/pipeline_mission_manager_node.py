import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, Int32


class PipelineMissionManager(Node):

    def __init__(self):
        super().__init__('pipeline_mission_manager_node')

        self.red_area = 0
        self.state = 'SEARCHING'

        self.lock_threshold = 1000
        self.stop_threshold = 0

        self.red_area_sub = self.create_subscription(
            Int32,
            '/pipeline/red_area',
            self.red_area_callback,
            10
        )

        self.enable_pub = self.create_publisher(
            Bool,
            '/pipeline/follower_enable',
            10
        )

        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info('Mission manager iniciado')

    def red_area_callback(self, msg):
        self.red_area = msg.data

    def set_follower_enabled(self, enabled):
        msg = Bool()
        msg.data = enabled
        self.enable_pub.publish(msg)

    def control_loop(self):

        if self.state == 'SEARCHING':
            self.set_follower_enabled(True)

            if self.red_area >= self.lock_threshold:
                self.state = 'RED_LOCKED'
                self.get_logger().warn(
                    f'ROJO DETECTADO MAYOR A 1000 | red_area={self.red_area}'
                )

        elif self.state == 'RED_LOCKED':
            self.set_follower_enabled(True)

            if self.red_area <= self.stop_threshold:
                self.state = 'STOPPED'
                self.set_follower_enabled(False)
                self.get_logger().warn(
                    'ROJO DESAPARECIÓ COMPLETAMENTE. SUB DETENIDO.'
                )

        elif self.state == 'STOPPED':
            self.set_follower_enabled(False)


def main(args=None):
    rclpy.init(args=args)
    node = PipelineMissionManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
