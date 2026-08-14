import os

os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

import gz.transport13 as gz_transport
from gz.msgs10.image_pb2 import Image as GzImage


class GzCameraBridge(Node):

    def __init__(self):
        super().__init__('gz_camera_bridge')

        self.pub = self.create_publisher(
            Image,
            '/bluerov2/front_camera/image_raw',
            10
        )

        self.gz_node = gz_transport.Node()

        self.gz_topic = '/bluerov2/front_camera/image'

        self.gz_node.subscribe(
            GzImage,
            self.gz_topic,
            self.gz_callback
        )

        self.get_logger().info(
            f'Camera bridge escuchando Gazebo topic {self.gz_topic}'
        )

    def gz_callback(self, msg):

        ros_img = Image()

        ros_img.header.stamp = self.get_clock().now().to_msg()
        ros_img.header.frame_id = 'front_rgb_camera'

        ros_img.height = msg.height
        ros_img.width = msg.width

        # RGB 8 bits
        ros_img.encoding = 'rgb8'

        ros_img.is_bigendian = False

        # 3 bytes por pixel (R,G,B)
        ros_img.step = msg.width * 3

        ros_img.data = bytes(msg.data)

        self.pub.publish(ros_img)


def main(args=None):
    rclpy.init(args=args)
    node = GzCameraBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
