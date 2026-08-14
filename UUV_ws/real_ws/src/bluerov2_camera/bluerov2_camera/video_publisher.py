import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2


class VideoPublisher(Node):

    def __init__(self):
        super().__init__("video_publisher")

        self.pub = self.create_publisher(
            CompressedImage,
            "/camera/image/compressed",
            10
        )

        # Pipeline que ya verificaste que funciona
        pipeline = (
            "udpsrc address=192.168.2.1 port=5600 "
            'caps="application/x-rtp,media=video,clock-rate=90000,encoding-name=H264,payload=96" '
            "! rtpjitterbuffer "
            "! rtph264depay "
            "! h264parse "
            "! avdec_h264 "
            "! videoconvert "
            "! appsink sync=false drop=true max-buffers=1"
        )

        self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

        if not self.cap.isOpened():
            self.get_logger().error("No se pudo abrir el flujo de video")
            raise RuntimeError("No se pudo abrir el flujo de video")

        self.get_logger().info("Video abierto correctamente")

        self.timer = self.create_timer(1.0 / 30.0, self.publish_frame)

    def publish_frame(self):
        ret, frame = self.cap.read()

        if not ret:
            return

        ok, buffer = cv2.imencode(".jpg", frame)

        if not ok:
            return

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = "jpeg"
        msg.data = buffer.tobytes()

        self.pub.publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = VideoPublisher()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
