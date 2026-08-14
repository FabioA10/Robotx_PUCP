import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class RealCameraBridge(Node):

    def __init__(self):
        super().__init__('real_camera_bridge')
        self.bridge = CvBridge()

        # Publisher para publicar la imagen en ROS2
        self.pub = self.create_publisher(
            Image,
            '/bluerov2/front_camera/image_raw',
            10
        )

        # Cambia el índice de la cámara según tu sistema
        self.cap = cv2.VideoCapture(0)  # 0 = primera cámara USB

        if not self.cap.isOpened():
            self.get_logger().error("No se puede abrir la cámara!")
            exit(1)

        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 Hz
        self.get_logger().info("Real camera bridge iniciado")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("No se pudo leer la cámara")
            return

        # Convertir BGR a RGB
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        ros_msg = self.bridge.cv2_to_imgmsg(frame_rgb, encoding='rgb8')
        self.pub.publish(ros_msg)

def main(args=None):
    rclpy.init(args=args)
    node = RealCameraBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
