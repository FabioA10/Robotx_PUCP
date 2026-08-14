import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class PipelineColorDetector(Node):

    def __init__(self):
        super().__init__('pipeline_color_detector_node')

        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(
            Image,
            '/bluerov2/front_camera/image_raw',
            self.image_callback,
            10
        )

        self.debug_pub = self.create_publisher(
            Image,
            '/bluerov2/front_camera/color_debug',
            10
        )

        self.status_pub = self.create_publisher(
            String,
            '/pipeline/color_status',
            10
        )

        self.red_pub = self.create_publisher(
            Bool,
            '/pipeline/red_detected',
            10
        )

        self.green_pub = self.create_publisher(
            Bool,
            '/pipeline/green_detected',
            10
        )

        self.min_area = 10

        self.get_logger().info('Pipeline color detector iniciado')

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='rgb8'
        )

        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

        lower_green = np.array([35, 30, 30])
        upper_green = np.array([95, 255, 255])
        mask_green = cv2.inRange(hsv, lower_green, upper_green)

        lower_red_1 = np.array([0, 40, 40])
        upper_red_1 = np.array([15, 255, 255])

        lower_red_2 = np.array([165, 40, 40])
        upper_red_2 = np.array([180, 255, 255])

        mask_red_1 = cv2.inRange(hsv, lower_red_1, upper_red_1)
        mask_red_2 = cv2.inRange(hsv, lower_red_2, upper_red_2)
        mask_red = cv2.bitwise_or(mask_red_1, mask_red_2)

        kernel = np.ones((5, 5), np.uint8)

        mask_green = cv2.morphologyEx(
            mask_green,
            cv2.MORPH_OPEN,
            kernel
        )
        mask_green = cv2.morphologyEx(
            mask_green,
            cv2.MORPH_DILATE,
            kernel
        )

        mask_red = cv2.morphologyEx(
            mask_red,
            cv2.MORPH_OPEN,
            kernel
        )
        mask_red = cv2.morphologyEx(
            mask_red,
            cv2.MORPH_DILATE,
            kernel
        )

        green_area = cv2.countNonZero(mask_green)
        red_area = cv2.countNonZero(mask_red)

        green_detected = green_area > self.min_area
        red_detected = red_area > self.min_area

        status = 'NONE'

        if red_detected:
            status = 'RED'
        elif green_detected:
            status = 'GREEN'

        self.status_pub.publish(String(data=status))
        self.red_pub.publish(Bool(data=red_detected))
        self.green_pub.publish(Bool(data=green_detected))

        debug_rgb = self.create_debug_image(
            frame,
            mask_green,
            mask_red
        )

        debug_msg = self.bridge.cv2_to_imgmsg(
            debug_rgb,
            encoding='rgb8'
        )

        debug_msg.header = msg.header

        self.debug_pub.publish(debug_msg)

        self.get_logger().info(
            f'status={status} '
            f'green_area={green_area} '
            f'red_area={red_area}'
        )

    def create_debug_image(self, frame, mask_green, mask_red):
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        debug_rgb = cv2.cvtColor(gray, cv2.COLOR_GRAY2RGB)

        debug_rgb[mask_green > 0] = [0, 255, 0]
        debug_rgb[mask_red > 0] = [255, 0, 0]

        self.draw_contours(
            debug_rgb,
            mask_green,
            label='GREEN',
            color=(0, 255, 0),
            thickness=2
        )

        self.draw_contours(
            debug_rgb,
            mask_red,
            label='RED',
            color=(255, 0, 0),
            thickness=3
        )

        return debug_rgb

    def draw_contours(self, image, mask, label, color, thickness):
        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        for cnt in contours:
            area = cv2.contourArea(cnt)

            if area <= self.min_area:
                continue

            x, y, w, h = cv2.boundingRect(cnt)

            cv2.rectangle(
                image,
                (x, y),
                (x + w, y + h),
                color,
                thickness
            )

            cv2.putText(
                image,
                f'{label} {int(area)}',
                (x, max(y - 5, 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                color,
                1
            )


def main(args=None):
    rclpy.init(args=args)

    node = PipelineColorDetector()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
