#!/usr/bin/env python3
"""Simple color detection node for TidyBot camera feed."""

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class VisionNode(Node):
    """Detect blue, green, and yellow objects in the RGB camera stream."""

    def __init__(self):
        super().__init__('tidybot_vision')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10,
        )
        self.get_logger().info(
            'Vision node started (Blue, Green, Yellow detection)'
        )

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_blue = np.array([100, 120, 70])
        upper_blue = np.array([130, 255, 255])

        lower_green = np.array([40, 70, 70])
        upper_green = np.array([80, 255, 255])

        lower_yellow = np.array([22, 120, 120])
        upper_yellow = np.array([35, 255, 255])

        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
        mask_green = cv2.inRange(hsv, lower_green, upper_green)
        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

        kernel = np.ones((5, 5), np.uint8)
        mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, kernel)
        mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_CLOSE, kernel)

        mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel)
        mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_CLOSE, kernel)

        mask_yellow = cv2.morphologyEx(mask_yellow, cv2.MORPH_OPEN, kernel)
        mask_yellow = cv2.morphologyEx(mask_yellow, cv2.MORPH_CLOSE, kernel)

        combined_mask = mask_blue | mask_green | mask_yellow

        def detect(mask, color_name, draw_color):
            contours, _ = cv2.findContours(
                mask,
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE,
            )

            for contour in contours:
                area = cv2.contourArea(contour)
                if area <= 500:
                    continue

                x, y, w, h = cv2.boundingRect(contour)
                center_x = x + w // 2
                center_y = y + h // 2

                cv2.rectangle(frame, (x, y), (x + w, y + h), draw_color, 2)
                cv2.putText(
                    frame,
                    color_name,
                    (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    draw_color,
                    2,
                )
                cv2.circle(frame, (center_x, center_y), 5, draw_color, -1)

        detect(mask_blue, 'BLUE', (255, 0, 0))
        detect(mask_green, 'GREEN', (0, 255, 0))
        detect(mask_yellow, 'YELLOW', (0, 255, 255))

        cv2.imshow('Camera Feed', frame)
        cv2.imshow('Mask', combined_mask)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = VisionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
