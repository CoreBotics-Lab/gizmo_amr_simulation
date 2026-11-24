#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge

class Apriltag_Tracker(Node):
    def __init__(self) -> None:
        super().__init__('apriltag_tracker')
        cv2.namedWindow('apriltag frame', cv2.WINDOW_FREERATIO)
        cv2.resizeWindow('apriltag frame', 1280, 720)

        self.cv_bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

    def image_callback(self, msg: Image) -> None:
        if msg is None:
            return

        frame = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        cv2.imshow('apriltag_gray', gray_img)
        cv2.imshow('apriltag frame', frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    apriltag_tracker = Apriltag_Tracker()
    try:
        rclpy.spin(apriltag_tracker)
    except KeyboardInterrupt:
        pass

    finally:
        apriltag_tracker.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
