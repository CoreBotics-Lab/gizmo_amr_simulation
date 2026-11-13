#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge

class cv_ros(Node):
    def __init__(self)->None:
        super().__init__('Computer_Vision_with_ROS2')
        self.bridge = CvBridge()
        self.camera_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        cv2.namedWindow('Robot Camera View', cv2.WINDOW_AUTOSIZE)
        cv2.resizeWindow('Robot Camera View', 1280, 720)

    def image_callback(self, msg: Image):
        
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        cv2.imshow('Robot Camera View', frame)
        cv2.waitKey(1)

    def destroy_node(self):
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    cvROS = cv_ros()
    try:
        rclpy.spin(cvROS)

    except KeyboardInterrupt:
        pass

    finally:
        cvROS.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__=='__main__':
    main()