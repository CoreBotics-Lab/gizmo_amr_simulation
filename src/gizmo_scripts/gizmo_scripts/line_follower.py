#!/usr/bin/env python3
from sys import exception
import os
os.environ["OMP_NUM_THREADS"] = "2"
os.environ["OPENBLAS_NUM_THREADS"] = "2"

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

import cv2
from cv_bridge import CvBridge

import numpy as np

class ColorContourTracker(Node):
    def __init__(self)->None:
        super().__init__('ColorContourTracker')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.error_pub = self.create_publisher(Float32, '/centroid_error', 10)

        cv2.namedWindow('HSV Selector', cv2.WINDOW_AUTOSIZE)
        cv2.resizeWindow('HSV Selector', 1280, 720)

        cv2.createTrackbar('H_min', 'HSV Selector', 0, 179, lambda x: None)
        cv2.createTrackbar('H_max', 'HSV Selector', 179, 179, lambda x: None)
        cv2.createTrackbar('S_min', 'HSV Selector', 0, 255, lambda x: None)
        cv2.createTrackbar('S_max', 'HSV Selector', 255, 255, lambda x: None)
        cv2.createTrackbar('V_min', 'HSV Selector', 0, 255, lambda x: None)
        cv2.createTrackbar('V_max', 'HSV Selector', 255, 255, lambda x: None)


        self.get_logger().info("CV Node started. Waiting for '/camera/image_raw'...")

    def image_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        height, width = frame.shape[:2]
        center_x = width // 2
        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        h_min = cv2.getTrackbarPos('H_min', 'HSV Selector')
        h_max = cv2.getTrackbarPos('H_max', 'HSV Selector')
        s_min = cv2.getTrackbarPos('S_min', 'HSV Selector')
        s_max = cv2.getTrackbarPos('S_max', 'HSV Selector')
        v_min = cv2.getTrackbarPos('V_min', 'HSV Selector')
        v_max = cv2.getTrackbarPos('V_max', 'HSV Selector')

        lower_hsv = np.array([h_min, s_min, v_min])
        upper_hsv = np.array([h_max, s_max, v_max])
        mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
        mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        result = cv2.bitwise_and(frame, frame, mask=mask)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            if cv2.contourArea(cnt) > 500:  # ignore tiny speckles (< 500 px²)
                cv2.drawContours(frame, [cnt], -1, (0, 255, 0), 2)
                # draw a bounding box around each detected object
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
                M = cv2.moments(cnt)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

                    # Normalized error [-1.0 to 1.0]
                    error = float((cx - center_x) / (width / 2.0))

                    # Publish to topic
                    error_msg = Float32()
                    error_msg.data = error
                    self.error_pub.publish(error_msg)
                    

        output = np.hstack((frame, mask_bgr, result))

        cv2.imshow('HSV Selector', output)
        cv2.waitKey(1)
        
    def destroy_all(self):
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        super().destroy_node()

def main(args=None)->None:
    rclpy.init(args=args)
    color_contour_tracker = ColorContourTracker()
    try:
        rclpy.spin(color_contour_tracker)
    except KeyboardInterrupt:
        pass
    finally:
        color_contour_tracker.destroy_all()
        if rclpy.ok():
            rclpy.shutdown()

if __name__=='__main__':
    main()