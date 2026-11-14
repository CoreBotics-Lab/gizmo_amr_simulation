#!/usr/bin/env python3

import os
os.environ["OMP_NUM_THREADS"] = "2"
os.environ["OPENBLAS_NUM_THREADS"] = "2"

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

from rclpy.executors import SingleThreadedExecutor

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
        
        DEFAULT_HSV = {
            'h_min': 22, 'h_max': 38,
            's_min': 100, 's_max': 255,
            'v_min': 100, 'v_max': 255
        }
        cv2.createTrackbar('H_min', 'HSV Selector', DEFAULT_HSV['h_min'], 179, lambda x: None)
        cv2.createTrackbar('H_max', 'HSV Selector', DEFAULT_HSV['h_max'], 179, lambda x: None)
        cv2.createTrackbar('S_min', 'HSV Selector', DEFAULT_HSV['s_min'], 255, lambda x: None)
        cv2.createTrackbar('S_max', 'HSV Selector', DEFAULT_HSV['s_max'], 255, lambda x: None)
        cv2.createTrackbar('V_min', 'HSV Selector', DEFAULT_HSV['v_min'], 255, lambda x: None)
        cv2.createTrackbar('V_max', 'HSV Selector', DEFAULT_HSV['v_max'], 255, lambda x: None)


        self.get_logger().info("CV Node started. Waiting for '/camera/image_raw'...")

    def image_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        height, width = frame.shape[:2]
        center_x = width // 2

        # Look only at the lower portion (e.g. from 70% to 90% of image height)
        roi_top = int(height * 0.7)
        roi_bottom = int(height * 0.9)

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        h_min = cv2.getTrackbarPos('H_min', 'HSV Selector')
        h_max = cv2.getTrackbarPos('H_max', 'HSV Selector')
        s_min = cv2.getTrackbarPos('S_min', 'HSV Selector')
        s_max = cv2.getTrackbarPos('S_max', 'HSV Selector')
        v_min = cv2.getTrackbarPos('V_min', 'HSV Selector')
        v_max = cv2.getTrackbarPos('V_max', 'HSV Selector')

        lower_hsv = np.array([h_min, s_min, v_min])
        upper_hsv = np.array([h_max, s_max, v_max])

        # Full-frame HSV mask for complete visibility during tuning
        mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
        mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

        # Constrain ROI mask for result frame and contour tracking
        roi_mask = np.zeros_like(mask)
        roi_mask[roi_top:roi_bottom, :] = mask[roi_top:roi_bottom, :]
        result = cv2.bitwise_and(frame, frame, mask=roi_mask)

        # Draw ROI boundary on the frames
        cv2.rectangle(frame, (0, roi_top), (width, roi_bottom), (0, 255, 255), 2)
        cv2.rectangle(mask_bgr, (0, roi_top), (width, roi_bottom), (0, 255, 255), 2)
        cv2.rectangle(result, (0, roi_top), (width, roi_bottom), (0, 255, 255), 2)

        contours, _ = cv2.findContours(roi_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
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

class ControllerNode(Node):
    def __init__(self)->None:
        super().__init__('Controller_node')
        self.error_sub = self.create_subscription(Float32, '/centroid_error', self.controller_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.linear_velocity = 0.15
        self.angular_velocity = 0.0
        self.Kp = 4.0

    def controller_callback(self, msg: Float32):
        if msg is None:
            return

        error: float = msg.data

        self.get_logger().info(f'error: {error}')
        cmd = Twist()
        self.angular_velocity = -self.Kp * error
        cmd.linear.x = self.linear_velocity       # Constant forward velocity
        cmd.angular.z = float(self.angular_velocity) # Steering correction
        self.cmd_vel_pub.publish(cmd)

def main(args=None)->None:
    rclpy.init(args=args)
    executor = SingleThreadedExecutor()
    color_contour_tracker = ColorContourTracker()
    controller_node = ControllerNode()

    try:
        executor.add_node(color_contour_tracker)
        executor.add_node(controller_node)

        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        color_contour_tracker.destroy_all()
        controller_node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()

if __name__=='__main__':
    main()