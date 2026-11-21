#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class Apriltag_Tracker(Node):
    def __init__(self) -> None:
        super().__init__('apriltag_tracker')
        



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
