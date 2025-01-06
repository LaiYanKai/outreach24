import rclpy
import rclpy.node
import sensor_msgs
from ultralytics import YOLO

import numpy as np
import cv2
import cv_bridge

bridge = cv_bridge.CvBridge()

class TrafficSignYOLO(rclpy.node.Node):

    def __init__(self):
        super().__init__('traffic')
        self.timer = self.create_timer(0.2, self.cbTimer)
        
        self.subImage = self.create_subscription(sensor_msgs.msg.Image, '/camera/image_raw', self.cbImage, 1)
        self.subImage

    def cbTimer(self):
        pass

    def cbImage(self, msg):
        self.get_logger().info('got msg')
        self.cv_image = bridge.imgmsg_to_cv2(msg, 'bgra8')
        self.get_logger().info('got msg2')


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(TrafficSignYOLO())
    rclpy.shutdown()


if __name__ == '__main__':
    main()