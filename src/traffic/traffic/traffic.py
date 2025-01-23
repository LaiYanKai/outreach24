# import rclpy
# import rclpy.node
# import sensor_msgs
# from ultralytics import YOLO
# from ultralytics.engine.results import Results

# import numpy as np
# import cv2
# import cv_bridge

# bridge = cv_bridge.CvBridge()

# class TrafficSignYOLO(rclpy.node.Node):

#     def __init__(self):
#         super().__init__('traffic')
#         self.timer = self.create_timer(0.2, self.cbTimer)
#         self.subImage = self.create_subscription(sensor_msgs.msg.Image, '/camera/image_raw', self.cbImage, 1)
#         self.subImage
#         self.model = YOLO("/home/rhesa/outreach24/best.pt")
#         self.cv_image = None


#     def cbTimer(self):
#         if not self.cv_image is None:
#             cv2.imshow('image',self.cv_image)
#             cv2.waitKey(1)
        

#     def cbImage(self, msg):
#         self.get_logger().info('got msg')
#         self.cv_image = bridge.imgmsg_to_cv2(msg, 'bgr8')
#         results = self.model(self.cv_image)
#         numpy_result = results[0].numpy()
#         print(numpy_result.boxes.xyxy)
#         if len(numpy_result.boxes.xyxy) > 0:
#             for box in range(len(numpy_result.boxes.xyxy)):
#                 self.cv_image = cv2.rectangle(self.cv_image,
#                     (int(numpy_result.boxes.xyxy[box][0]),int(numpy_result.boxes.xyxy[box][1])),
#                     (int(numpy_result.boxes.xyxy[box][2]),int(numpy_result.boxes.data[box][3])),
#                     (0,255,255),2)
#                 self.get_logger().info(f'Class={results[0].boxes.cls}')
# def main(args=None):
#     rclpy.init(args=args)
#     rclpy.spin(TrafficSignYOLO())
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()

import rclpy
import rclpy.node
import sensor_msgs
from ultralytics import YOLO
from ultralytics.engine.results import Results

import numpy as np
import cv2
import cv_bridge

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from geometry_msgs.msg import Twist

from math import pi

bridge = cv_bridge.CvBridge()

class TrafficSignYOLO(rclpy.node.Node):

    def __init__(self):
        super().__init__('traffic')
        self.timer = self.create_timer(0.2, self.cbTimer)
        self.subImage = self.create_subscription(sensor_msgs.msg.Image, '/camera/image_raw', self.cbImage, 1)
        self.subImage
        self.model = YOLO("/home/rhesa/outreach24/best.pt")
        self.cv_image = None
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sign = -1
        self.i = -1
        self.repeat = 5


    def cbTimer(self):
        if not self.cv_image is None:
            # if self.i % (self.repeat) == 0:
            #     cv2.imshow('image',self.cv_image)
            #     cv2.waitKey(1)
            msg = Twist()
            if self.sign == 0:
                msg.linear.x = -0.01
                msg.linear.z = 0.0
            elif self.sign == 1:
                msg.angular.z = pi/9
                msg.linear.x = 0.0
            elif self.sign == 2:
                msg.angular.z = -pi/9
                msg.linear.x = 0.0
            elif self.sign == 3:
                msg.linear.x = 0.2
                msg.angular.z = 0.0
            elif self.sign == 4:
                msg.linear.x = 0.05
                msg.angular.z = 0.0
            else:
                msg.linear.x = 0.0
                msg.angular.z = 0.0
            self.publisher_.publish(msg)

        
    def cbImage(self, msg):
        self.i += 1
        if self.i % self.repeat != 0 :
            return
        print(self.i)
        self.get_logger().info('got msg')
        self.cv_image = bridge.imgmsg_to_cv2(msg, 'bgr8')
        results = self.model(self.cv_image)
        numpy_result = results[0].numpy()
        self.sign = -1
        print(numpy_result.boxes.xyxy)
        if len(numpy_result.boxes.xyxy) > 0:
            for box in range(len(numpy_result.boxes.xyxy)):
                self.cv_image = cv2.rectangle(self.cv_image,
                    (int(numpy_result.boxes.xyxy[box][0]),int(numpy_result.boxes.xyxy[box][1])),
                    (int(numpy_result.boxes.xyxy[box][2]),int(numpy_result.boxes.data[box][3])),
                    (0,255,255),2)
                self.sign = results[0][0].boxes.cls.item()
                self.get_logger().info(f'Class={results[0].boxes.cls}')
def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(TrafficSignYOLO())
    rclpy.shutdown()


if __name__ == '__main__':
    main()