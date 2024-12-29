#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import cv2
import numpy as np

WINDOW_WIDTH = 800
WINDOW_HEIGHT = 600
RADIUS = 10
WINDOW_NAME = "Control window"
RADIUS_BTN = 50
FORWARD_BTN = (100, 425)
BACKWARD_BTN = (300, 425)
LEFT_ROTATION_BTN = (500, 425)
RIGHT_ROTATION_BTN = (700, 425)

class Position_publisher(Node):
    
    def __init__(self):
        super().__init__('PositionNode')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW_NAME, WINDOW_WIDTH, WINDOW_HEIGHT)
        cv2.setMouseCallback(WINDOW_NAME, self.mouse_callback)
        self.img = np.zeros((WINDOW_HEIGHT, WINDOW_WIDTH,3), dtype=np.uint8)

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.img = np.zeros((WINDOW_HEIGHT, WINDOW_WIDTH,3), dtype=np.uint8)
            msg = String()
            robot_msg = Twist()
            msg.data = "x: %d, y: %d" % (x, y)
            robot_msg.linear.y = 0.0
            robot_msg.linear.z = 0.0
            robot_msg.angular.x = 0.0
            robot_msg.angular.y = 0.0
            
            cv2.circle(self.img, (x, y), RADIUS, (255, 0, 0), 3)

            if (x - FORWARD_BTN[0])**2 + (y - FORWARD_BTN[1])**2 <= RADIUS_BTN**2:
                cv2.putText(self.img, "FORWARD", (50,50), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(255,255,255), thickness=2)
                robot_msg.linear.x = 0.3
                robot_msg.angular.z = 0.0
                self.publisher.publish(robot_msg)
            elif (x - BACKWARD_BTN[0])**2 + (y - BACKWARD_BTN[1])**2 <= RADIUS_BTN**2:
                cv2.putText(self.img, "BACKWARD", (50,50), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(255,255,255), thickness=2)
                robot_msg.linear.x = -0.3
                robot_msg.angular.z = 0.0
                self.publisher.publish(robot_msg)
            elif (x - LEFT_ROTATION_BTN[0])**2 + (y - LEFT_ROTATION_BTN[1])**2 <= RADIUS_BTN**2:
                cv2.putText(self.img, "LEFT ROTATION", (50,50), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(255,255,255), thickness=2)
                robot_msg.linear.x = 0.0
                robot_msg.angular.z = 0.3
                self.publisher.publish(robot_msg)
            elif (x - RIGHT_ROTATION_BTN[0])**2 + (y - RIGHT_ROTATION_BTN[1])**2 <= RADIUS_BTN**2:
                cv2.putText(self.img, "RIGHT ROTATION", (50,50), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(255,255,255), thickness=2)
                robot_msg.linear.x = 0.0
                robot_msg.angular.z = -0.3
                self.publisher.publish(robot_msg)
            else:
                cv2.putText(self.img, "STOP", (50,50), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(255,255,255), thickness=2)
                robot_msg.linear.x = 0.0
                robot_msg.angular.z = 0.0
                self.publisher.publish(robot_msg)
            self.get_logger().info(msg.data)

    def run(self):
        while rclpy.ok():
            
            cv2.circle(self.img, FORWARD_BTN, RADIUS_BTN, (0, 0, 255), 3)
            cv2.circle(self.img, BACKWARD_BTN, RADIUS_BTN, (0, 0, 255), 3)
            cv2.circle(self.img, LEFT_ROTATION_BTN, RADIUS_BTN, (0, 0, 255), 3)
            cv2.circle(self.img, RIGHT_ROTATION_BTN, RADIUS_BTN, (0, 0, 255), 3)
            cv2.putText(self.img, "FORWARD", (FORWARD_BTN[0]-RADIUS_BTN, FORWARD_BTN[1]+70), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.6, color=(0,0,255), thickness=2)
            cv2.putText(self.img, "BACKWARD", (BACKWARD_BTN[0]-RADIUS_BTN, BACKWARD_BTN[1]+70), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.6, color=(0,0,255), thickness=2)
            cv2.putText(self.img, "ROTATE LEFT", (LEFT_ROTATION_BTN[0]-RADIUS_BTN, LEFT_ROTATION_BTN[1]+70), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.6, color=(0,0,255), thickness=2)
            cv2.putText(self.img, "ROTATE RIGHT", (RIGHT_ROTATION_BTN[0]-RADIUS_BTN, RIGHT_ROTATION_BTN[1]+70), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.6, color=(0,0,255), thickness=2)
            cv2.imshow(WINDOW_NAME, self.img)
            if cv2.waitKey(1) == 27:  # ESC kończy działanie
                break
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    pos_publisher = Position_publisher()
    # tim_publisher = Timer_publisher()
    pos_publisher.run()

    rclpy.spin(pos_publisher)
    # rclpy.spin(tim_publisher)
    rclpy.shutdown()
    


if __name__ == '__main__':
    main()