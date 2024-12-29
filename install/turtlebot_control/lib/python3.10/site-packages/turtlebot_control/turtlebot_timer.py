#!/usr/bin/env python3

import rclpy
from std_msgs.msg import String
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist

class Timer_subscriber(Node):
    
    def __init__(self):
        super().__init__('TimerNode')
        timer_period = 0.5  # seconds
        self.last_message = Twist()
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.subs = self.create_subscription(Twist, "/cmd_vel", self.sub_callback, 10)

    def sub_callback(self, msg):
        self.last_message = msg

    def timer_callback(self):
        self.get_logger().info(f"Linear velocity x: {self.last_message.linear.x}, Angular velocity z: {self.last_message.angular.z})")
        
def main(args=None):
    rclpy.init(args=args)
    tim_sub = Timer_subscriber()
    rclpy.spin(tim_sub)
    tim_sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()