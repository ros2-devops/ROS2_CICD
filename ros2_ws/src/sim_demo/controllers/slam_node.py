#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class SlamNavigator(Node):
    def __init__(self):
        super().__init__('slam_navigator')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.25, self.move)
        self.counter = 0

    def move(self):
        msg = Twist()
        msg.linear.x = 0.2 if self.counter < 5 else -0.2
        self.publisher.publish(msg)
        self.counter = (self.counter + 1) % 10

def main():
    rclpy.init()
    node = SlamNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
