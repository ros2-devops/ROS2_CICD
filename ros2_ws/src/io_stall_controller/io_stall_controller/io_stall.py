import rclpy
from rclpy.node import Node
from controller import Robot
import random
import time

class IOStallController(Node):
    def __init__(self):
        super().__init__('io_stall')
        self.get_logger().info("IO Stall controller started")
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        self.start_loop()

    def start_loop(self):
        while rclpy.ok() and self.robot.step(self.timestep) != -1:
            if random.random() < 0.01:
                sleep_duration = random.uniform(0.05, 0.2)
                self.get_logger().info(f"Stalling for {sleep_duration:.3f}s")
                time.sleep(sleep_duration)

def main(args=None):
    rclpy.init(args=args)
    IOStallController()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
