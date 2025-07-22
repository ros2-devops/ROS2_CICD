import rclpy
from rclpy.node import Node
from controller import Robot
import os
import time

class RAMHogController(Node):
    def __init__(self):
        super().__init__('ram_hog')
        self.get_logger().info("RAM Hog controller started")

        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())

        # Optional duration from environment
        self.duration = int(os.getenv("SIM_DURATION", "60"))
        self.max_mb = 200  # Cap memory usage
        self.leak = []
        self.count = 0
        self.start_time = time.time()

        self.run()

    def run(self):
        while rclpy.ok() and self.robot.step(self.timestep) != -1:
            elapsed = time.time() - self.start_time
            if elapsed > self.duration:
                self.get_logger().info("Time limit reached, exiting.")
                break

            if self.count % 100 == 0 and len(self.leak) < self.max_mb:
                self.leak.append([0] * 1000000)
                self.get_logger().info(f"Leaked ~{len(self.leak)} MB")
            self.count += 1

def main(args=None):
    rclpy.init(args=args)
    RAMHogController()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
