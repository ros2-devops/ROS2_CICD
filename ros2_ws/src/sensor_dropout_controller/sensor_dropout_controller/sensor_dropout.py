import rclpy
from rclpy.node import Node
from controller import Robot
import random

class SensorDropoutController(Node):
    def __init__(self):
        super().__init__('sensor_dropout')
        self.get_logger().info("Sensor Dropout controller started")
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())

        self.gps = self.robot.getDevice("gps")
        self.compass = self.robot.getDevice("compass")

        self.gps.enable(self.timestep)
        self.compass.enable(self.timestep)

        now = self.robot.getTime()
        self.drop_start = now + random.uniform(5, 15)
        self.drop_end = self.drop_start + random.uniform(2, 4)

        self.start_loop()

    def start_loop(self):
        while rclpy.ok() and self.robot.step(self.timestep) != -1:
            now = self.robot.getTime()
            if self.drop_start <= now <= self.drop_end:
                self.gps.disable()
                self.compass.disable()
                self.get_logger().info("Sensors dropped")
            else:
                if self.gps.getSamplingPeriod() == 0:
                    self.gps.enable(self.timestep)
                if self.compass.getSamplingPeriod() == 0:
                    self.compass.enable(self.timestep)

def main(args=None):
    rclpy.init(args=args)
    SensorDropoutController()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
