# hog.py
import rclpy
from rclpy.node import Node
import threading
import time

class CpuHog(Node):
    def __init__(self):
        super().__init__('cpu_hog')
        self.get_logger().info('CPU Hog started.')
        threading.Thread(target=self.burn_cpu, daemon=True).start()

    def burn_cpu(self):
        while rclpy.ok():
            _ = sum(i * i for i in range(500000))  # simulate CPU load
            time.sleep(0.01)

def main(args=None):
    rclpy.init(args=args)
    node = CpuHog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
