# hog.py
import rclpy
from rclpy.node import Node
import threading
import time

class CpuHog(Node):
    def __init__(self):
        super().__init__('cpu_hog')
        self.get_logger().info('CPU Hog with bursts started.')
        self.start_time = time.time()
        threading.Thread(target=self.stress_loop, daemon=True).start()

    def stress_loop(self):
        cycle_duration = 10  # every 10 seconds
        stress_duration = 3  # stress lasts for 3 seconds

        while rclpy.ok():
            now = time.time()
            elapsed = now - self.start_time
            cycle_position = elapsed % cycle_duration

            if cycle_position < stress_duration:
                self.burn_cpu()
            else:
                time.sleep(0.1)

    def burn_cpu(self):
        # Burn CPU for a very short slice of time to keep it responsive
        for _ in range(1000):
            _ = sum(i * i for i in range(10000))

def main(args=None):
    rclpy.init(args=args)
    node = CpuHog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
