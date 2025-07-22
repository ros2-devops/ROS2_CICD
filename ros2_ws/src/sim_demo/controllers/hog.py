import rclpy
from rclpy.node import Node
import threading
import time

class CpuHog(Node):
    def __init__(self):
        super().__init__('cpu_hog')
        self.get_logger().info('CPU Hog with bursts started.')

        self.duration_sec = 90  # hardcoded runtime limit (no env needed)
        self.start_time = time.time()

        threading.Thread(target=self.stress_loop, daemon=True).start()
        self.create_timer(1.0, self.check_shutdown)

    def stress_loop(self):
        cycle_duration = 15
        stress_duration = 5

        while rclpy.ok():
            now = time.time()
            elapsed = now - self.start_time
            cycle_position = elapsed % cycle_duration

            if cycle_position < stress_duration:
                self.burn_cpu()
            else:
                time.sleep(0.1)

    def burn_cpu(self):
        for _ in range(10000):
            _ = sum(i * i for i in range(100000))

    def check_shutdown(self):
        elapsed = time.time() - self.start_time
        if elapsed >= self.duration_sec:
            self.get_logger().warning(f"Shutting down after {elapsed:.1f}s.")
            self.destroy_node()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = CpuHog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
