#!/usr/bin/env python3
from datetime import datetime
import rclpy
from rclpy.node import Node
import psutil
import csv
import time
import os
import sys

class MetricsCollector(Node):
    def __init__(self):
        super().__init__('metrics_node')
        self.metrics_file = os.path.expanduser('~/ros_metrics.csv')
        self.result_file = os.path.expanduser('~/assertion_result.txt')
        self.timer_period = 1.0
        self.max_duration = int(os.getenv('SIM_DURATION', 10))  # fallback
        self.cpu_threshold = 85.0
        self.mem_threshold = 80.0
        self.cpu_violations = 0
        self.memory_violations = 0
        self.get_logger().info("MetricsCollector node started.")
        self.cpu_log = []
        self.mem_log = []
        self.start_time = time.time()
        self.timer = self.create_timer(self.timer_period, self.collect_metrics)

    def collect_metrics(self):
        elapsed = time.time() - self.start_time
        cpu = psutil.cpu_percent()
        mem = psutil.virtual_memory().percent
        self.cpu_log.append(cpu)
        self.mem_log.append(mem)

        if cpu > self.cpu_threshold:
            self.cpu_violations += 1
        if mem > self.mem_threshold:
            self.memory_violations += 1

        self.get_logger().info(f"[Metrics] CPU: {cpu:.1f}%, Mem: {mem:.1f}%")

        with open(self.metrics_file, 'a', newline='') as f:
            csv.writer(f).writerow([elapsed, cpu, mem])

        if elapsed >= self.max_duration:
            self.evaluate()

    def evaluate(self):
        result = "PASS"
        if self.cpu_violations >= 2 or self.memory_violations > 0:
            result = "FAIL"
        with open(self.result_file, 'w') as f:
            f.write(result)
        self.get_logger().info(f"[Evaluation] CPU violations: {self.cpu_violations}, Mem violations: {self.memory_violations}")
        self.get_logger().info(f"[Assertion Result] {result}")
        rclpy.shutdown()
        sys.exit(0 if result == "PASS" else 1)
        timestamp = datetime.now().isoformat()
        scenario = os.getenv("SCENARIO", "unknown")
        log_file = os.path.expanduser("~/simulation_log.csv")
        with open(log_file, 'a') as f:
            f.write(f"{timestamp},{scenario},{result}\n")

def main(args=None):
    rclpy.init(args=args)
    node = MetricsCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
