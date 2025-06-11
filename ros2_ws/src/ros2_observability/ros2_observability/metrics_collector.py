#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import psutil
import csv
import time
import os
import sys

class MetricsCollector(Node):
    def __init__(self):
        super().__init__('metrics_collector')
        self.metrics_file = os.path.expanduser("~/ros_metrics.csv")
        self.timer = self.create_timer(1.0, self.collect_metrics)
        self.start_time = time.time()
        self.duration = 10  # seconds to run metrics
        self.cpu_usage_values = []
        self.memory_usage_values = []
        self.get_logger().info("MetricsCollector node started.")

    def collect_metrics(self):
        elapsed = time.time() - self.start_time
        cpu = psutil.cpu_percent()
        memory = psutil.virtual_memory().percent
        self.cpu_usage_values.append(cpu)
        self.memory_usage_values.append(memory)

        self.get_logger().info(f"[Metrics] CPU: {cpu}%, Memory: {memory}%")

        # Write to CSV
        with open(self.metrics_file, mode='a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([elapsed, cpu, memory])

        if elapsed >= self.duration:
            self.timer.cancel()
            self.evaluate_metrics()

    def evaluate_metrics(self):
        avg_cpu = sum(self.cpu_usage_values) / len(self.cpu_usage_values)
        avg_mem = sum(self.memory_usage_values) / len(self.memory_usage_values)

        self.get_logger().info(f"[Evaluation] Avg CPU: {avg_cpu:.2f}%, Avg Mem: {avg_mem:.2f}%")

        # Runtime assertion: fail if average CPU > 90% or memory > 85%
        if avg_cpu > 90 or avg_mem > 85:
            self.get_logger().error("[ASSERTION FAILED] Resource usage too high.")
            rclpy.shutdown()
            sys.exit(1)
        else:
            self.get_logger().info("[ASSERTION PASSED] System metrics are within limits.")
            rclpy.shutdown()
            sys.exit(0)

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
