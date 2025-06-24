#!/usr/bin/env python3
"""
ROS 2 Metrics Collector
• Logs runtime CPU/Memory usage
• Evaluates pass/fail based on thresholds
• Outputs:
    - ros_metrics_<scenario>.csv
    - dataset/training_data_<scenario>.csv
    - dataset/training_data_all.csv
    - assertion_result_<scenario>.txt
    - simulation_log_<scenario>.csv
"""

from datetime import datetime
import os
import sys
import time
import csv
import psutil
import rclpy
from rclpy.node import Node

WORK_DIR = os.getcwd()
DATASET_DIR = os.path.join(WORK_DIR, "dataset")
os.makedirs(DATASET_DIR, exist_ok=True)

class MetricsCollector(Node):
    def __init__(self):
        super().__init__('metrics_node')

        self.scenario = os.getenv("SCENARIO", "unknown")

        # Output paths
        self.metrics_file     = os.path.join(WORK_DIR, f'ros_metrics_{self.scenario}.csv')
        self.result_file      = os.path.join(WORK_DIR, f'assertion_result_{self.scenario}.txt')
        self.log_file         = os.path.join(WORK_DIR, f'simulation_log_{self.scenario}.csv')
        self.training_file    = os.path.join(DATASET_DIR, f"training_data_{self.scenario}.csv")
        self.training_file_all = os.path.join(DATASET_DIR, "training_data_all.csv")

        # Config
        self.timer_period  = float(os.getenv('LOG_INTERVAL', 1.0))
        self.max_duration  = int(os.getenv('SIM_DURATION', 10))
        self.cpu_threshold = 85.0
        self.mem_threshold = 80.0

        # State
        self.cpu_violations = 0
        self.mem_violations = 0
        self.start_time = time.time()

        self.get_logger().info("MetricsCollector node started.")
        self.timer = self.create_timer(self.timer_period, self.collect)

    def collect(self):
        elapsed = time.time() - self.start_time
        cpu = psutil.cpu_percent()
        mem = psutil.virtual_memory().percent
        timestamp = datetime.now().isoformat()

        # Threshold warnings
        if cpu > self.cpu_threshold:
            self.cpu_violations += 1
        elif cpu > self.cpu_threshold - 5:
            self.get_logger().warn(f"CPU near limit: {cpu:.1f}%")

        if mem > self.mem_threshold:
            self.mem_violations += 1
        elif mem > self.mem_threshold - 5:
            self.get_logger().warn(f"Mem near limit: {mem:.1f}%")

        self.get_logger().info(f"[Metrics] CPU {cpu:.1f} %, Mem {mem:.1f} %")

        row = [elapsed, cpu, mem]

        # Log per-run evaluation data
        with open(self.metrics_file, 'a', newline='') as f:
            csv.writer(f).writerow(row)

        # Append to training dataset (per-scenario and global)
        with open(self.training_file, 'a', newline='') as f:
            csv.writer(f).writerow(row)
        with open(self.training_file_all, 'a', newline='') as f:
            csv.writer(f).writerow(row)

        if elapsed >= self.max_duration:
            self.evaluate()

    def evaluate(self):
        result = "PASS"
        if self.cpu_violations >= 2 or self.mem_violations > 0:
            result = "FAIL"

        # Save assertion result
        with open(self.result_file, 'w') as f:
            f.write(result)

        # Log summary
        timestamp = datetime.now().isoformat()
        with open(self.log_file, 'a') as f:
            f.write(f"{timestamp},{self.scenario},{result}\n")

        self.get_logger().info(
            f"[Evaluation] CPU viol: {self.cpu_violations}, "
            f"Mem viol: {self.mem_violations}"
        )
        self.get_logger().info(f"[Assertion Result] {result}")

        rclpy.shutdown()
        sys.exit(0 if result == "PASS" else 1)

# ─────────────────────────────

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
