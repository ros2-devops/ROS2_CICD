#!/usr/bin/env python3
"""
ROS 2 Metrics Collector
‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
• Samples CPU / memory once per second
• Fails if CPU > 85 % twice **or** Memory > 80 % once
• Writes:
    - ros_metrics.csv          (time-series samples)
    - assertion_result.txt     (PASS / FAIL)
    - simulation_log.csv       (timestamp, scenario, PASS/FAIL)
All files are saved in os.getcwd() so that CI can pick them up easily.
"""
from datetime import datetime
import os
import sys
import time
import csv
import psutil
import rclpy
from rclpy.node import Node

WORK_DIR = os.getcwd()                     # ← GitHub Actions workspace path

class MetricsCollector(Node):
    def __init__(self):
        super().__init__('metrics_node')

        scenario = os.getenv("SCENARIO", "unknown")
        # ---------- file locations ----------
        self.metrics_file = os.path.join(WORK_DIR, f'ros_metrics_{scenario}.csv')
        self.result_file  = os.path.join(WORK_DIR, f'assertion_result_{scenario}.txt')
        self.log_file     = os.path.join(WORK_DIR, f'simulation_log_{scenario}.csv')

        

        # ---------- settings ----------
        self.timer_period  = float(os.getenv('LOG_INTERVAL', 1.0))
        self.max_duration  = int(os.getenv('SIM_DURATION', 10))
        self.cpu_threshold = 85.0
        self.mem_threshold = 80.0

        # ---------- state ----------
        self.cpu_violations  = 0
        self.mem_violations  = 0
        self.cpu_log: list[float] = []
        self.mem_log: list[float] = []
        self.start_time = time.time()

        self.get_logger().info("MetricsCollector node started.")
        self.timer = self.create_timer(self.timer_period, self.collect)

    # --------------------------------------------------

    def collect(self):
        elapsed = time.time() - self.start_time
        cpu     = psutil.cpu_percent()
        mem     = psutil.virtual_memory().percent
        cloud_metrics_path = os.path.join(os.getcwd(), "cloud_metrics.csv")
        timestamp = datetime.now().isoformat()
        scenario = os.getenv("SCENARIO", "unknown")

        with open(cloud_metrics_path, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([timestamp, scenario, cpu, mem])

        self.cpu_log.append(cpu)
        self.mem_log.append(mem)

        if cpu > self.cpu_threshold:
            self.cpu_violations += 1
        elif cpu > self.cpu_threshold - 5:
            self.get_logger().warn(f"CPU near limit: {cpu:.1f}%")

        if mem > self.mem_threshold:
            self.mem_violations += 1
        elif mem > self.mem_threshold - 5:
            self.get_logger().warn(f"Mem near limit: {mem:.1f}%")

        self.get_logger().info(f"[Metrics] CPU {cpu:.1f} %, Mem {mem:.1f} %")

        # append sample
        with open(self.metrics_file, 'a', newline='') as f:
            csv.writer(f).writerow([elapsed, cpu, mem])

        if elapsed >= self.max_duration:
            self.evaluate()

    # --------------------------------------------------

    def evaluate(self):
        result = "PASS"
        if self.cpu_violations >= 2 or self.mem_violations > 0:
            result = "FAIL"

        # write assertion_result.txt
        with open(self.result_file, 'w') as f:
            f.write(result)

        # append run summary to simulation_log.csv
        timestamp = datetime.now().isoformat()
        scenario  = os.getenv("SCENARIO", "unknown")
        with open(self.log_file, 'a') as f:
            f.write(f"{timestamp},{scenario},{result}\n")

        self.get_logger().info(
            f"[Evaluation] CPU viol: {self.cpu_violations}, "
            f"Mem viol: {self.mem_violations}"
        )
        self.get_logger().info(f"[Assertion Result] {result}")

        # graceful shutdown with appropriate exit code
        rclpy.shutdown()
        sys.exit(0 if result == "PASS" else 1)

# ------------------------------------------------------

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
