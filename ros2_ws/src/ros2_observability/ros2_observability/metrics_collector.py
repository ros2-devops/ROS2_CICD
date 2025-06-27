#!/usr/bin/env python3
"""
Enhanced ROS 2 Metrics Collector
• Logs raw CPU / Memory
• Adds rolling–mean and slope (trend) features
• Saves per-scenario CSV:
      Time, CPU, Memory, CPU_roll, CPU_slope, Mem_roll, Mem_slope
"""

from datetime import datetime
import os, sys, time, csv, psutil, numpy as np
import rclpy
from rclpy.node import Node

WINDOW_SEC   = 10          # rolling window length
VIOLATE_CPU  = 85.0
VIOLATE_MEM  = 80.0
WORK_DIR     = os.getcwd()

class MetricsCollector(Node):
    def __init__(self):
        super().__init__('metrics_node')

        self.scenario      = os.getenv("SCENARIO", "unknown")
        self.metrics_file  = os.path.join(
            WORK_DIR, f"ros_metrics_{self.scenario}.csv")

        # timers
        self.period        = float(os.getenv("LOG_INTERVAL", 1.0))
        self.max_duration  = int(os.getenv("SIM_DURATION", 60))

        # rolling buffers
        self.cpu_hist: list[float] = []
        self.mem_hist: list[float] = []
        self.start_time   = time.time()

        # write header once
        if not os.path.exists(self.metrics_file):
            with open(self.metrics_file, "w", newline="") as f:
                csv.writer(f).writerow([
                    "Time","CPU","Memory",
                    "CPU_roll","CPU_slope",
                    "Mem_roll","Mem_slope"
                ])

        self.timer = self.create_timer(self.period, self.collect)

    # ------------------------------------
    def collect(self):
        now  = time.time() - self.start_time
        cpu  = psutil.cpu_percent()
        mem  = psutil.virtual_memory().percent

        # update history
        self.cpu_hist.append(cpu)
        self.mem_hist.append(mem)
        if len(self.cpu_hist) > WINDOW_SEC / self.period + 1:
            self.cpu_hist.pop(0); self.mem_hist.pop(0)

        # rolling + slope
        cpu_roll = np.mean(self.cpu_hist)
        mem_roll = np.mean(self.mem_hist)
        cpu_slope = (self.cpu_hist[-1] - self.cpu_hist[0]) / len(self.cpu_hist)
        mem_slope = (self.mem_hist[-1] - self.mem_hist[0]) / len(self.mem_hist)

        # write row
        with open(self.metrics_file, "a", newline="") as f:
            csv.writer(f).writerow([
                round(now,3), cpu, mem,
                round(cpu_roll,2), round(cpu_slope,3),
                round(mem_roll,2), round(mem_slope,3)
            ])

        # simple console feedback
        self.get_logger().info(
            f"CPU {cpu:.1f}%  mem {mem:.1f}%  "
            f"(roll {cpu_roll:.1f}/{mem_roll:.1f})")

        if now >= self.max_duration:
            self.get_logger().info("Reached SIM_DURATION → shutting down")
            rclpy.shutdown()


def main(argv=None):
    rclpy.init(args=argv)
    node = MetricsCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()
