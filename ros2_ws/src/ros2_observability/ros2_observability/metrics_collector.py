#!/usr/bin/env python3
"""
Dissertation-level Metrics Collector
────────────────────────────────────
• Logs CPU, memory, rolling stats, slopes
• Adds disk, network, system/user CPU time, and temperature
• Logs threshold violations → simulation_log_<scenario>.csv
"""

from __future__ import annotations
from datetime import datetime
import os, csv, time, psutil, numpy as np
import rclpy
from rclpy.node import Node

WIN_SEC  = 10
THR_CPU  = 85.0
THR_MEM  = 80.0
WORK_DIR = os.getcwd()


class MetricsCollector(Node):
    def __init__(self) -> None:
        super().__init__("metrics_node")
        self.scenario     = os.getenv("SCENARIO", "unknown")
        self.period       = float(os.getenv("LOG_INTERVAL", 0.5))
        self.max_duration = int(os.getenv("SIM_DURATION", 120))

        self.metrics_path = os.path.join(WORK_DIR, f"ros_metrics_{self.scenario}.csv")
        self.simlog_path  = os.path.join(WORK_DIR, f"simulation_log_{self.scenario}.csv")
        self.result_file  = os.path.join(WORK_DIR, f'assertion_result_{self.scenario}.txt')

        self.t0 = time.time()
        self.cpu_hist: list[float] = []
        self.mem_hist: list[float] = []
        self.cpu_viol = 0
        self.mem_viol = 0

        self.disk_start = psutil.disk_io_counters()
        self.net_start  = psutil.net_io_counters()

        self._bootstrap_files()
        self.timer = self.create_timer(self.period, self._collect)

        self.get_logger().info(f"[START] scenario={self.scenario}, T={self.max_duration}s, dt={self.period}s")

    def _bootstrap_files(self) -> None:
        if not os.path.exists(self.metrics_path):
            with open(self.metrics_path, "w", newline="") as f:
                csv.writer(f).writerow([
                    "Time", "CPU", "Memory",
                    "CPU_roll", "CPU_slope", "Mem_roll", "Mem_slope",
                    "CPU_user", "CPU_sys", "CPU_count", "CPU_freq",
                    "Disk_read", "Disk_write",
                    "Net_sent", "Net_recv", "Temperature",
                    "CPU_viol", "Mem_viol"
                ])
        if not os.path.exists(self.simlog_path):
            with open(self.simlog_path, "w") as f:
                f.write("Timestamp,Scenario,Result,CPU_violations,Mem_violations,MaxCPU,MaxMem\n")

    def _collect(self) -> None:
        now = round(time.time() - self.t0, 3)
        cpu = psutil.cpu_percent()
        mem = psutil.virtual_memory().percent

        self.cpu_hist.append(cpu)
        self.mem_hist.append(mem)
        max_len = int(WIN_SEC / self.period) + 1
        if len(self.cpu_hist) > max_len:
            self.cpu_hist.pop(0)
            self.mem_hist.pop(0)

        cpu_roll = float(np.mean(self.cpu_hist))
        mem_roll = float(np.mean(self.mem_hist))
        cpu_slope = (self.cpu_hist[-1] - self.cpu_hist[0]) / len(self.cpu_hist)
        mem_slope = (self.mem_hist[-1] - self.mem_hist[0]) / len(self.mem_hist)

        # Additional metrics
        cpu_times = psutil.cpu_times_percent()
        cpu_user = cpu_times.user
        cpu_sys  = cpu_times.system
        cpu_count = psutil.cpu_count()
        cpu_freq  = psutil.cpu_freq().current if psutil.cpu_freq() else -1

        disk = psutil.disk_io_counters()
        disk_read  = (disk.read_bytes - self.disk_start.read_bytes) / (1024 ** 2)
        disk_write = (disk.write_bytes - self.disk_start.write_bytes) / (1024 ** 2)

        net = psutil.net_io_counters()
        net_sent = (net.bytes_sent - self.net_start.bytes_sent) / (1024 ** 2)
        net_recv = (net.bytes_recv - self.net_start.bytes_recv) / (1024 ** 2)

        temps = psutil.sensors_temperatures() if hasattr(psutil, "sensors_temperatures") else {}
        temp = -1
        for name, entries in temps.items():
            for entry in entries:
                if "cpu" in entry.label.lower() or "core" in entry.label.lower():
                    temp = entry.current
                    break

        self.cpu_viol += cpu > THR_CPU
        self.mem_viol += mem > THR_MEM

        with open(self.metrics_path, "a", newline="") as f:
            csv.writer(f).writerow([
                now, cpu, mem,
                round(cpu_roll, 2), round(cpu_slope, 3),
                round(mem_roll, 2), round(mem_slope, 3),
                round(cpu_user, 2), round(cpu_sys, 2),
                cpu_count, round(cpu_freq, 1),
                round(disk_read, 2), round(disk_write, 2),
                round(net_sent, 2), round(net_recv, 2),
                round(temp, 1),
                self.cpu_viol, self.mem_viol
            ])

        if now >= self.max_duration:
            self._finalise()

    def _finalise(self) -> None:
        result = "FAIL" if (self.cpu_viol or self.mem_viol) else "PASS"
        with open(self.simlog_path, "a") as f:
            f.write(f"{datetime.now().isoformat()},{self.scenario},{result},"
                    f"{self.cpu_viol},{self.mem_viol},"
                    f"{max(self.cpu_hist):.1f},{max(self.mem_hist):.1f}\n")
        with open(self.result_file, 'w') as f:
            f.write(result)

        self.get_logger().warning(
            f"Finished ({result}) — CPU_viol={self.cpu_viol}, MEM_viol={self.mem_viol}"
        )
        rclpy.shutdown()


def main(argv=None) -> None:
    rclpy.init(args=argv)
    try:
        rclpy.spin(MetricsCollector())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
