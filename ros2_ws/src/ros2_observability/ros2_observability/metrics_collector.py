#!/usr/bin/env python3
"""
Metrics Collector – Dissertation edition
───────────────────────────────────────
• Logs CPU & Memory *plus* rolling mean and slope (trend).
• Emits **self-describing CSV** with provenance header.
• Tracks threshold violations and writes a compact run-level summary
  to simulation_log_<scenario>.csv  (append only).

CSV layout (after the two-line header):

Time,CPU,Memory,CPU_mean,CPU_slope,Mem_mean,Mem_slope,CPU_viol,Mem_viol
"""

from __future__ import annotations
from datetime import datetime
import os, sys, time, csv, psutil, numpy as np
import rclpy
from rclpy.node import Node

# ────────── static config ─────────────────────────────────────────────
WIN_SEC     = 10                                 # rolling window
THR_CPU     = 85.0                               # violate if >
THR_MEM     = 80.0                               # violate if >
WORK_DIR    = os.getcwd()

class MetricsCollector(Node):
    def __init__(self) -> None:
        super().__init__("metrics_node")

        self.scenario      = os.getenv("SCENARIO", "unknown")
        self.period        = float(os.getenv("LOG_INTERVAL", 1.0))
        self.max_duration  = int(os.getenv("SIM_DURATION", 60))

        self.metrics_file  = os.path.join(WORK_DIR,
                               f"ros_metrics_{self.scenario}.csv")
        self.simlog_file   = os.path.join(WORK_DIR,
                               f"simulation_log_{self.scenario}.csv")

        self.start_time  = time.time()
        self.cpu_hist, self.mem_hist = [], []
        self.cpu_viol, self.mem_viol = 0, 0

        self._bootstrap_files()
        self.timer = self.create_timer(self.period, self._collect)

        self.get_logger().info(
            f"MetricsCollector ⟪{self.scenario}⟫ started "
            f"(T={self.max_duration}s, Δt={self.period}s)")

    # ──────────────────────────────────────────────────────────────────
    def _bootstrap_files(self) -> None:
        if not os.path.exists(self.metrics_file):
            with open(self.metrics_file, "w", newline="") as f:
                wr = csv.writer(f)
                wr.writerow([f"# scenario={self.scenario}",
                             f"# generated={datetime.now().isoformat()}"])
                wr.writerow(["Time", "CPU", "Memory",
                            "CPU_roll", "CPU_slope",
                            "Mem_roll", "Mem_slope"])
        if not os.path.exists(self.simlog_file):
            with open(self.simlog_file, "w") as f:
                f.write("Timestamp,Scenario,Result,CPU_violations,"
                        "Mem_violations,MaxCPU,MaxMem\n")

    # ──────────────────────────────────────────────────────────────────
    def _collect(self) -> None:
        now   = time.time() - self.start_time
        cpu   = psutil.cpu_percent()
        mem   = psutil.virtual_memory().percent

        # keep rolling window
        self.cpu_hist.append(cpu);  self.mem_hist.append(mem)
        if len(self.cpu_hist) > int(WIN_SEC / self.period) + 1:
            self.cpu_hist.pop(0);   self.mem_hist.pop(0)

        cpu_roll = np.mean(self.cpu_hist)
        mem_roll = np.mean(self.mem_hist)

        cpu_slope = (self.cpu_hist[-1] - self.cpu_hist[0]) / len(self.cpu_hist)
        mem_slope = (self.mem_hist[-1] - self.mem_hist[0]) / len(self.mem_hist)

        # threshold counters
        self.cpu_viol += cpu > THR_CPU
        self.mem_viol += mem > THR_MEM

        # append CSV row
        with open(self.metrics_file, "a", newline="") as f:
            csv.writer(f).writerow([
                round(now,3), cpu, mem,
                round(cpu_roll,2), round(cpu_slope,3),
                round(mem_roll,2), round(mem_slope,3),
                self.cpu_viol, self.mem_viol
            ])

        self.get_logger().info(
            f"t={now:5.1f}s | CPU {cpu:5.1f}%  mem {mem:5.1f}% "
            f"(μ {cpu_mean:4.1f}/{mem_mean:4.1f})")

        if now >= self.max_duration:
            self._finalise()

        if len(self.cpu_hist) < 3:
            self.get_logger().warning("Insufficient metrics collected — skipping log")
            return


    # ──────────────────────────────────────────────────────────────────
    def _finalise(self) -> None:
        result = "FAIL" if (self.cpu_viol or self.mem_viol) else "PASS"
        with open(self.simlog_file, "a") as f:
            f.write(f"{datetime.now().isoformat()},{self.scenario},{result},"
                    f"{self.cpu_viol},{self.mem_viol},"
                    f"{max(self.cpu_hist):.1f},{max(self.mem_hist):.1f}\n")

        self.get_logger().warning(
            f"Finished after {self.max_duration}s → {result} "
            f"(CPU viol={self.cpu_viol}, MEM viol={self.mem_viol})")
        rclpy.shutdown()

def main(argv=None) -> None:
    rclpy.init(args=argv)
    try:
        rclpy.spin(MetricsCollector())
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()  
