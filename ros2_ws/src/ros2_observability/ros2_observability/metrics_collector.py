import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import psutil
import time
import csv


class MetricsCollector(Node):
    def __init__(self):
        super().__init__('metrics_collector')
        self.declare_parameter('log_interval', 2.0)
        interval = self.get_parameter('log_interval').get_parameter_value().double_value

        self.log_timer = self.create_timer(interval, self.log_metrics)

        self.cpu_log = []
        self.mem_log = []

        self.get_logger().info('✅ MetricsCollector started...')

    def log_metrics(self):
        cpu = psutil.cpu_percent()
        mem = psutil.virtual_memory().percent
        timestamp = self.get_clock().now().to_msg()

        self.get_logger().info(f'[Metrics] CPU: {cpu}% | Memory: {mem}%')

        self.cpu_log.append((timestamp.sec, cpu))
        self.mem_log.append((timestamp.sec, mem))
        with open('/home/ec24136/ros_metrics.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Timestamp', 'CPU (%)', 'Memory (%)'])
            for ts, c in zip(self.cpu_log, self.mem_log):
                writer.writerow([ts[0], ts[1], c[1]])

def main(args=None):
    rclpy.init(args=args)
    node = MetricsCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
