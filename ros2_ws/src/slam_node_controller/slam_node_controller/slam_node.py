import rclpy
from rclpy.node import Node
import time
import threading

class Slam_nodeNode(Node):
    def __init__(self):
        super().__init__('slam_node')
        self.get_logger().info('Simulates SLAM navigation started.')
        threading.Thread(target=self.run, daemon=True).start()

    def run(self):
        while rclpy.ok():
            self.get_logger().info('Simulates SLAM navigation logic running...')
            time.sleep(5)

def main(args=None):
    rclpy.init(args=args)
    node = Slam_nodeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
