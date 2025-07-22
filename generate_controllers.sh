#!/bin/bash
set -euo pipefail
echo '🛠️ Creating controller packages...'
mkdir -p ros2_ws/src
cd ros2_ws/src

# ===== io_stall_controller =====
mkdir -p io_stall_controller/io_stall_controller
mkdir -p io_stall_controller/resource
touch io_stall_controller/resource/io_stall_controller
cat > io_stall_controller/setup.py <<EOF
from setuptools import setup

package_name = 'io_stall_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ec24136',
    maintainer_email='ec24136@qmul.ac.uk',
    description='Simulates I/O stalls in a robot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'io_stall = io_stall_controller.io_stall:main',
        ],
    },
)
EOF
cat > io_stall_controller/package.xml <<EOF
<?xml version='1.0'?>
<package format='3'>
  <name>io_stall_controller</name>
  <version>0.0.1</version>
  <description>Simulates I/O stalls in a robot</description>
  <maintainer email='ec24136@qmul.ac.uk'>ec24136</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <exec_depend>rclpy</exec_depend>
</package>
EOF
cat > io_stall_controller/io_stall_controller/io_stall.py <<EOF
import rclpy
from rclpy.node import Node
import time
import threading

class Io_stallNode(Node):
    def __init__(self):
        super().__init__('io_stall')
        self.get_logger().info('Simulates I/O stalls in a robot started.')
        threading.Thread(target=self.run, daemon=True).start()

    def run(self):
        while rclpy.ok():
            self.get_logger().info('Simulates I/O stalls in a robot logic running...')
            time.sleep(5)

def main(args=None):
    rclpy.init(args=args)
    node = Io_stallNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF

# ===== ram_hog_controller =====
mkdir -p ram_hog_controller/ram_hog_controller
mkdir -p ram_hog_controller/resource
touch ram_hog_controller/resource/ram_hog_controller
cat > ram_hog_controller/setup.py <<EOF
from setuptools import setup

package_name = 'ram_hog_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ec24136',
    maintainer_email='ec24136@qmul.ac.uk',
    description='Simulates RAM stress on a robot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'ram_hog = ram_hog_controller.ram_hog:main',
        ],
    },
)
EOF
cat > ram_hog_controller/package.xml <<EOF
<?xml version='1.0'?>
<package format='3'>
  <name>ram_hog_controller</name>
  <version>0.0.1</version>
  <description>Simulates RAM stress on a robot</description>
  <maintainer email='ec24136@qmul.ac.uk'>ec24136</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <exec_depend>rclpy</exec_depend>
</package>
EOF
cat > ram_hog_controller/ram_hog_controller/ram_hog.py <<EOF
import rclpy
from rclpy.node import Node
import time
import threading

class Ram_hogNode(Node):
    def __init__(self):
        super().__init__('ram_hog')
        self.get_logger().info('Simulates RAM stress on a robot started.')
        threading.Thread(target=self.run, daemon=True).start()

    def run(self):
        while rclpy.ok():
            self.get_logger().info('Simulates RAM stress on a robot logic running...')
            time.sleep(5)

def main(args=None):
    rclpy.init(args=args)
    node = Ram_hogNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF

# ===== sensor_dropout_controller =====
mkdir -p sensor_dropout_controller/sensor_dropout_controller
mkdir -p sensor_dropout_controller/resource
touch sensor_dropout_controller/resource/sensor_dropout_controller
cat > sensor_dropout_controller/setup.py <<EOF
from setuptools import setup

package_name = 'sensor_dropout_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ec24136',
    maintainer_email='ec24136@qmul.ac.uk',
    description='Simulates sensor dropout',
    license='MIT',
    entry_points={
        'console_scripts': [
            'sensor_dropout = sensor_dropout_controller.sensor_dropout:main',
        ],
    },
)
EOF
cat > sensor_dropout_controller/package.xml <<EOF
<?xml version='1.0'?>
<package format='3'>
  <name>sensor_dropout_controller</name>
  <version>0.0.1</version>
  <description>Simulates sensor dropout</description>
  <maintainer email='ec24136@qmul.ac.uk'>ec24136</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <exec_depend>rclpy</exec_depend>
</package>
EOF
cat > sensor_dropout_controller/sensor_dropout_controller/sensor_dropout.py <<EOF
import rclpy
from rclpy.node import Node
import time
import threading

class Sensor_dropoutNode(Node):
    def __init__(self):
        super().__init__('sensor_dropout')
        self.get_logger().info('Simulates sensor dropout started.')
        threading.Thread(target=self.run, daemon=True).start()

    def run(self):
        while rclpy.ok():
            self.get_logger().info('Simulates sensor dropout logic running...')
            time.sleep(5)

def main(args=None):
    rclpy.init(args=args)
    node = Sensor_dropoutNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF

# ===== slam_node_controller =====
mkdir -p slam_node_controller/slam_node_controller
mkdir -p slam_node_controller/resource
touch slam_node_controller/resource/slam_node_controller
cat > slam_node_controller/setup.py <<EOF
from setuptools import setup

package_name = 'slam_node_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ec24136',
    maintainer_email='ec24136@qmul.ac.uk',
    description='Simulates SLAM navigation',
    license='MIT',
    entry_points={
        'console_scripts': [
            'slam_node = slam_node_controller.slam_node:main',
        ],
    },
)
EOF
cat > slam_node_controller/package.xml <<EOF
<?xml version='1.0'?>
<package format='3'>
  <name>slam_node_controller</name>
  <version>0.0.1</version>
  <description>Simulates SLAM navigation</description>
  <maintainer email='ec24136@qmul.ac.uk'>ec24136</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <exec_depend>rclpy</exec_depend>
</package>
EOF
cat > slam_node_controller/slam_node_controller/slam_node.py <<EOF
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
EOF


# Move IO stall
mv ros2_ws/src/sim_demo/controllers/io_stall.py ros2_ws/src/io_stall_controller/io_stall_controller/io_stall.py

# Move RAM hog
mv ros2_ws/src/sim_demo/controllers/ram_hog.py ros2_ws/src/ram_hog_controller/ram_hog_controller/ram_hog.py

# Move Sensor Dropout
mv ros2_ws/src/sim_demo/controllers/sensor_dropout.py ros2_ws/src/sensor_dropout_controller/sensor_dropout_controller/sensor_dropout.py

# Move SLAM node
mv ros2_ws/src/sim_demo/controllers/slam_node.py ros2_ws/src/slam_node_controller/slam_node_controller/slam_node.py

echo '✅ Done creating controller packages.'
cd ../..
colcon build
source install/setup.bash
