#!/bin/bash
set -e

# Start Xvfb for headless Webots
Xvfb :99 -screen 0 1024x768x16 &

# Source ROS and launch your simulation
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

# Let user override the CMD (e.g., for ros2 launch or ros2 run)
exec "$@"
