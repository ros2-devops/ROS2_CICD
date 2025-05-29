# Stage 1: build
FROM ros:humble-ros-base AS builder

# Setup workspace
WORKDIR /ros2_ws
COPY ros2_ws/src ./src

# Install dependencies
RUN apt update && apt install -y \
    python3-colcon-common-extensions \
    python3-pip \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -r -y

# Build
RUN . /opt/ros/humble/setup.sh && colcon build

# Stage 2: runtime
FROM ros:humble-ros-base

# Copy built workspace
COPY --from=builder /ros2_ws/install /ros2_ws/install

# Source + launch
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && ros2 launch sim_demo sim.launch.py"]
