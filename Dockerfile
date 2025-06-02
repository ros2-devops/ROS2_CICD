##########################
# Stage 1 — Build stage
##########################

FROM ubuntu:22.04 AS builder

# Install system tools
RUN apt-get update && apt-get install -y curl gnupg2 lsb-release locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Add ROS 2 Humble repo
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | \
    gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros2.list

# Install ROS 2 Humble and build tools
RUN apt-get update && apt-get install -y ros-humble-ros-base python3-colcon-common-extensions python3-pip

# Setup ROS workspace
WORKDIR /ros2_ws
COPY ros2_ws/src ./src

# Install dependencies
RUN rosdep init && rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

# Build ROS 2 workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

##########################
# Stage 2 — Runtime stage
##########################

FROM ubuntu:22.04

# Repeat ROS installation in runtime stage
RUN apt-get update && apt-get install -y curl gnupg2 lsb-release locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | \
    gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros2.list

RUN apt-get update && apt-get install -y ros-humble-ros-base

# Copy built workspace from build stage
COPY --from=builder /ros2_ws/install /ros2_ws/install

ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && ros2 launch sim_demo sim.launch.py"]
