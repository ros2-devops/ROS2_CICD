##############################
# Stage 1 — Build stage
##############################

FROM ubuntu:22.04 AS builder

ENV DEBIAN_FRONTEND=noninteractive

# Install base system tools
RUN apt-get update && apt-get install -y \
    curl gnupg2 lsb-release locales wget software-properties-common git

RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Add ROS 2 Humble repo key
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros2.list

# Install ROS 2 Humble
RUN apt-get update && apt-get install -y \
    ros-humble-ros-base python3-colcon-common-extensions python3-pip python3-rosdep

# Install Webots Headless
RUN wget -qO - https://cyberbotics.com/Cyberbotics.asc | apt-key add - && \
    add-apt-repository "deb https://cyberbotics.com/debian/ binary-amd64/" && \
    apt-get update && \
    apt-get install -y webots webots-headless

# Setup workspace
WORKDIR /ros2_ws
COPY ros2_ws/src ./src

# Clone webots_ros2 package
RUN git clone https://github.com/cyberbotics/webots_ros2.git src/webots_ros2

# Install dependencies
RUN rosdep init && rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

# Build workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

##############################
# Stage 2 — Runtime stage
##############################

FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    curl gnupg2 lsb-release locales wget software-properties-common git

RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Add ROS 2 Humble
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros2.list

RUN apt-get update && apt-get install -y \
    ros-humble-ros-base webots webots-headless

# Copy built workspace
COPY --from=builder /ros2_ws/install /ros2_ws/install

ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && ros2 launch sim_demo webots_sim.launch.py"]
