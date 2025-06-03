##############################
# Stage 1 — BUILD
##############################
FROM ubuntu:22.04 AS builder
ENV DEBIAN_FRONTEND=noninteractive

# ── Base tools ───────────────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y \
    curl gnupg2 lsb-release locales wget software-properties-common git bzip2
RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# ── ROS 2 Humble repo ────────────────────────────────────────────────────────
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
 | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
 http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
 > /etc/apt/sources.list.d/ros2.list

# ── Webots repo ──────────────────────────────────────────────────────────────
RUN wget -qO /usr/share/keyrings/webots-keyring.asc \
 https://cyberbotics.com/Cyberbotics.asc && \
 echo "deb [signed-by=/usr/share/keyrings/webots-keyring.asc] \
 https://cyberbotics.com/debian/ binary-amd64/" \
 > /etc/apt/sources.list.d/webots.list

# ── Install ROS 2 + Webots + dev tools (incl. vision_msgs) ───────────────────
RUN apt-get update && apt-get install -y \
    ros-humble-ros-base \
    ros-humble-vision-msgs \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    webots

# ── Workspace setup ──────────────────────────────────────────────────────────
WORKDIR /ros2_ws
COPY ros2_ws/src ./src

# Clone webots_ros2 tag 2023.1.3 with sub-modules (Humble compatible)
RUN git clone --branch 2023.1.3 --recurse-submodules \
    https://github.com/cyberbotics/webots_ros2.git src/webots_ros2

# ── Resolve ROS deps ─────────────────────────────────────────────────────────
RUN rosdep init && rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

# ── Build workspace ──────────────────────────────────────────────────────────
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

##############################
# Stage 2 — RUNTIME
##############################
FROM ubuntu:22.04
ENV DEBIAN_FRONTEND=noninteractive

# Base utilities
RUN apt-get update && apt-get install -y \
    curl gnupg2 lsb-release locales wget software-properties-common git bzip2
RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# ROS 2 repo
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
 | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
 http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
 > /etc/apt/sources.list.d/ros2.list

# Webots repo
RUN wget -qO /usr/share/keyrings/webots-keyring.asc \
 https://cyberbotics.com/Cyberbotics.asc && \
 echo "deb [signed-by=/usr/share/keyrings/webots-keyring.asc] \
 https://cyberbotics.com/debian/ binary-amd64/" \
 > /etc/apt/sources.list.d/webots.list

# Install runtime ROS 2 + Webots + vision_msgs
RUN apt-get update && apt-get install -y \
    ros-humble-ros-base \
    ros-humble-vision-msgs \
    webots

# ── Copy built workspace ─────────────────────────────────────────────────────
COPY --from=builder /ros2_ws/install /ros2_ws/install

# ── Default entrypoint: run Webots sim headless ──────────────────────────────
ENTRYPOINT ["/bin/bash", "-c", \
  "source /opt/ros/humble/setup.bash && \
   source /ros2_ws/install/setup.bash && \
   ros2 launch sim_demo webots_sim.launch.py"]
