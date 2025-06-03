##############################
# Stage 1 — Build stage
##############################

FROM ubuntu:22.04 AS builder

ENV DEBIAN_FRONTEND=noninteractive

# Install system tools
RUN apt-get update && apt-get install -y \
    curl gnupg2 lsb-release locales wget software-properties-common git bzip2

RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Add ROS 2 Humble repo key
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | \
    gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros2.list

# Install ROS 2 Humble, colcon, rosdep
RUN apt-get update && apt-get install -y \
    ros-humble-ros-base \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep

# Install Webots from official Cyberbotics tarball (official headless release)
RUN wget https://github.com/cyberbotics/webots/releases/download/R2023b/webots-R2023b-ubuntu-22.04-x86-64.tar.bz2 && \
    tar -xjf webots-R2023b-ubuntu-22.04-x86-64.tar.bz2 && \
    mv webots-R2023b /usr/local/webots && \
    ln -s /usr/local/webots/webots /usr/bin/webots && \
    ln -s /usr/local/webots/webots-bin /usr/bin/webots-bin

# Setup Webots env variables (optional)
ENV WEBOTS_HOME /usr/local/webots
ENV PATH $PATH:/usr/local/webots

# Setup workspace
WORKDIR /ros2_ws
COPY ros2_ws/src ./src

# Clone webots_ros2 repo inside workspace
RUN git clone https://github.com/cyberbotics/webots_ros2.git src/webots_ros2

# Install ROS 2 dependencies
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
    curl gnupg2 lsb-release locales wget software-properties-common git bzip2

RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Add ROS 2 Humble repo key
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | \
    gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros2.list

# Install ROS 2 runtime + Webots runtime
RUN apt-get update && apt-get install -y ros-humble-ros-base

# Install Webots runtime in runtime stage
RUN wget https://github.com/cyberbotics/webots/releases/download/R2023b/webots-R2023b-ubuntu-22.04-x86-64.tar.bz2 && \
    tar -xjf webots-R2023b-ubuntu-22.04-x86-64.tar.bz2 && \
    mv webots-R2023b /usr/local/webots && \
    ln -s /usr/local/webots/webots /usr/bin/webots && \
    ln -s /usr/local/webots/webots-bin /usr/bin/webots-bin

ENV WEBOTS_HOME /usr/local/webots
ENV PATH $PATH:/usr/local/webots

# Copy built workspace
COPY --from=builder /ros2_ws/install /ros2_ws/install

ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && ros2 launch sim_demo webots_sim.launch.py"]
