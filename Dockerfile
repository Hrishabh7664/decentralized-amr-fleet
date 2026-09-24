# Multi-Stage Dockerfile for Decentralized Multi-AMR Fleet Simulation and Dashboard
FROM osrf/ros:humble-desktop-full

# Set environment
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV WORKSPACE=/root/ros2_ws
SHELL ["/bin/bash", "-c"]

# Install system dependencies, Gazebo Classic, rosbridge, Nav2, Python tools, and Node.js
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    git \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-rosbridge-suite \
    ros-humble-nav2-msgs \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-tf2-tools \
    ros-humble-rmw-cyclonedds-cpp \
    net-tools \
    iputils-ping \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN pip3 install --no-cache-dir \
    pytest \
    numpy \
    pyyaml

# Install Node.js 18.x for Dashboard
RUN curl -fsSL https://deb.nodesource.com/setup_18.x | bash - \
    && apt-get install -y --no-install-recommends nodejs \
    && rm -rf /var/lib/apt/lists/*

# Create ROS 2 workspace
WORKDIR ${WORKSPACE}
RUN mkdir -p ${WORKSPACE}/src

# Copy project source into container
COPY src/ ${WORKSPACE}/src/
COPY scripts/ ${WORKSPACE}/scripts/
COPY config/ ${WORKSPACE}/config/ 2>/dev/null || true

# Initialize rosdep and install package dependencies
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && apt-get update \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -r -y \
    && rm -rf /var/lib/apt/lists/*

# Build colcon workspace with custom ROS 2 messages
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source workspace on entry
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc \
    && echo "source ${WORKSPACE}/install/setup.bash" >> /root/.bashrc \
    && echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /root/.bashrc

# Expose ports:
# 9090: rosbridge WebSocket
# 3000: Web Monitoring Dashboard
# 11345: Gazebo Master (if running remote)
EXPOSE 9090 3000 11345

# Default entrypoint launches 3-robot simulation in headless mode
CMD ["/bin/bash", "-c", "source ${WORKSPACE}/install/setup.bash && ros2 launch amr_fleet fleet_3.launch.py gui:=false"]
