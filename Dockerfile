# Use the official OSRF ROS 2 Jazzy image
FROM osrf/ros:jazzy-desktop

# Install additional useful tools
RUN apt-get update && apt-get install -y \
    x11-apps \
    python3-pip \
    nano \
    && rm -rf /var/lib/apt/lists/*

# Set up a workspace directory
WORKDIR /ros2_ws

# Source the ROS environment automatically on entry
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc