# =============================================================================
# ROS 2 Jazzy — macOS Docker Development Image
# =============================================================================
# Base: Official OSRF ROS 2 Jazzy Desktop (Ubuntu Noble)
# Includes: Full ROS 2 desktop, RViz2, Gazebo-compatible GUI support via X11
# Target : macOS (Intel & Apple Silicon) with XQuartz for GUI forwarding
# =============================================================================

FROM osrf/ros:jazzy-desktop

# Prevent interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# ---------------------------------------------------------------------------
# System dependencies & developer tooling
# ---------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    # X11 GUI forwarding utilities
    x11-apps \
    x11-xserver-utils \
    xauth \
    mesa-utils \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    # ROS 2 build tools
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-argcomplete \
    ros-jazzy-ament-cmake \
    # Useful CLI utilities
    nano \
    vim \
    git \
    curl \
    wget \
    htop \
    tree \
    bash-completion \
    # Networking / diagnostics
    net-tools \
    iputils-ping \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# rosdep initialisation (ignore "already initialised" errors on rebuild)
# ---------------------------------------------------------------------------
RUN rosdep init 2>/dev/null || true && rosdep update

# ---------------------------------------------------------------------------
# Create an unprivileged developer user that mirrors typical UID 1000
# This avoids root-owned files appearing in mounted host volumes
# ---------------------------------------------------------------------------
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd  --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
    && apt-get update && apt-get install -y sudo \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# Workspace setup
# ---------------------------------------------------------------------------
RUN mkdir -p /ros2_ws/src

WORKDIR /ros2_ws

# Give the ros user ownership of the workspace
RUN chown -R ${USERNAME}:${USERNAME} /ros2_ws

# ---------------------------------------------------------------------------
# Shell environment — auto-source ROS 2 for every interactive session
# ---------------------------------------------------------------------------
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/${USERNAME}/.bashrc \
    && echo "source /ros2_ws/install/setup.bash 2>/dev/null || true" >> /home/${USERNAME}/.bashrc \
    && echo "export ROS_DOMAIN_ID=0" >> /home/${USERNAME}/.bashrc \
    && echo "export RCUTILS_COLORIZED_OUTPUT=1" >> /home/${USERNAME}/.bashrc \
    && echo "" >> /home/${USERNAME}/.bashrc \
    && echo "# Convenient aliases" >> /home/${USERNAME}/.bashrc \
    && echo "alias cb='colcon build --symlink-install'" >> /home/${USERNAME}/.bashrc \
    && echo "alias ct='colcon test'" >> /home/${USERNAME}/.bashrc \
    && echo "alias cs='source install/setup.bash'" >> /home/${USERNAME}/.bashrc \
    && echo "alias rl='ros2 launch'" >> /home/${USERNAME}/.bashrc \
    && echo "alias rr='ros2 run'" >> /home/${USERNAME}/.bashrc \
    && echo "alias rt='ros2 topic'" >> /home/${USERNAME}/.bashrc \
    && echo "alias rn='ros2 node'" >> /home/${USERNAME}/.bashrc

# Also source for root sessions (used during container startup checks)
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

# ---------------------------------------------------------------------------
# Copy & register the container entrypoint
# ---------------------------------------------------------------------------
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

USER ${USERNAME}

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]