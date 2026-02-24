# =============================================================================
# ROS 2 Jazzy — macOS Docker Development Image
# =============================================================================
# Base: Official OSRF ROS 2 Jazzy Desktop (Ubuntu Noble)
# Includes: Full ROS 2 desktop, RViz2, GUI via X11 + VirtualGL on macOS
# Target : macOS (Intel & Apple Silicon) with XQuartz for GUI forwarding
#
# GUI Strategy:
#   XQuartz (macOS) only provides OpenGL 2.1 via indirect GLX.
#   RViz2 / OGRE requires OpenGL 3.3+.
#   VirtualGL intercepts all GLX/OpenGL calls, renders offscreen using
#   Mesa llvmpipe (software, OpenGL 4.5), then ships the 2-D result to
#   XQuartz as a plain X11 image.  No GPU or special kernel module needed.
#   Launch GUI apps with:  vglrun rviz2   or use the 'rv', 'rq' aliases.
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
    # Virtual framebuffer — provides a local X server for VirtualGL's 3D rendering
    xvfb \
    # Mesa OpenGL — llvmpipe software renderer (OpenGL 4.5, used by VirtualGL)
    mesa-utils \
    libgl1 \
    libgl1-mesa-dri \
    libglu1-mesa \
    libegl-mesa0 \
    libegl1 \
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
# VirtualGL 2.6.5 — offscreen OpenGL rendering, composited to XQuartz via X11
# 2.6.x is the last stable series whose .deb ships all transport plugins.
# Proxy transport (VGL_TRANSPORT=proxy) = X11 transport; no vglclient needed.
# Ubuntu 24.04 does not package virtualgl, so we pull the upstream .deb.
# ---------------------------------------------------------------------------
ARG VIRTUALGL_VERSION=2.6.5
RUN wget -q "https://github.com/VirtualGL/virtualgl/releases/download/${VIRTUALGL_VERSION}/virtualgl_${VIRTUALGL_VERSION}_amd64.deb" \
    -O /tmp/virtualgl.deb \
    && apt-get install -y /tmp/virtualgl.deb \
    && rm /tmp/virtualgl.deb \
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

# Ubuntu 24.04 (Noble) already ships with GID/UID 1000 (user "ubuntu").
# Rename or create the group/user so our "ros" user always owns UID:GID 1000.
RUN apt-get update && apt-get install -y sudo && rm -rf /var/lib/apt/lists/* \
    && existing_group="$(getent group  ${USER_GID} | cut -d: -f1 || true)" \
    && if   [ -z "$existing_group" ]; then \
         groupadd --gid ${USER_GID} ${USERNAME}; \
       elif [ "$existing_group" != "${USERNAME}" ]; then \
         groupmod -n ${USERNAME} "$existing_group"; \
       fi \
    && existing_user="$(getent passwd ${USER_UID} | cut -d: -f1 || true)" \
    && if   [ -z "$existing_user" ]; then \
         useradd --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash ${USERNAME}; \
       elif [ "$existing_user" != "${USERNAME}" ]; then \
         usermod -l ${USERNAME} -d /home/${USERNAME} -m "$existing_user"; \
       fi \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

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
    && echo "export QT_X11_NO_MITSHM=1" >> /home/${USERNAME}/.bashrc \
    && echo "# VirtualGL / Mesa software rendering" >> /home/${USERNAME}/.bashrc \
    && echo "# Xvfb runs on :99 (started by /entrypoint.sh) — VirtualGL renders there" >> /home/${USERNAME}/.bashrc \
    && echo "export VGL_DISPLAY=:99" >> /home/${USERNAME}/.bashrc \
    && echo "# X11 transport: composite frames directly to XQuartz (no vglclient needed)" >> /home/${USERNAME}/.bashrc \
    && echo "export VGL_TRANSPORT=x11" >> /home/${USERNAME}/.bashrc \
    && echo "export GALLIUM_DRIVER=llvmpipe" >> /home/${USERNAME}/.bashrc \
    && echo "export LIBGL_ALWAYS_SOFTWARE=1" >> /home/${USERNAME}/.bashrc \
    && echo "" >> /home/${USERNAME}/.bashrc \
    && echo "# Convenient aliases" >> /home/${USERNAME}/.bashrc \
    && echo "alias cb='colcon build --symlink-install'" >> /home/${USERNAME}/.bashrc \
    && echo "alias ct='colcon test'" >> /home/${USERNAME}/.bashrc \
    && echo "alias cs='source install/setup.bash'" >> /home/${USERNAME}/.bashrc \
    && echo "alias rl='ros2 launch'" >> /home/${USERNAME}/.bashrc \
    && echo "alias rr='ros2 run'" >> /home/${USERNAME}/.bashrc \
    && echo "alias rt='ros2 topic'" >> /home/${USERNAME}/.bashrc \
    && echo "alias rn='ros2 node'" >> /home/${USERNAME}/.bashrc \
    && echo "# GUI shortcuts (always use vglrun for OpenGL apps)" >> /home/${USERNAME}/.bashrc \
    && echo "alias rv='vglrun rviz2'" >> /home/${USERNAME}/.bashrc \
    && echo "alias rq='vglrun rqt'" >> /home/${USERNAME}/.bashrc \
    && echo "alias gz='vglrun gazebo'" >> /home/${USERNAME}/.bashrc

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