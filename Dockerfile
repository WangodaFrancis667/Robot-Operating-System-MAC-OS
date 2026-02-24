# =============================================================================
# ROS 2 Jazzy — macOS Docker Development Image
# =============================================================================
# Base  : Official OSRF ROS 2 Jazzy Desktop (Ubuntu Noble)
# Target: macOS (Apple Silicon & Intel) with XQuartz for GUI forwarding
#
# GUI Strategy:
#   XQuartz on macOS only provides OpenGL 2.1 via indirect GLX.
#   RViz2 / OGRE requires OpenGL 3.3+.
#
#   Solution — three-layer approach:
#     1. Xvfb (:99)      — virtual framebuffer; never shown to the user
#     2. VirtualGL        — intercepts all OpenGL calls, renders offscreen
#                           into Xvfb using Mesa llvmpipe (supports GL 4.5)
#     3. X11 forwarding   — VirtualGL composites finished frames to XQuartz
#                           as plain 2-D X11 images; XQuartz sees no GL at all
#
#   MESA_GL_VERSION_OVERRIDE=3.3 tells OGRE the software renderer supports
#   GL 3.3, preventing the "unable to create OpenGL context" crash.
#
#   Always launch OpenGL apps with: vglrun <app>  (or the rv / rq / gz aliases)
# =============================================================================

FROM osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive

# ---------------------------------------------------------------------------
# System dependencies
# ---------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    # X11 & virtual framebuffer
    x11-apps \
    x11-xserver-utils \
    xauth \
    xvfb \
    # Mesa software renderer (llvmpipe → OpenGL 4.5)
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
    # Developer utilities
    nano \
    vim \
    git \
    curl \
    wget \
    htop \
    tree \
    bash-completion \
    net-tools \
    iputils-ping \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# VirtualGL — architecture-aware install
# Detects amd64 vs arm64 at build time so the same Dockerfile works on
# both Intel Macs and Apple Silicon (via Docker's Rosetta emulation).
# ---------------------------------------------------------------------------
ARG VIRTUALGL_VERSION=3.1.1
RUN ARCH=$(dpkg --print-architecture) \
    && echo "[Dockerfile] Installing VirtualGL ${VIRTUALGL_VERSION} for ${ARCH}" \
    && wget -q \
       "https://github.com/VirtualGL/virtualgl/releases/download/${VIRTUALGL_VERSION}/virtualgl_${VIRTUALGL_VERSION}_${ARCH}.deb" \
       -O /tmp/virtualgl.deb \
    && apt-get install -y /tmp/virtualgl.deb \
    && rm /tmp/virtualgl.deb \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# rosdep initialisation
# ---------------------------------------------------------------------------
RUN rosdep init 2>/dev/null || true && rosdep update

# ---------------------------------------------------------------------------
# Developer user (UID/GID 1000) — avoids root-owned files in host volumes
# Ubuntu Noble ships a default "ubuntu" user at 1000; we rename it to "ros".
# ---------------------------------------------------------------------------
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000

RUN existing_group="$(getent group  ${USER_GID} | cut -d: -f1 || true)" \
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
# Workspace
# ---------------------------------------------------------------------------
RUN mkdir -p /ros2_ws/src \
    && chown -R ${USERNAME}:${USERNAME} /ros2_ws

WORKDIR /ros2_ws

# ---------------------------------------------------------------------------
# System-wide wrapper scripts for GUI apps
# These live in /usr/local/bin/ so they work in ALL shell types
# (interactive, non-interactive, login, exec'd) — not just bash with .bashrc.
# ---------------------------------------------------------------------------
RUN printf '#!/bin/bash\nexec vglrun rviz2 "$@"\n'  > /usr/local/bin/rv  \
    && printf '#!/bin/bash\nexec vglrun rqt "$@"\n'    > /usr/local/bin/rq  \
    && printf '#!/bin/bash\nexec vglrun gazebo "$@"\n' > /usr/local/bin/gz  \
    && chmod +x /usr/local/bin/rv /usr/local/bin/rq /usr/local/bin/gz

# ---------------------------------------------------------------------------
# User shell environment (~/.bashrc)
# ---------------------------------------------------------------------------
RUN echo "" >> /home/${USERNAME}/.bashrc \
    && echo "# ── ROS 2 Jazzy ──────────────────────────────────────────" >> /home/${USERNAME}/.bashrc \
    && echo "source /opt/ros/jazzy/setup.bash" >> /home/${USERNAME}/.bashrc \
    && echo "source /ros2_ws/install/setup.bash 2>/dev/null || true" >> /home/${USERNAME}/.bashrc \
    && echo "export ROS_DOMAIN_ID=0" >> /home/${USERNAME}/.bashrc \
    && echo "export RCUTILS_COLORIZED_OUTPUT=1" >> /home/${USERNAME}/.bashrc \
    \
    && echo "# ── X11 / OpenGL ─────────────────────────────────────────" >> /home/${USERNAME}/.bashrc \
    && echo "export QT_X11_NO_MITSHM=1" >> /home/${USERNAME}/.bashrc \
    && echo "# Xvfb virtual display started by entrypoint.sh" >> /home/${USERNAME}/.bashrc \
    && echo "export VGL_DISPLAY=:99" >> /home/${USERNAME}/.bashrc \
    && echo "# Mesa software renderer — required for OpenGL inside Docker on macOS" >> /home/${USERNAME}/.bashrc \
    && echo "export GALLIUM_DRIVER=llvmpipe" >> /home/${USERNAME}/.bashrc \
    && echo "export LIBGL_ALWAYS_SOFTWARE=1" >> /home/${USERNAME}/.bashrc \
    && echo "# Override reported GL version so OGRE accepts the SW renderer" >> /home/${USERNAME}/.bashrc \
    && echo "export MESA_GL_VERSION_OVERRIDE=3.3" >> /home/${USERNAME}/.bashrc \
    && echo "export MESA_GLSL_VERSION_OVERRIDE=330" >> /home/${USERNAME}/.bashrc \
    \
    && echo "# ── Colcon / ROS aliases ─────────────────────────────────" >> /home/${USERNAME}/.bashrc \
    && echo "alias cb='colcon build --symlink-install'" >> /home/${USERNAME}/.bashrc \
    && echo "alias ct='colcon test'" >> /home/${USERNAME}/.bashrc \
    && echo "alias cs='source install/setup.bash'" >> /home/${USERNAME}/.bashrc \
    && echo "alias rl='ros2 launch'" >> /home/${USERNAME}/.bashrc \
    && echo "alias rr='ros2 run'" >> /home/${USERNAME}/.bashrc \
    && echo "alias rt='ros2 topic'" >> /home/${USERNAME}/.bashrc \
    && echo "alias rn='ros2 node'" >> /home/${USERNAME}/.bashrc \
    && echo "# GUI shortcuts (also available as system commands rv / rq / gz)" >> /home/${USERNAME}/.bashrc \
    && echo "alias rv='vglrun rviz2'" >> /home/${USERNAME}/.bashrc \
    && echo "alias rq='vglrun rqt'" >> /home/${USERNAME}/.bashrc \
    && echo "alias gz='vglrun gazebo'" >> /home/${USERNAME}/.bashrc

# Also source ROS for root sessions used during build-time checks
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

# ---------------------------------------------------------------------------
# Entrypoint
# ---------------------------------------------------------------------------
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

USER ${USERNAME}

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]