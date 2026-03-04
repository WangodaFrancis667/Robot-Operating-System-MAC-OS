# =============================================================================
# ROS 2 Jazzy — Cross-Platform Docker Image (Linux / macOS / Windows)
# =============================================================================
# Base  : Official OSRF ROS 2 Jazzy Desktop (Ubuntu Noble 24.04)
#
# GUI Strategy — Pure Mesa Software Rendering + VNC/noVNC:
#   NO VirtualGL. VirtualGL's GLX transport causes segfaults when both
#   DISPLAY and VGL_DISPLAY point to the same Xvfb (:99).
#
#   Instead we use Mesa's llvmpipe directly:
#     - LIBGL_ALWAYS_SOFTWARE=1    → Mesa llvmpipe handles all OpenGL calls
#     - MESA_GL_VERSION_OVERRIDE=3.3 → tells OGRE/RViz2 GL 3.3 is available
#     - Xvfb (:99)               → virtual framebuffer (no physical display)
#     - x11vnc                   → streams Xvfb:99 over VNC port 5900
#     - noVNC + websockify        → browser access at http://localhost:6080/vnc.html
#
#   This stack works identically on:
#     - macOS (Apple Silicon & Intel) — no XQuartz needed
#     - Linux                          — no display needed
#     - Windows (Docker Desktop/WSL2)  — no VcXsrv/X410 needed
# =============================================================================

FROM osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive

# ---------------------------------------------------------------------------
# System dependencies
# ---------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    # --- X11 / virtual framebuffer ---
    x11-apps \
    x11-xserver-utils \
    xauth \
    xvfb \
    dbus-x11 \
    # --- Qt xcb platform plugin runtime deps (Ubuntu Noble) ---
    # Without these Qt aborts with "could not load platform plugin xcb"
    libxcb-xinerama0 \
    libxcb-cursor0 \
    libxcb-icccm4 \
    libxcb-image0 \
    libxcb-keysyms1 \
    libxcb-randr0 \
    libxcb-render-util0 \
    libxcb-shape0 \
    libxcb-xfixes0 \
    libxkbcommon-x11-0 \
    libx11-xcb1 \
    # --- VNC (x11vnc serves Xvfb; websockify/noVNC = browser UI) ---
    x11vnc \
    python3-websockify \
    # noVNC: install from GitHub release for reliable web root path
    # (Ubuntu Noble's apt package has inconsistent vnc.html location)
    # Installed below in its own RUN step
    # --- Mesa software renderer (llvmpipe — OpenGL 4.5 capable) ---
    mesa-utils \
    libgl1 \
    libgl1-mesa-dri \
    libglu1-mesa \
    libegl-mesa0 \
    libegl1 \
    libglx-mesa0 \
    # --- ROS 2 toolchain ---
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-argcomplete \
    ros-jazzy-ament-cmake \
    # --- Gazebo Harmonic + ROS 2 bridge ---
    ros-jazzy-ros-gz \
    # --- Developer utilities ---
    nano \
    vim \
    git \
    curl \
    wget \
    htop \
    tree \
    bash-completion \
    net-tools \
    iproute2 \
    iputils-ping \
    sudo \
    procps \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# noVNC — install from official GitHub release for a reliable web root.
# The Ubuntu apt package puts vnc.html in different locations across versions.
# We pin to a known release and always find it at /opt/novnc/vnc.html.
# ---------------------------------------------------------------------------
ARG NOVNC_VERSION=1.4.0
RUN wget -qO /tmp/novnc.tar.gz \
        "https://github.com/novnc/noVNC/archive/refs/tags/v${NOVNC_VERSION}.tar.gz" \
    && mkdir -p /opt/novnc \
    && tar -xzf /tmp/novnc.tar.gz -C /opt/novnc --strip-components=1 \
    && rm /tmp/novnc.tar.gz \
    # Create a convenience symlink so websockify --web serves vnc.html at /
    && ln -sf /opt/novnc/vnc.html /opt/novnc/index.html

# ---------------------------------------------------------------------------
# rosdep initialisation
# ---------------------------------------------------------------------------
RUN rosdep init 2>/dev/null || true && rosdep update

# ---------------------------------------------------------------------------
# Developer user (UID/GID 1000) — avoids root-owned files in host volumes.
# Ubuntu Noble ships a default "ubuntu" user at UID 1000; we rename to "ros".
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
# System-wide GUI wrapper scripts — /usr/local/bin/ (works in all shell types)
#
# Key changes vs. old version:
#   - NO vglrun  (VirtualGL removed — causes segfaults with pure Xvfb)
#   - LIBGL_ALWAYS_SOFTWARE=1 ensures Mesa llvmpipe is used for OpenGL
#   - XDG_RUNTIME_DIR set to avoid Qt warnings
# ---------------------------------------------------------------------------
RUN printf '%s\n' \
    '#!/bin/bash' \
    'export DISPLAY=${DISPLAY:-:99}' \
    'export LIBGL_ALWAYS_SOFTWARE=1' \
    'export GALLIUM_DRIVER=llvmpipe' \
    'export MESA_GL_VERSION_OVERRIDE=3.3' \
    'export MESA_GLSL_VERSION_OVERRIDE=330' \
    'export OGRE_RTT_MODE=Copy' \
    'export QT_QPA_PLATFORM=xcb' \
    'export XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR:-/tmp/runtime-ros}' \
    'mkdir -p "$XDG_RUNTIME_DIR" && chmod 700 "$XDG_RUNTIME_DIR"' \
    'exec rviz2 "$@"' \
    > /usr/local/bin/rv \
    && printf '%s\n' \
    '#!/bin/bash' \
    'export DISPLAY=${DISPLAY:-:99}' \
    'export LIBGL_ALWAYS_SOFTWARE=1' \
    'export GALLIUM_DRIVER=llvmpipe' \
    'export QT_QPA_PLATFORM=xcb' \
    'export XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR:-/tmp/runtime-ros}' \
    'mkdir -p "$XDG_RUNTIME_DIR" && chmod 700 "$XDG_RUNTIME_DIR"' \
    'exec rqt "$@"' \
    > /usr/local/bin/rq \
    && printf '%s\n' \
    '#!/bin/bash' \
    'export DISPLAY=${DISPLAY:-:99}' \
    'export LIBGL_ALWAYS_SOFTWARE=1' \
    'export GALLIUM_DRIVER=llvmpipe' \
    'export MESA_GL_VERSION_OVERRIDE=3.3' \
    'export MESA_GLSL_VERSION_OVERRIDE=330' \
    'export LP_NUM_THREADS=4' \
    'export LIBGL_ALWAYS_INDIRECT=0' \
    'export LIBGL_DRI3_DISABLE=1' \
    'export QT_QPA_PLATFORM=xcb' \
    'export XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR:-/tmp/runtime-ros}' \
    'mkdir -p "$XDG_RUNTIME_DIR" && chmod 700 "$XDG_RUNTIME_DIR"' \
    '# Gazebo performance: disable online model downloads, use local resources' \
    'export GAZEBO_MODEL_DATABASE_URI=""' \
    'export GAZEBO_RESOURCE_PATH="/usr/share/gazebo-11/"' \
    'GZ_BIN=$(command -v gz 2>/dev/null)' \
    'if [ -z "$GZ_BIN" ] || [ "$GZ_BIN" = "/usr/local/bin/gz" ]; then' \
    '    GZ_BIN=$(find /usr /opt -maxdepth 8 -name "gz" ! -path "/usr/local/bin/gz" -type f 2>/dev/null | head -1)' \
    'fi' \
    'if [ -z "$GZ_BIN" ]; then echo "Error: gz binary not found" >&2; exit 1; fi' \
    'exec "$GZ_BIN" sim "$@"' \
    > /usr/local/bin/gz \
    && chmod +x /usr/local/bin/rv /usr/local/bin/rq /usr/local/bin/gz

# ---------------------------------------------------------------------------
# User shell environment — ~/.bashrc
# ---------------------------------------------------------------------------
RUN echo '' >> /home/${USERNAME}/.bashrc && \
    echo '# ── ROS 2 Jazzy ──────────────────────────────────────────────────────────' >> /home/${USERNAME}/.bashrc && \
    echo 'source /opt/ros/jazzy/setup.bash' >> /home/${USERNAME}/.bashrc && \
    echo 'source /ros2_ws/install/setup.bash 2>/dev/null || true' >> /home/${USERNAME}/.bashrc && \
    echo 'export ROS_DOMAIN_ID=0' >> /home/${USERNAME}/.bashrc && \
    echo 'export RCUTILS_COLORIZED_OUTPUT=1' >> /home/${USERNAME}/.bashrc && \
    echo '' >> /home/${USERNAME}/.bashrc && \
    echo '# ── Display / OpenGL (Mesa llvmpipe — no VirtualGL) ──────────────────────' >> /home/${USERNAME}/.bashrc && \
    echo '# DISPLAY=:99 → Xvfb virtual framebuffer inside the container.' >> /home/${USERNAME}/.bashrc && \
    echo '# View the desktop in a browser: http://localhost:6080/vnc.html' >> /home/${USERNAME}/.bashrc && \
    echo 'export DISPLAY=:99' >> /home/${USERNAME}/.bashrc && \
    echo 'export LIBGL_ALWAYS_SOFTWARE=1' >> /home/${USERNAME}/.bashrc && \
    echo 'export GALLIUM_DRIVER=llvmpipe' >> /home/${USERNAME}/.bashrc && \
    echo '' >> /home/${USERNAME}/.bashrc && \
    echo '# Mesa performance tuning for software rendering' >> /home/${USERNAME}/.bashrc && \
    echo 'export MESA_GL_VERSION_OVERRIDE=3.3' >> /home/${USERNAME}/.bashrc && \
    echo 'export MESA_GLSL_VERSION_OVERRIDE=330' >> /home/${USERNAME}/.bashrc && \
    echo 'export LP_NUM_THREADS=4' >> /home/${USERNAME}/.bashrc && \
    echo 'export LIBGL_ALWAYS_INDIRECT=0' >> /home/${USERNAME}/.bashrc && \
    echo 'export LIBGL_DRI3_DISABLE=1' >> /home/${USERNAME}/.bashrc && \
    echo '' >> /home/${USERNAME}/.bashrc && \
    echo '# OGRE/RViz2 optimizations' >> /home/${USERNAME}/.bashrc && \
    echo 'export OGRE_RTT_MODE=Copy' >> /home/${USERNAME}/.bashrc && \
    echo 'export QT_QPA_PLATFORM=xcb' >> /home/${USERNAME}/.bashrc && \
    echo '' >> /home/${USERNAME}/.bashrc && \
    echo '# Gazebo performance optimizations' >> /home/${USERNAME}/.bashrc && \
    echo 'export GAZEBO_MODEL_DATABASE_URI=""' >> /home/${USERNAME}/.bashrc && \
    echo 'export GAZEBO_RESOURCE_PATH="/usr/share/gazebo-11/"' >> /home/${USERNAME}/.bashrc && \
    echo '' >> /home/${USERNAME}/.bashrc && \
    echo 'export XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR:-/tmp/runtime-ros}' >> /home/${USERNAME}/.bashrc && \
    echo 'mkdir -p "$XDG_RUNTIME_DIR" && chmod 700 "$XDG_RUNTIME_DIR" 2>/dev/null || true' >> /home/${USERNAME}/.bashrc && \
    echo '' >> /home/${USERNAME}/.bashrc && \
    echo '# ── Colcon / ROS aliases ──────────────────────────────────────────────────' >> /home/${USERNAME}/.bashrc && \
    echo "alias cb='colcon build --symlink-install'" >> /home/${USERNAME}/.bashrc && \
    echo "alias ct='colcon test'" >> /home/${USERNAME}/.bashrc && \
    echo "alias cs='source install/setup.bash'" >> /home/${USERNAME}/.bashrc && \
    echo "alias rl='ros2 launch'" >> /home/${USERNAME}/.bashrc && \
    echo "alias rr='ros2 run'" >> /home/${USERNAME}/.bashrc && \
    echo "alias rt='ros2 topic'" >> /home/${USERNAME}/.bashrc && \
    echo "alias rn='ros2 node'" >> /home/${USERNAME}/.bashrc

# Root sessions (build-time checks)
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

# ---------------------------------------------------------------------------
# Entrypoint
# ---------------------------------------------------------------------------
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Note: Do NOT set USER here - entrypoint runs as root and drops to user

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]