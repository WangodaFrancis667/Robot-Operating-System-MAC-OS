#!/bin/bash
# =============================================================================
# Container Entrypoint — ROS 2 Jazzy / macOS Docker Dev Environment
# =============================================================================
# This script is executed every time the container starts.
# It sources the ROS 2 environment, sets up the workspace, and then
# hands off to whatever CMD was passed (default: bash).
# =============================================================================

set -e

# ---------------------------------------------------------------------------
# 0. Start a virtual X framebuffer for VirtualGL's offscreen 3-D rendering
#    VirtualGL renders OpenGL here (Mesa llvmpipe → OpenGL 4.5), then
#    composites the result to XQuartz via the DISPLAY variable.
# ---------------------------------------------------------------------------
Xvfb :99 -screen 0 1920x1080x24 +extension GLX -ac -nolisten tcp &
XVFB_PID=$!
export VGL_DISPLAY=:99
# X11 transport: VirtualGL composites rendered frames to XQuartz directly
# (no vglclient daemon required on the macOS host)
export VGL_TRANSPORT=x11
# Give Xvfb a moment to initialise before any GPU work begins
for _i in $(seq 1 20); do
    xdpyinfo -display :99 &>/dev/null && break
    sleep 0.1
done
unset _i

# ---------------------------------------------------------------------------
# 1. Source the base ROS 2 installation
# ---------------------------------------------------------------------------
source /opt/ros/jazzy/setup.bash

# ---------------------------------------------------------------------------
# 2. Source the local workspace overlay (if it has been built)
# ---------------------------------------------------------------------------
if [ -f "/ros2_ws/install/setup.bash" ]; then
    source /ros2_ws/install/setup.bash
fi

# ---------------------------------------------------------------------------
# 3. Print welcome banner
# ---------------------------------------------------------------------------
echo ""
echo "╔══════════════════════════════════════════════════════════╗"
echo "║        ROS 2 Jazzy — macOS Docker Dev Environment       ║"
echo "╠══════════════════════════════════════════════════════════╣"
echo "║  ROS Distro : $(printenv ROS_DISTRO)"
echo "║  Workspace  : /ros2_ws"
echo "║  ROS Domain : $(printenv ROS_DOMAIN_ID)"
echo "╠══════════════════════════════════════════════════════════╣"
echo "║  GUI apps (always use vglrun for OpenGL apps):          ║"
echo "║    rv        — vglrun rviz2                             ║"
echo "║    rq        — vglrun rqt                               ║"
echo "║    gz        — vglrun gazebo                            ║"
echo "╠══════════════════════════════════════════════════════════╣"
echo "║  ROS 2 aliases:                                         ║"
echo "║    cb  — colcon build --symlink-install                 ║"
echo "║    cs  — source install/setup.bash                      ║"
echo "║    rl  — ros2 launch                                    ║"
echo "║    rr  — ros2 run                                       ║"
echo "║    rt  — ros2 topic                                     ║"
echo "║    rn  — ros2 node                                      ║"
echo "╚══════════════════════════════════════════════════════════╝"
echo ""

# ---------------------------------------------------------------------------
# 4. Execute the command passed to the container (default: bash)
# ---------------------------------------------------------------------------
exec "$@"
