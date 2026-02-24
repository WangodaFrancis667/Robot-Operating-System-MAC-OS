#!/bin/bash
# =============================================================================
# Container Entrypoint — ROS 2 Jazzy / macOS Docker Dev Environment
# =============================================================================

set -e

# ---------------------------------------------------------------------------
# 0. Core environment — export everything VirtualGL + Mesa need.
#    docker-compose injects these, but we re-export here explicitly so they
#    are guaranteed to be present for every child process (vglrun, rviz2…).
# ---------------------------------------------------------------------------
export GALLIUM_DRIVER=llvmpipe
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE=3.3   # Tells OGRE the SW renderer supports GL 3.3+
export MESA_GLSL_VERSION_OVERRIDE=330
export QT_X11_NO_MITSHM=1
# Force VirtualGL to use plain X11 transport — vglclient is not running on macOS
export VGL_TRANSPORT=x11

# ---------------------------------------------------------------------------
# 1. Start a virtual X framebuffer on display :99.
#    VirtualGL renders OpenGL here (Mesa llvmpipe → OpenGL 4.5).
#    Each finished frame is composited to XQuartz as plain X11 pixels —
#    XQuartz never sees a single OpenGL / GLX call.
# ---------------------------------------------------------------------------
echo "[entrypoint] Starting Xvfb on display :99 ..."

# Clean up any stale lock from a previous container run
rm -f /tmp/.X99-lock /tmp/.X11-unix/X99

Xvfb :99 -screen 0 1920x1080x24 +extension GLX +extension RANDR -ac &
XVFB_PID=$!
export VGL_DISPLAY=:99

# Wait up to 5 seconds for Xvfb to become ready
XVFB_READY=0
for i in $(seq 1 50); do
    if xdpyinfo -display :99 &>/dev/null; then
        XVFB_READY=1
        break
    fi
    sleep 0.1
done

if [ "$XVFB_READY" -eq 0 ]; then
    echo "[entrypoint] ERROR: Xvfb failed to start on display :99."
    echo "             GUI apps (rviz2, rqt, gazebo) will not work."
    echo "             Check that xvfb is installed: apt list --installed | grep xvfb"
else
    echo "[entrypoint] Xvfb is ready on display :99 (PID $XVFB_PID)."
fi

# ---------------------------------------------------------------------------
# 2. Source the base ROS 2 installation
# ---------------------------------------------------------------------------
source /opt/ros/jazzy/setup.bash

# ---------------------------------------------------------------------------
# 3. Source the local workspace overlay (if it has been built)
# ---------------------------------------------------------------------------
if [ -f "/ros2_ws/install/setup.bash" ]; then
    source /ros2_ws/install/setup.bash
fi

# ---------------------------------------------------------------------------
# 4. Print welcome banner
# ---------------------------------------------------------------------------
echo ""
echo "╔══════════════════════════════════════════════════════════╗"
echo "║        ROS 2 Jazzy — macOS Docker Dev Environment       ║"
echo "╠══════════════════════════════════════════════════════════╣"
printf "║  ROS Distro : %-42s ║\n" "$(printenv ROS_DISTRO)"
printf "║  Workspace  : %-42s ║\n" "/ros2_ws"
printf "║  ROS Domain : %-42s ║\n" "$(printenv ROS_DOMAIN_ID)"
echo "╠══════════════════════════════════════════════════════════╣"
echo "║  GUI apps — always launch via vglrun (or use aliases):  ║"
echo "║    rv        — vglrun rviz2                             ║"
echo "║    rq        — vglrun rqt                               ║"
echo "║    gz        — vglrun gazebo                            ║"
echo "╠══════════════════════════════════════════════════════════╣"
echo "║  ROS 2 aliases:                                         ║"
echo "║    cb  — colcon build --symlink-install                 ║"
echo "║    cs  — source install/setup.bash                      ║"
echo "║    rl  — ros2 launch                                    ║"
echo "║    rr  — ros2 run     rt — ros2 topic                   ║"
echo "╚══════════════════════════════════════════════════════════╝"
echo ""

# ---------------------------------------------------------------------------
# 5. Execute the command passed to the container.
#    We invoke bash as an interactive shell (-i) so that ~/.bashrc is sourced
#    and all aliases (rv, rq, gz, cb, cs …) are available immediately.
# ---------------------------------------------------------------------------
if [ "$#" -eq 0 ] || [ "$1" = "bash" ]; then
    exec bash --login -i
else
    exec "$@"
fi