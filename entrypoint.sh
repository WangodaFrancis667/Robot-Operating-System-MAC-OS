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
# All GUI apps render on the local Xvfb :99 virtual framebuffer.
# The screen is served from the container via VNC (5900) and noVNC (6080).
export DISPLAY=:99
export VGL_DISPLAY=:99
export GALLIUM_DRIVER=llvmpipe
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE=3.3   # Tells OGRE the SW renderer supports GL 3.3+
export MESA_GLSL_VERSION_OVERRIDE=330
# OGRE_RTT_MODE=Copy — use off-screen texture blit instead of GLX pbuffer.
export OGRE_RTT_MODE=Copy
# QT_QPA_PLATFORM=xcb — Qt must be told to use the X11 xcb backend explicitly.
# When DISPLAY=:99 Qt sometimes fails to auto-detect the platform and aborts
# with "could not load platform plugin xcb" even though xcb is installed.
export QT_QPA_PLATFORM=xcb

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
    echo "             GUI apps (rviz2, rqt, gz sim) will not work."
    echo "             Check that xvfb is installed: apt list --installed | grep xvfb"
else
    echo "[entrypoint] Xvfb is ready on display :99 (PID $XVFB_PID)."

    # -----------------------------------------------------------------------
    # Start x11vnc — serves Xvfb:99 over VNC on port 5900.
    # Connect from macOS:  open vnc://localhost:5900  (no password)
    # -foreground is NOT used here; we start it in the background and then
    # explicitly wait for TCP port 5900 to open before proceeding.
    # -----------------------------------------------------------------------
    x11vnc -display :99 -forever -nopw -rfbport 5900 \
            -listen 0.0.0.0 -quiet -o /tmp/x11vnc.log &
    X11VNC_PID=$!

    # Wait up to 5 s for x11vnc to open port 5900
    VNC_READY=0
    for i in $(seq 1 50); do
        if ss -tlnp 2>/dev/null | grep -q ':5900' || \
           netstat -tlnp 2>/dev/null | grep -q ':5900'; then
            VNC_READY=1
            break
        fi
        sleep 0.1
    done

    if [ "$VNC_READY" -eq 1 ]; then
        echo "[entrypoint] x11vnc started — VNC on port 5900 (PID $X11VNC_PID)."
    else
        echo "[entrypoint] WARNING: x11vnc may not be ready yet (port 5900 not confirmed)."
    fi

    # -----------------------------------------------------------------------
    # Start noVNC — browser-based VNC UI on port 6080.
    # Open on macOS:  http://localhost:6080/vnc.html
    #
    # We use websockify directly and locate the noVNC web root ourselves.
    # --daemon is avoided because it masks startup errors; we background it
    # manually and redirect output to a log file.
    # -----------------------------------------------------------------------
    NOVNC_WEB=""
    for candidate in /usr/share/novnc /usr/share/noVNC /opt/novnc; do
        if [ -f "${candidate}/vnc.html" ] || [ -f "${candidate}/vnc_lite.html" ]; then
            NOVNC_WEB="$candidate"
            break
        fi
    done

    if [ -n "$NOVNC_WEB" ]; then
        websockify --web="$NOVNC_WEB" 6080 localhost:5900 \
            >/tmp/novnc.log 2>&1 &
        echo "[entrypoint] noVNC started — http://localhost:6080/vnc.html"
        echo "[entrypoint]   noVNC web root: $NOVNC_WEB"
    else
        # Fallback: websockify without a web root still proxies raw VNC;
        # use a VNC client on localhost:5900 instead.
        websockify 6080 localhost:5900 >/tmp/novnc.log 2>&1 &
        echo "[entrypoint] WARNING: noVNC web root not found — raw websocket proxy"
        echo "[entrypoint]   running on port 6080. Use a VNC client on port 5900."
    fi
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
echo "║  View the desktop (GUI apps appear here):               ║"
echo "║    Browser : http://localhost:6080/vnc.html             ║"
echo "║    VNC     : vnc://localhost:5900  (no password)        ║"
echo "╠══════════════════════════════════════════════════════════╣"
echo "║  GUI aliases (run these in the shell below):            ║"
echo "║    rv  — rviz2       rq  — rqt       gz  — gz sim       ║"
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