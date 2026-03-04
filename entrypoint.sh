#!/bin/bash
# =============================================================================
# Container Entrypoint — ROS 2 Jazzy / Cross-Platform Docker Dev Environment
# =============================================================================
# Runs as root to start system services (Xvfb, x11vnc, noVNC), then drops
# to the "ros" user for the interactive shell.
#
# GUI stack (no VirtualGL — pure Mesa llvmpipe):
#   Xvfb :99  →  x11vnc (port 5900)  →  websockify/noVNC (port 6080)
#   Browser: http://localhost:6080/vnc.html
# =============================================================================

set -e

# ---------------------------------------------------------------------------
# 0. Core environment — guaranteed present for every child process.
#    We do NOT use VirtualGL (vglrun) here. Pure Mesa llvmpipe is used for
#    all OpenGL rendering — this avoids the GLX transport segfault that
#    occurs when VGL_DISPLAY == DISPLAY (both :99).
# ---------------------------------------------------------------------------
export DISPLAY=:99
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe

# Mesa performance tuning for software rendering
export MESA_GL_VERSION_OVERRIDE=3.3
export MESA_GLSL_VERSION_OVERRIDE=330
export LP_NUM_THREADS=4
export LIBGL_ALWAYS_INDIRECT=0
export LIBGL_DRI3_DISABLE=1

# OGRE/RViz2 optimizations
export OGRE_RTT_MODE=Copy
export QT_QPA_PLATFORM=xcb

# Gazebo performance optimizations
export GAZEBO_MODEL_DATABASE_URI=""
export GAZEBO_RESOURCE_PATH="/usr/share/gazebo-11/"

# XDG_RUNTIME_DIR — prevents Qt "not set, defaulting to /tmp/runtime-*" warnings
export XDG_RUNTIME_DIR=/tmp/runtime-ros
mkdir -p "$XDG_RUNTIME_DIR"
chmod 700 "$XDG_RUNTIME_DIR"

# ---------------------------------------------------------------------------
# 1. Clean up any stale X lock files from a previous container run.
#    Create /tmp/.X11-unix directory with proper permissions for Xvfb.
# ---------------------------------------------------------------------------
rm -f /tmp/.X99-lock /tmp/.X11-unix/X99 2>/dev/null || true
mkdir -p /tmp/.X11-unix
chmod 1777 /tmp/.X11-unix

# ---------------------------------------------------------------------------
# 2. Start Xvfb — virtual framebuffer on display :99.
#    Mesa llvmpipe renders OpenGL directly here (no VirtualGL required).
#    +extension GLX is needed by Qt/OGRE even in software-only mode.
# ---------------------------------------------------------------------------
echo "[entrypoint] Starting Xvfb on display :99 (optimized for performance)..."

# Reduced resolution (1280x720) and color depth (16-bit) for better performance
Xvfb :99 -screen 0 1280x720x16 \
    +extension GLX \
    +extension RANDR \
    +extension RENDER \
    -ac \
    -nolisten tcp \
    -dpi 96 \
    -fbdir /tmp \
    >/tmp/xvfb.log 2>&1 &
XVFB_PID=$!

# Wait up to 5 s for Xvfb socket to appear
XVFB_READY=0
for i in $(seq 1 50); do
    if [ -S /tmp/.X11-unix/X99 ] && kill -0 $XVFB_PID 2>/dev/null; then
        XVFB_READY=1
        break
    fi
    sleep 0.1
done

if [ "$XVFB_READY" -eq 0 ]; then
    echo "[entrypoint] ERROR: Xvfb failed to start on display :99."
    echo "             Check /tmp/xvfb.log for details."
    if [ -f /tmp/xvfb.log ]; then
        echo "             Last lines from Xvfb:"
        tail -10 /tmp/xvfb.log | sed 's/^/             /'
    fi
    # Don't exit — drop to shell so user can diagnose
else
    echo "[entrypoint] Xvfb ready on :99 (PID $XVFB_PID)."

    # -------------------------------------------------------------------------
    # 3. Start x11vnc — VNC server on port 5900 (no password).
    #    Performance optimizations:
    #    -threads       : multi-threaded encoding
    #    -ncache 10     : client-side caching for better performance
    #    -ncache_cr     : cache reset for reliability
    #    -defer 5       : defer screen updates slightly for batching
    #    -wait 5        : wait between pointer events (reduces load)
    # -------------------------------------------------------------------------
    x11vnc \
        -display :99 \
        -forever \
        -shared \
        -repeat \
        -noxdamage \
        -nopw \
        -rfbport 5900 \
        -listen 0.0.0.0 \
        -threads \
        -ncache 10 \
        -ncache_cr \
        -defer 5 \
        -wait 5 \
        -quiet \
        -o /tmp/x11vnc.log \
        &
    X11VNC_PID=$!

    # Wait up to 5 s for x11vnc to open port 5900
    VNC_READY=0
    for i in $(seq 1 50); do
        if ss -tlnp 2>/dev/null | grep -q ':5900'; then
            VNC_READY=1; break
        fi
        sleep 0.1
    done
    [ "$VNC_READY" -eq 1 ] \
        && echo "[entrypoint] x11vnc started — VNC on port 5900 (PID $X11VNC_PID)." \
        || echo "[entrypoint] WARNING: x11vnc port 5900 not confirmed open yet."

    # -------------------------------------------------------------------------
    # 4. Start noVNC / websockify — browser UI on port 6080.
    #
    #    noVNC web root is installed at /opt/novnc (via Dockerfile).
    #    websockify proxies WebSocket → TCP-VNC (localhost:5900).
    #
    #    We set --heartbeat=30 to keep the WebSocket alive through NAT/proxies.
    # -------------------------------------------------------------------------
    NOVNC_WEB=/opt/novnc

    if [ -f "${NOVNC_WEB}/vnc.html" ]; then
        websockify \
            --web="${NOVNC_WEB}" \
            --heartbeat=30 \
            6080 \
            localhost:5900 \
            >/tmp/novnc.log 2>&1 &
        NOVNC_PID=$!
        echo "[entrypoint] noVNC started — http://localhost:6080/vnc.html (PID $NOVNC_PID)"
    else
        echo "[entrypoint] WARNING: noVNC web root not found at ${NOVNC_WEB}."
        echo "             Falling back to raw websockify proxy on port 6080."
        echo "             Use a VNC client at vnc://localhost:5900 instead."
        websockify 6080 localhost:5900 >/tmp/novnc.log 2>&1 &
    fi
fi

# ---------------------------------------------------------------------------
# 5. Source ROS 2
# ---------------------------------------------------------------------------
source /opt/ros/jazzy/setup.bash

if [ -f "/ros2_ws/install/setup.bash" ]; then
    source /ros2_ws/install/setup.bash
fi

# ---------------------------------------------------------------------------
# 6. Welcome banner
# ---------------------------------------------------------------------------
echo ""
echo "╔══════════════════════════════════════════════════════════╗"
echo "║     ROS 2 Jazzy — Cross-Platform Docker Dev Environment  ║"
echo "╠══════════════════════════════════════════════════════════╣"
printf "║  ROS Distro : %-42s ║\n" "${ROS_DISTRO:-jazzy}"
printf "║  Workspace  : %-42s ║\n" "/ros2_ws"
printf "║  ROS Domain : %-42s ║\n" "${ROS_DOMAIN_ID:-0}"
echo "╠══════════════════════════════════════════════════════════╣"
echo "║  GUI Desktop (open in browser or VNC client):           ║"
echo "║    Browser : http://localhost:6080/vnc.html             ║"
echo "║    VNC     : vnc://localhost:5900  (no password)        ║"
echo "╠══════════════════════════════════════════════════════════╣"
echo "║  GUI commands (run from this shell):                    ║"
echo "║    rv  — rviz2       rq  — rqt       gz  — gz sim       ║"
echo "╠══════════════════════════════════════════════════════════╣"
echo "║  ROS 2 aliases:                                         ║"
echo "║    cb  — colcon build   cs  — source install/setup.bash ║"
echo "║    rl  — ros2 launch    rr  — ros2 run                  ║"
echo "║    rt  — ros2 topic     rn  — ros2 node                 ║"
echo "╚══════════════════════════════════════════════════════════╝"
echo ""
echo "  Renderer: Mesa llvmpipe (software OpenGL — no GPU required)"
echo "  OpenGL  : $(DISPLAY=:99 LIBGL_ALWAYS_SOFTWARE=1 glxinfo 2>/dev/null | grep 'OpenGL version' || echo 'run glxinfo to check')"
echo ""

# ---------------------------------------------------------------------------
# 7. Drop to interactive shell as the "ros" user.
#    su --login -s /bin/bash -w DISPLAY,LIBGL_ALWAYS_SOFTWARE,...
#    sources ~/.bashrc (which re-exports all env vars) so the user's shell
#    always has the correct environment even if they open extra terminals.
# ---------------------------------------------------------------------------
if [ "$#" -eq 0 ] || [ "$1" = "bash" ]; then
    # Pass key env vars into the user shell explicitly
    exec su - ros --shell /bin/bash -w \
        DISPLAY,LIBGL_ALWAYS_SOFTWARE,GALLIUM_DRIVER,\
        MESA_GL_VERSION_OVERRIDE,MESA_GLSL_VERSION_OVERRIDE,\
        OGRE_RTT_MODE,QT_QPA_PLATFORM,XDG_RUNTIME_DIR,\
        ROS_DOMAIN_ID,RCUTILS_COLORIZED_OUTPUT \
        -c 'exec bash --login -i'
else
    exec su - ros --shell /bin/bash -w \
        DISPLAY,LIBGL_ALWAYS_SOFTWARE,GALLIUM_DRIVER,\
        MESA_GL_VERSION_OVERRIDE,MESA_GLSL_VERSION_OVERRIDE,\
        OGRE_RTT_MODE,QT_QPA_PLATFORM,XDG_RUNTIME_DIR,\
        ROS_DOMAIN_ID,RCUTILS_COLORIZED_OUTPUT \
        -c "exec $*"
fi