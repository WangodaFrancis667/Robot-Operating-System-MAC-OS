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
echo "║  Handy aliases:"
echo "║    cb  — colcon build --symlink-install"
echo "║    cs  — source install/setup.bash"
echo "║    rl  — ros2 launch"
echo "║    rr  — ros2 run"
echo "║    rt  — ros2 topic"
echo "║    rn  — ros2 node"
echo "╚══════════════════════════════════════════════════════════╝"
echo ""

# ---------------------------------------------------------------------------
# 4. Execute the command passed to the container (default: bash)
# ---------------------------------------------------------------------------
exec "$@"
