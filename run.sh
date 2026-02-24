#!/usr/bin/env bash
# =============================================================================
# run.sh — ROS 2 Jazzy macOS Docker Helper
# =============================================================================
# Manages the full lifecycle of your ROS 2 Docker development environment.
#
# Usage:
#   ./run.sh [command]
#
# Commands:
#   start   — Build image (if needed) and launch the container, then open shell
#   shell   — Open an interactive shell in the already-running container
#   build   — Force-rebuild the Docker image from scratch
#   stop    — Stop and remove the container (data in ./src is preserved)
#   status  — Show the current state of the container
#   logs    — Tail the container logs
#   gui     — Quick-check that XQuartz is running and display is accessible
#   help    — Show this help message  (default)
# =============================================================================

set -euo pipefail

# ---------------------------------------------------------------------------
# Colour palette (graceful fallback if terminal has no colour support)
# ---------------------------------------------------------------------------
if [ -t 1 ]; then
    RED='\033[0;31m'
    GREEN='\033[0;32m'
    YELLOW='\033[1;33m'
    CYAN='\033[0;36m'
    BOLD='\033[1m'
    RESET='\033[0m'
else
    RED=''; GREEN=''; YELLOW=''; CYAN=''; BOLD=''; RESET=''
fi

# ---------------------------------------------------------------------------
# Logging helpers
# ---------------------------------------------------------------------------
info()    { echo -e "${CYAN}[INFO]${RESET}  $*"; }
success() { echo -e "${GREEN}[OK]${RESET}    $*"; }
warn()    { echo -e "${YELLOW}[WARN]${RESET}  $*"; }
error()   { echo -e "${RED}[ERROR]${RESET} $*" >&2; }
die()     { error "$*"; exit 1; }

# ---------------------------------------------------------------------------
# Script & project locations
# ---------------------------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMPOSE_FILE="${SCRIPT_DIR}/docker-compose.yml"
CONTAINER_NAME="ros_jazzy_dev"
SERVICE_NAME="ros_dev"
SRC_DIR="${SCRIPT_DIR}/src"

# ---------------------------------------------------------------------------
# Preflight checks
# ---------------------------------------------------------------------------
check_docker() {
    if ! command -v docker &>/dev/null; then
        die "Docker is not installed or not in PATH.\n  Install it from https://docs.docker.com/desktop/install/mac-install/"
    fi
    if ! docker info &>/dev/null; then
        die "Docker daemon is not running.\n  Please start Docker Desktop and try again."
    fi
    success "Docker is running."
}

check_xquartz() {
    info "Checking XQuartz..."

    if ! command -v xquartz &>/dev/null && [ ! -d "/Applications/Utilities/XQuartz.app" ]; then
        warn "XQuartz does not appear to be installed."
        warn "  Install it from: https://www.xquartz.org/"
        warn "  GUI apps (RViz2, rqt, Gazebo) will not work without it."
        return
    fi

    # Open XQuartz if it isn't already running
    if ! pgrep -x Xquartz &>/dev/null && ! pgrep -x quartz-wm &>/dev/null; then
        info "Starting XQuartz..."
        open -a XQuartz
        # Give it a moment to create the X11 socket
        sleep 2
    fi

    if command -v xhost &>/dev/null; then
        xhost +localhost               &>/dev/null || true
        xhost +127.0.0.1              &>/dev/null || true
        xhost +host.docker.internal   &>/dev/null || true
        success "XQuartz: display access granted to localhost."
    else
        warn "xhost not found — XQuartz may not be fully started."
        warn "  Log out and back in, or reboot, after installing XQuartz."
    fi

    if [ -z "${DISPLAY:-}" ]; then
        export DISPLAY=:0
        info "DISPLAY was unset; defaulted to ':0'."
    fi
}

ensure_src_dir() {
    if [ ! -d "${SRC_DIR}" ]; then
        info "Creating ./src directory for your ROS 2 packages..."
        mkdir -p "${SRC_DIR}"
        success "Created ${SRC_DIR}"
    fi
}

# ---------------------------------------------------------------------------
# Command implementations
# ---------------------------------------------------------------------------
cmd_help() {
    echo ""
    echo -e "${BOLD}ROS 2 Jazzy macOS Docker Helper${RESET}"
    echo ""
    echo -e "  ${CYAN}./run.sh start${RESET}   — Build (if needed) & start container, then open shell"
    echo -e "  ${CYAN}./run.sh shell${RESET}   — Attach an interactive shell to the running container"
    echo -e "  ${CYAN}./run.sh build${RESET}   — Force-rebuild the Docker image from scratch"
    echo -e "  ${CYAN}./run.sh stop${RESET}    — Stop & remove the container (./src data preserved)"
    echo -e "  ${CYAN}./run.sh status${RESET}  — Show current container state"
    echo -e "  ${CYAN}./run.sh logs${RESET}    — Tail container logs (Ctrl-C to exit)"
    echo -e "  ${CYAN}./run.sh gui${RESET}     — Verify XQuartz / display forwarding"
    echo -e "  ${CYAN}./run.sh help${RESET}    — Show this message"
    echo ""
}

cmd_build() {
    check_docker
    info "Building Docker image (this may take a few minutes on first run)..."
    docker compose -f "${COMPOSE_FILE}" build --no-cache
    success "Image built successfully."
}

cmd_start() {
    check_docker
    check_xquartz
    ensure_src_dir

    if ! docker image inspect ros2-jazzy-macos:latest &>/dev/null; then
        info "Image not found — building for the first time..."
        docker compose -f "${COMPOSE_FILE}" build
    fi

    info "Starting container '${CONTAINER_NAME}'..."
    docker compose -f "${COMPOSE_FILE}" up -d

    success "Container is up."
    echo ""
    info "Opening interactive shell (type 'exit' to leave without stopping the container)..."
    echo ""

    cmd_shell
}

cmd_shell() {
    if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        die "Container '${CONTAINER_NAME}' is not running.\n  Run './run.sh start' first."
    fi
    # Use 'bash -i' so .bashrc is sourced and all aliases are available
    docker compose -f "${COMPOSE_FILE}" exec "${SERVICE_NAME}" bash -i
}

cmd_stop() {
    check_docker
    info "Stopping container '${CONTAINER_NAME}'..."
    docker compose -f "${COMPOSE_FILE}" down
    success "Container stopped and removed."
}

cmd_status() {
    check_docker
    echo ""
    echo -e "${BOLD}Container status:${RESET}"
    docker compose -f "${COMPOSE_FILE}" ps
    echo ""
}

cmd_logs() {
    check_docker
    info "Tailing logs for '${CONTAINER_NAME}' (Ctrl-C to exit)..."
    docker compose -f "${COMPOSE_FILE}" logs -f
}

cmd_gui() {
    check_xquartz
    echo ""
    info "Run these inside the container to test GUI forwarding:"
    echo ""
    echo -e "  ${CYAN}xeyes${RESET}          — Basic X11 test (no OpenGL needed)"
    echo -e "  ${CYAN}vglrun glxgears${RESET} — OpenGL test via VirtualGL + llvmpipe"
    echo -e "  ${CYAN}rv${RESET}             — RViz2 (alias for: vglrun rviz2)"
    echo -e "  ${CYAN}rq${RESET}             — rqt   (alias for: vglrun rqt)"
    echo ""
    warn "If xeyes works but vglrun glxgears fails:"
    warn "  Check Xvfb is running inside the container: ps aux | grep Xvfb"
    echo ""
    warn "If nothing works — XQuartz setup checklist:"
    warn "  1. XQuartz → Preferences → Security → enable 'Allow connections from network clients'"
    warn "  2. Fully quit XQuartz (menu bar icon → Quit) and reopen it"
    warn "  3. Run: xhost +localhost  (on the host, outside the container)"
    echo ""
}

# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
COMMAND="${1:-help}"

case "${COMMAND}" in
    start)   cmd_start  ;;
    shell)   cmd_shell  ;;
    build)   cmd_build  ;;
    stop)    cmd_stop   ;;
    status)  cmd_status ;;
    logs)    cmd_logs   ;;
    gui)     cmd_gui    ;;
    help|--help|-h) cmd_help ;;
    *)
        error "Unknown command: '${COMMAND}'"
        cmd_help
        exit 1
        ;;
esac