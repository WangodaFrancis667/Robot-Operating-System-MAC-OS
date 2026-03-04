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
    # XQuartz is no longer required — GUI is served via VNC/noVNC from inside
    # the container (http://localhost:6080/vnc.html or vnc://localhost:5900).
    # This function is kept as an optional helper for users who still want
    # direct X11 forwarding alongside VNC.
    :
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
    echo -e "  ${BOLD}GUI Desktop (VNC):${RESET}"
    echo -e "    Browser  →  ${CYAN}http://localhost:6080/vnc.html${RESET}"
    echo -e "    VNC client →  ${CYAN}vnc://localhost:5900${RESET}  (no password)"
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
    echo ""
    info "GUI is served via VNC from inside the container (no XQuartz needed):"
    echo ""
    echo -e "  ${CYAN}Browser${RESET}    →  http://localhost:6080/vnc.html   (noVNC — no client needed)"
    echo -e "  ${CYAN}VNC client${RESET} →  vnc://localhost:5900             (no password)"
    echo -e "  ${CYAN}macOS Screen Sharing${RESET}:  Finder → Go → Connect to Server → vnc://localhost:5900"
    echo ""
    info "To test GUI rendering from inside the container shell:"
    echo -e "  ${CYAN}xeyes${RESET}            — Basic X11 test (no OpenGL)"
    echo -e "  ${CYAN}vglrun glxgears${RESET}  — OpenGL via VirtualGL + Mesa llvmpipe"
    echo -e "  ${CYAN}rv${RESET}               — RViz2"
    echo -e "  ${CYAN}rq${RESET}               — rqt"
    echo -e "  ${CYAN}gz${RESET}               — Gazebo Sim"
    echo ""
    warn "If the VNC desktop is blank:"
    warn "  Check Xvfb is running:  ps aux | grep Xvfb"
    warn "  Check x11vnc is running: ps aux | grep x11vnc"
    warn "  Check noVNC is running: ps aux | grep websockify"
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