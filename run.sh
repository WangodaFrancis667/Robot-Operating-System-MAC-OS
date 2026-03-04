#!/usr/bin/env bash
# =============================================================================
# run.sh — ROS 2 Jazzy Docker Helper (Cross-Platform)
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
#   gui     — Show GUI connection info and quick diagnostic tips
#   help    — Show this help message (default)
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
IMAGE_NAME="ros2-jazzy-dev:latest"

# ---------------------------------------------------------------------------
# Preflight checks
# ---------------------------------------------------------------------------
check_docker() {
    if ! command -v docker &>/dev/null; then
        die "Docker is not installed or not in PATH.\n  Install: https://docs.docker.com/desktop/"
    fi
    if ! docker info &>/dev/null; then
        die "Docker daemon is not running.\n  Start Docker Desktop and try again."
    fi
    success "Docker is running."
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
    echo -e "${BOLD}ROS 2 Jazzy Cross-Platform Docker Helper${RESET}"
    echo ""
    echo -e "  ${CYAN}./run.sh start${RESET}   — Build (if needed) & start container, then open shell"
    echo -e "  ${CYAN}./run.sh shell${RESET}   — Attach an interactive shell to the running container"
    echo -e "  ${CYAN}./run.sh build${RESET}   — Force-rebuild the Docker image from scratch"
    echo -e "  ${CYAN}./run.sh stop${RESET}    — Stop & remove the container (./src data preserved)"
    echo -e "  ${CYAN}./run.sh status${RESET}  — Show current container state"
    echo -e "  ${CYAN}./run.sh logs${RESET}    — Tail container logs (Ctrl-C to exit)"
    echo -e "  ${CYAN}./run.sh gui${RESET}     — Show GUI access info and diagnostic tips"
    echo -e "  ${CYAN}./run.sh help${RESET}    — Show this message"
    echo ""
    echo -e "  ${BOLD}GUI Desktop:${RESET}"
    echo -e "    Browser  →  ${CYAN}http://localhost:6080/vnc.html${RESET}"
    echo -e "    VNC      →  ${CYAN}vnc://localhost:5900${RESET}  (no password)"
    echo ""
}

cmd_build() {
    check_docker
    info "Building Docker image '${IMAGE_NAME}' (this takes a few minutes on first run)..."
    docker compose -f "${COMPOSE_FILE}" build --no-cache
    success "Image built successfully."
}

cmd_start() {
    check_docker
    ensure_src_dir

    if ! docker image inspect "${IMAGE_NAME}" &>/dev/null; then
        info "Image not found — building for the first time (this may take 5–10 min)..."
        docker compose -f "${COMPOSE_FILE}" build
    fi

    info "Starting container '${CONTAINER_NAME}'..."
    docker compose -f "${COMPOSE_FILE}" up -d

    # Give the entrypoint a moment to start Xvfb + VNC + noVNC
    sleep 2

    success "Container is up."
    echo ""
    echo -e "  ${BOLD}GUI Desktop (open in your browser):${RESET}"
    echo -e "    ${CYAN}http://localhost:6080/vnc.html${RESET}"
    echo ""
    echo -e "  ${BOLD}VNC client (optional):${RESET}"
    echo -e "    ${CYAN}vnc://localhost:5900${RESET}  (no password)"
    echo ""
    info "Opening interactive shell — type 'exit' to leave (container keeps running)."
    echo ""

    cmd_shell
}

cmd_shell() {
    if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        die "Container '${CONTAINER_NAME}' is not running.\n  Run './run.sh start' first."
    fi
    # -i flag sources ~/.bashrc so all aliases (rv, rq, gz, cb…) are available
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
    info "GUI is served via VNC from inside the container — no XQuartz or X server needed."
    echo ""
    echo -e "  ${CYAN}Browser (recommended)${RESET}"
    echo -e "    http://localhost:6080/vnc.html"
    echo ""
    echo -e "  ${CYAN}VNC client${RESET}"
    echo -e "    vnc://localhost:5900  (no password)"
    echo -e "    macOS: Finder → Go → Connect to Server → vnc://localhost:5900"
    echo ""
    info "Test GUI from the container shell:"
    echo -e "  ${CYAN}xeyes${RESET}          — Basic X11 test (no OpenGL required)"
    echo -e "  ${CYAN}glxgears${RESET}       — OpenGL test via Mesa llvmpipe"
    echo -e "  ${CYAN}glxinfo | head -20${RESET}  — Check OpenGL renderer and version"
    echo -e "  ${CYAN}rv${RESET}             — RViz2"
    echo -e "  ${CYAN}rq${RESET}             — rqt"
    echo -e "  ${CYAN}gz${RESET}             — Gazebo Sim"
    echo ""
    warn "Troubleshooting:"
    warn "  Blank browser screen → wait 5–10 s after 'start', then reload."
    warn "  Check services inside container:"
    warn "    ps aux | grep Xvfb        — should show Xvfb :99"
    warn "    ps aux | grep x11vnc      — should show x11vnc"
    warn "    ps aux | grep websockify  — should show websockify/noVNC"
    warn "    cat /tmp/x11vnc.log       — x11vnc errors"
    warn "    cat /tmp/novnc.log        — noVNC/websockify errors"
    echo ""
}

# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
COMMAND="${1:-help}"

case "${COMMAND}" in
    start)          cmd_start  ;;
    shell)          cmd_shell  ;;
    build)          cmd_build  ;;
    stop)           cmd_stop   ;;
    status)         cmd_status ;;
    logs)           cmd_logs   ;;
    gui)            cmd_gui    ;;
    help|--help|-h) cmd_help   ;;
    *)
        error "Unknown command: '${COMMAND}'"
        cmd_help
        exit 1
        ;;
esac