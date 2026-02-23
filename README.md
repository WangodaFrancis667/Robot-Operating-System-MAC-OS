# ROS 2 Jazzy on macOS — Docker Dev Environment

> **Run a full ROS 2 Jazzy desktop environment — including GUI tools such as
> RViz2, rqt and Gazebo — on macOS (Intel & Apple Silicon) using Docker and X11
> forwarding via XQuartz.**

---

## Table of Contents

1. [Overview](#overview)
2. [Architecture](#architecture)
3. [Prerequisites](#prerequisites)
4. [Quick Start](#quick-start)
5. [Project Structure](#project-structure)
6. [Detailed Usage](#detailed-usage)
   - [Starting the environment](#starting-the-environment)
   - [Opening a shell](#opening-a-shell)
   - [Running GUI apps](#running-gui-apps)
   - [Building ROS 2 packages](#building-ros-2-packages)
   - [Stopping the environment](#stopping-the-environment)
7. [GUI Forwarding Setup (XQuartz)](#gui-forwarding-setup-xquartz)
8. [Handy Aliases Inside the Container](#handy-aliases-inside-the-container)
9. [Troubleshooting](#troubleshooting)
10. [FAQ](#faq)
11. [Contributing](#contributing)
12. [License](#license)

---

## Overview

macOS does not ship with a native X Window System, and Docker Desktop for Mac
does not expose the host's GPU or display in the same way as Linux. This project
solves both problems by wiring together:

| Component | Role |
|-----------|------|
| **Docker Desktop** | Runs the ROS 2 container in a Linux VM |
| **XQuartz** | Provides an X11 server on macOS that the container can talk to |
| **docker-compose** | Declares the container, volumes, and environment variables |
| **run.sh** | One-stop helper script for every lifecycle operation |
| **entrypoint.sh** | Bootstraps the ROS 2 environment inside the container |

The result is a reproducible, fully-isolated ROS 2 workspace that any hobbyist
or professional robotics developer can clone and have running in minutes — with
**no changes to the host macOS system** beyond Docker Desktop and XQuartz.

---

## Architecture

```
macOS Host
├── XQuartz (X11 server — display :0)
├── Docker Desktop
│   └── Container: ros_jazzy_dev
│       ├── /ros2_ws/src  ←── bind-mount → ./src  (your packages live here)
│       ├── /ros2_ws/build  (named volume — fast rebuilds)
│       ├── /ros2_ws/install
│       └── /ros2_ws/log
└── ./src  (persisted on your Mac)
```

X11 traffic from the container is forwarded to XQuartz on the host using the
`DISPLAY=host.docker.internal:0` environment variable.

---

## Prerequisites

| Requirement | Minimum Version | Notes |
|-------------|-----------------|-------|
| **macOS** | 12 Monterey | Works on Intel & Apple Silicon (M1/M2/M3) |
| **Docker Desktop** | 4.x | [Download](https://www.docker.com/products/docker-desktop/) |
| **XQuartz** | 2.8.x | [Download](https://www.xquartz.org/) — required for GUI |
| **Git** | Any | To clone this repository |
| **Disk space** | ~6 GB free | For the ROS 2 Jazzy image |
| **RAM** | 4 GB+ | 8 GB recommended for Gazebo |

> **Apple Silicon note:** The official OSRF ROS 2 Jazzy image is built for
> `linux/amd64`. Docker Desktop transparently emulates it via Rosetta 2.
> Performance is generally good for development; expect some slowdown in
> compute-heavy simulations.

---

## Quick Start

```bash
# 1. Clone the repository
git clone https://github.com/your-username/ros2-macos-docker.git
cd ros2-macos-docker

# 2. Make the helper script executable
chmod +x run.sh

# 3. Install & configure XQuartz (first time only — see section below)
#    Then log out and back in to ensure the X11 socket is created.

# 4. Build the image and open a shell (first build takes ~5 minutes)
./run.sh start
```

That's it. You're inside a fully-sourced ROS 2 Jazzy shell. Try:

```bash
ros2 run demo_nodes_cpp talker &
ros2 run demo_nodes_py listener
```

---

## Project Structure

```
.
├── Dockerfile          # Builds the ROS 2 Jazzy dev image
├── docker-compose.yml  # Container, volumes & environment configuration
├── entrypoint.sh       # Sources ROS 2 and prints a welcome banner on startup
├── run.sh              # Lifecycle helper (start / stop / build / shell / logs …)
├── src/                # Your ROS 2 packages — bind-mounted into the container
│   └── (add packages here)
└── README.md
```

> **Your work lives in `./src`.**  Everything in `./src` is bind-mounted to
> `/ros2_ws/src` inside the container and is fully persisted on your Mac.
> Deleting or recreating the container will *not* lose your code.

---

## Detailed Usage

### Starting the environment

```bash
./run.sh start
```

This will:
1. Check Docker is running.
2. Start XQuartz and grant display access.
3. Build the Docker image if it doesn't exist yet.
4. Launch the container in the background.
5. Drop you into an interactive shell.

---

### Opening a shell

If the container is already running (e.g. in another terminal) you can open
additional shells at any time:

```bash
./run.sh shell
```

---

### Running GUI apps

Once inside the container, launch any X11 application:

```bash
# Sanity check — a small animated window should appear on your Mac
xeyes

# ROS 2 visualiser
rviz2

# ROS 2 GUI tools suite
rqt

# OpenGL test
glxgears
```

If nothing appears, see [GUI Forwarding Setup](#gui-forwarding-setup-xquartz)
and [Troubleshooting](#troubleshooting).

---

### Building ROS 2 packages

Place your packages inside `./src/` (they appear at `/ros2_ws/src/` inside the
container), then:

```bash
# Navigate to workspace root
cd /ros2_ws

# Install any missing dependencies declared in package.xml files
rosdep install --from-paths src --ignore-src -r -y

# Build (short alias: cb)
colcon build --symlink-install

# Source the newly built overlay (short alias: cs)
source install/setup.bash
```

---

### Stopping the environment

```bash
./run.sh stop
```

This stops and removes the container. Your `./src` directory and all named
volumes (build cache) are preserved.

---

### Other commands

| Command | Description |
|---------|-------------|
| `./run.sh build` | Force-rebuild the image from scratch |
| `./run.sh status` | Show container state |
| `./run.sh logs` | Tail container stdout/stderr |
| `./run.sh gui` | Verify XQuartz / display setup |
| `./run.sh help` | Print usage summary |

---

## GUI Forwarding Setup (XQuartz)

XQuartz must be configured **once** to accept connections from the Docker VM.

### Step-by-step

1. **Install XQuartz** from [xquartz.org](https://www.xquartz.org/) and run
   the installer.

2. **Log out and log back in** (or reboot). XQuartz replaces the default X11
   socket; the change only takes effect after a re-login.

3. **Open XQuartz** (it may already be running in the menu bar).

4. Open **XQuartz → Preferences → Security** and enable:
   - ✅ *Allow connections from network clients*

   ![XQuartz Security Preferences](https://i.imgur.com/placeholder.png)
   *(Screenshot: XQuartz Preferences → Security tab)*

5. **Restart XQuartz** after changing the preference (quit from the menu bar
   icon and reopen).

6. Run `./run.sh gui` from your terminal to verify everything is wired up
   correctly.

> **Why is this needed?** Docker Desktop runs containers inside a Linux VM.
> The X11 socket `/tmp/.X11-unix/X0` is bind-mounted into the container, but
> the VM's network stack treats connections as coming from a remote host.
> Enabling *Allow connections from network clients* tells XQuartz to accept
> them.

---

## Handy Aliases Inside the Container

The following aliases are pre-configured in `~/.bashrc` inside the container:

| Alias | Expands to |
|-------|-----------|
| `cb` | `colcon build --symlink-install` |
| `ct` | `colcon test` |
| `cs` | `source install/setup.bash` |
| `rl` | `ros2 launch` |
| `rr` | `ros2 run` |
| `rt` | `ros2 topic` |
| `rn` | `ros2 node` |

---

## Troubleshooting

### GUI app opens but shows a blank / black window

Software rendering is enabled by default (`LIBGL_ALWAYS_SOFTWARE=1`).
If you see a black window in RViz2 or Gazebo, try:

```bash
# Inside the container
export LIBGL_ALWAYS_INDIRECT=1
rviz2
```

### `cannot connect to X server host.docker.internal:0`

1. Make sure XQuartz is open.
2. Run `./run.sh gui` on the host — it calls `xhost +localhost`.
3. Confirm *Allow connections from network clients* is enabled in XQuartz
   Preferences → Security, then **restart XQuartz**.

### `Authorization required, but no authorization protocol specified`

```bash
# On the host (outside the container)
xhost +localhost
xhost +127.0.0.1
```

### Docker image build fails with `no space left on device`

```bash
docker system prune -af --volumes
```

This removes **all** unused images and volumes. Only run this if you are happy
to lose cached layers.

### `colcon build` fails with missing dependencies

```bash
rosdep install --from-paths src --ignore-src -r -y
```

### The container starts but ROS 2 commands are not found

The entrypoint should have sourced `/opt/ros/jazzy/setup.bash` automatically.
If not, run it manually:

```bash
source /opt/ros/jazzy/setup.bash
```

### Apple Silicon: `exec format error`

The image is `linux/amd64`. Ensure **Rosetta** is enabled in Docker Desktop:
*Settings → General → Use Rosetta for x86/amd64 emulation*.

---

## FAQ

**Q: Can I use this with ROS 1 (Noetic)?**
A: The Dockerfile targets ROS 2 Jazzy. To use ROS 1, change the base image to
`osrf/ros:noetic-desktop-full` and update the setup paths accordingly.

**Q: Will GUI apps work without XQuartz?**
A: No. macOS does not have a native X11 server. XQuartz is the only supported
option for X11-based GUI forwarding on macOS.

**Q: How do I change the ROS Domain ID?**
A: Edit `ROS_DOMAIN_ID` in `docker-compose.yml`, or just run `export
ROS_DOMAIN_ID=<number>` inside the container for a temporary change.

**Q: I updated a package in `./src` — do I need to rebuild the image?**
A: No. `./src` is a bind mount. Your edits are reflected immediately inside the
container. Just run `cb` (colcon build) inside the container.

**Q: Can I run multiple terminals connected to the same container?**
A: Yes — open as many terminals as you like and run `./run.sh shell` in each.

**Q: How do I completely reset everything?**
A: Run `./run.sh stop` then `docker image rm ros2-jazzy-macos:latest`. The next
`./run.sh start` will rebuild from scratch. Your source code in `./src` is
untouched.

---

## Contributing

Contributions, bug reports, and feature requests are welcome!

1. Fork the repository.
2. Create a feature branch: `git checkout -b feature/my-improvement`
3. Commit your changes with a descriptive message.
4. Push and open a Pull Request.

Please keep changes focused and include a brief description of *why* the change
is useful, especially for hobbyist users who may be new to Docker or ROS.

---

## License

This project is released under the [MIT License](LICENSE).

---

*Made with ❤️ for the robotics hobbyist community.*
