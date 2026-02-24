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
7. [GUI Forwarding Setup (XQuartz)](#gui-forwarding-setup-xquartz)
8. [Aliases Inside the Container](#aliases-inside-the-container)
9. [Troubleshooting](#troubleshooting)
10. [FAQ](#faq)
11. [Contributing](#contributing)
12. [License](#license)

---

## Overview

macOS does not ship with a native X Window System, and Docker Desktop for Mac
does not expose the host GPU or display the same way Linux does. This project
solves both problems by wiring together:

| Component | Role |
|-----------|------|
| **Docker Desktop** | Runs the ROS 2 container in a Linux VM |
| **XQuartz** | Provides an X11 server on macOS that the container forwards to |
| **Xvfb** | Virtual framebuffer inside the container — OpenGL is rendered here |
| **VirtualGL** | Intercepts OpenGL calls, renders offscreen via Mesa llvmpipe, ships pixels to XQuartz |
| **Mesa llvmpipe** | Pure software OpenGL 4.5 renderer — no GPU required |
| **docker-compose** | Declares the container, volumes, and environment |
| **run.sh** | One-stop helper script for every lifecycle operation |
| **entrypoint.sh** | Bootstraps the ROS 2 environment and starts Xvfb on container launch |

### Why this stack?

XQuartz only provides OpenGL 2.1 via indirect GLX. RViz2 and OGRE require
OpenGL 3.3+. The solution is a three-layer pipeline entirely inside the
container:

```
Mesa llvmpipe (OpenGL 4.5)
        ↓  renders offscreen into
Xvfb :99 (virtual framebuffer, never shown)
        ↓  VirtualGL reads pixels and sends as plain X11 image to
XQuartz :0 (your Mac screen)
```

XQuartz never receives a single OpenGL call — only 2D pixel data. This is why
it works despite XQuartz's OpenGL 2.1 limitation.

**Key environment variables that make this work:**

| Variable | Value | Purpose |
|----------|-------|---------|
| `LIBGL_ALWAYS_SOFTWARE` | `1` | Forces Mesa software renderer |
| `GALLIUM_DRIVER` | `llvmpipe` | Selects Mesa's llvmpipe backend |
| `MESA_GL_VERSION_OVERRIDE` | `3.3` | Tells OGRE/RViz2 the renderer meets the GL 3.3 requirement |
| `VGL_DISPLAY` | `:99` | Points VirtualGL at the Xvfb display |
| `VGL_COMPRESS` | `proxy` | Forces plain X11 transport — critical because `host.docker.internal:0` looks like a remote host to VirtualGL, which would otherwise require a `vglclient` daemon on the Mac |

---

## Architecture

```
macOS Host
├── XQuartz (X11 server — display :0)
├── Docker Desktop
│   └── Container: ros_jazzy_dev
│       ├── Xvfb :99          ← virtual framebuffer (OpenGL rendered here)
│       ├── VirtualGL          ← intercepts GL, composites to XQuartz via X11
│       ├── /ros2_ws/src  ←── bind-mount → ./src  (your packages live here)
│       ├── /ros2_ws/build     (named volume — fast rebuilds)
│       ├── /ros2_ws/install
│       └── /ros2_ws/log
└── ./src  (persisted on your Mac)
```

---

## Prerequisites

| Requirement | Minimum Version | Notes |
|-------------|-----------------|-------|
| **macOS** | 12 Monterey | Works on Intel & Apple Silicon (M1/M2/M3/M4) |
| **Docker Desktop** | 4.x | [Download](https://www.docker.com/products/docker-desktop/) |
| **XQuartz** | 2.8.x | [Download](https://www.xquartz.org/) — required for GUI |
| **Git** | Any | To clone this repository |
| **Disk space** | ~6 GB free | For the ROS 2 Jazzy image |
| **RAM** | 4 GB+ | 8 GB recommended for Gazebo |

> **Apple Silicon note:** The official OSRF ROS 2 Jazzy image is `linux/amd64`.
> Docker Desktop transparently emulates it via Rosetta 2. Performance is good
> for development; expect slowdown in compute-heavy simulations.

---

## Quick Start

```bash
# 1. Clone the repository
git clone https://github.com/your-username/ros2-macos-docker.git
cd ros2-macos-docker

# 2. Make scripts executable
chmod +x run.sh

# 3. Configure XQuartz (first time only — see section below)
#    Then log out and back in to ensure the X11 socket is active.

# 4. Build the image and open a shell (first build ~5–10 minutes)
./run.sh start
```

Inside the container, try:

```bash
# Basic X11 sanity check — a small animated window should appear on your Mac
xeyes

# OpenGL sanity check
vglrun glxgears

# RViz2
rv
```

---

## Project Structure

```
.
├── Dockerfile          # Builds the ROS 2 Jazzy dev image
├── docker-compose.yml  # Container, volumes & environment configuration
├── entrypoint.sh       # Starts Xvfb, sources ROS 2, prints welcome banner
├── run.sh              # Lifecycle helper (start / stop / build / shell / logs …)
├── src/                # Your ROS 2 packages — bind-mounted into the container
│   └── (add packages here)
└── README.md
```

> **Your work lives in `./src`.** Everything in `./src` is bind-mounted to
> `/ros2_ws/src` inside the container and fully persisted on your Mac.
> Deleting or recreating the container will **not** lose your code.

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
4. Launch the container (which starts Xvfb internally via `entrypoint.sh`).
5. Drop you into an interactive shell with all aliases loaded.

### Opening additional shells

```bash
./run.sh shell
```

### Running GUI apps

```bash
# X11 test (no OpenGL — good first check)
xeyes

# OpenGL test via VirtualGL + llvmpipe
vglrun glxgears

# ROS 2 tools — use the aliases or vglrun directly
rv           # rviz2
rq           # rqt
gz           # gazebo

# Equivalent explicit forms
vglrun rviz2
vglrun rqt
vglrun gazebo
```

> **Always use `vglrun`** (or the `rv`/`rq`/`gz` aliases) for any OpenGL
> application. Running `rviz2` directly will fail because XQuartz cannot
> provide OpenGL 3.3+ natively.

### Building ROS 2 packages

```bash
cd /ros2_ws

# Install missing dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build
cb                        # alias for: colcon build --symlink-install

# Source the overlay
cs                        # alias for: source install/setup.bash
```

### Stopping the environment

```bash
./run.sh stop
```

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

XQuartz must be configured **once** to accept connections from Docker.

### Step-by-step

1. **Install XQuartz** from [xquartz.org](https://www.xquartz.org/).

2. **Log out and log back in** (or reboot). This is required — XQuartz
   replaces the default X11 socket and the change only takes effect after
   re-login.

3. **Open XQuartz → Preferences → Security** and enable:
   - ✅ *Allow connections from network clients*

4. **Fully quit XQuartz** (menu bar icon → Quit X11) and reopen it.

5. Run `./run.sh start` — the script automatically calls `xhost +localhost`
   before launching the container.

> **Why is the Security setting needed?** Docker Desktop runs containers inside
> a Linux VM. The container connects to XQuartz via `host.docker.internal:0`,
> which XQuartz treats as a network connection. The Security setting permits it.

---

## Aliases Inside the Container

All aliases are available immediately when you open a shell via `./run.sh start`
or `./run.sh shell`. They are also available as system-wide commands in
`/usr/local/bin/`.

| Alias | Expands to |
|-------|------------|
| `rv` | `vglrun rviz2` |
| `rq` | `vglrun rqt` |
| `gz` | `vglrun gazebo` |
| `cb` | `colcon build --symlink-install` |
| `ct` | `colcon test` |
| `cs` | `source install/setup.bash` |
| `rl` | `ros2 launch` |
| `rr` | `ros2 run` |
| `rt` | `ros2 topic` |
| `rn` | `ros2 node` |

---

## Troubleshooting

### `rviz2` crashes with "Unable to create an OpenGL context"

Never run `rviz2` directly. Always use VirtualGL:

```bash
rv          # correct — uses vglrun
vglrun rviz2  # also correct
rviz2         # wrong — will crash on macOS+Docker
```

### `cannot connect to X server host.docker.internal:0`

1. Make sure XQuartz is open.
2. Confirm *Allow connections from network clients* is enabled in XQuartz
   Preferences → Security, then **fully restart XQuartz**.
3. Run `./run.sh gui` — this calls `xhost +localhost` automatically.

### `Authorization required, but no authorization protocol specified`

Run on the host (outside the container):

```bash
xhost +localhost
xhost +127.0.0.1
```

### `QStandardPaths: XDG_RUNTIME_DIR not set`

This is a harmless warning. Qt falls back to `/tmp/runtime-<user>`. It does
not affect functionality and can be safely ignored.

### `[VGL] ERROR: Could not connect to VGL client`

This means `VGL_COMPRESS` is not set to `proxy`. Because the container connects
to XQuartz via `host.docker.internal:0` (a hostname), VirtualGL classifies the
display as remote and tries to use its binary VGL Transport, which requires a
`vglclient` daemon on the Mac. The fix is already in `docker-compose.yml`:

```yaml
- VGL_COMPRESS=proxy
```

If you are running a custom setup, export this in your shell:

```bash
export VGL_COMPRESS=proxy
```

### Xvfb failed to start — GUI apps show no display

Check inside the container:

```bash
ps aux | grep Xvfb
xdpyinfo -display :99
```

If Xvfb is not running, restart the container:

```bash
./run.sh stop
./run.sh start
```

### `colcon build` fails with missing dependencies

```bash
rosdep install --from-paths src --ignore-src -r -y
```

### Apple Silicon: `exec format error`

The image is `linux/amd64`. Enable Rosetta in Docker Desktop:
*Settings → General → Use Rosetta for x86/amd64 emulation*.

### Docker image build fails with `no space left on device`

```bash
docker system prune -af --volumes
```

---

## FAQ

**Q: Can I use this with ROS 1 (Noetic)?**
Change the base image in `Dockerfile` to `osrf/ros:noetic-desktop-full` and
update the `setup.bash` paths accordingly.

**Q: Why does `xeyes` work but `rviz2` without `vglrun` doesn't?**
`xeyes` is a plain X11 app — no OpenGL. `rviz2` requires OpenGL 3.3+, which
XQuartz cannot provide directly. `vglrun` routes OpenGL through Mesa llvmpipe
inside the container, bypassing XQuartz's GL limitation entirely.

**Q: How do I change the ROS Domain ID?**
Edit `ROS_DOMAIN_ID` in `docker-compose.yml`, or run
`export ROS_DOMAIN_ID=<n>` inside the container for a temporary change.

**Q: I updated a package in `./src` — do I need to rebuild the image?**
No. `./src` is a bind mount. Just run `cb` inside the container.

**Q: Can I open multiple terminals connected to the same container?**
Yes — run `./run.sh shell` in as many terminals as you like.

**Q: How do I completely reset everything?**
```bash
./run.sh stop
docker image rm ros2-jazzy-macos:latest
./run.sh start   # rebuilds from scratch
```
Your source code in `./src` is untouched.

---

## Contributing

Contributions, bug reports, and feature requests are welcome!

1. Fork the repository.
2. Create a feature branch: `git checkout -b feature/my-improvement`
3. Commit with a descriptive message.
4. Push and open a Pull Request.

Please include a brief description of *why* the change is useful, especially
for hobbyist users who may be new to Docker or ROS.

---

## License

This project is released under the [MIT License](LICENSE).

---

*Made with ❤️ for the robotics hobbyist community.*