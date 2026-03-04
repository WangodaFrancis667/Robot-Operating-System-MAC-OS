<div align="center">

# ROS 2 Jazzy · macOS Docker Dev Environment

**Run a full ROS 2 Jazzy desktop environment — including RViz2, rqt and Gazebo — on macOS (Intel & Apple Silicon) using Docker and X11 forwarding via XQuartz.**

<br/>

[![ROS 2 Jazzy](https://img.shields.io/badge/ROS_2-Jazzy_Jalisco-22314E?style=for-the-badge&logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/)
[![Docker](https://img.shields.io/badge/Docker-Desktop_4.x-2496ED?style=for-the-badge&logo=docker&logoColor=white)](https://www.docker.com/products/docker-desktop/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-Noble_24.04-E95420?style=for-the-badge&logo=ubuntu&logoColor=white)](https://releases.ubuntu.com/noble/)
[![macOS](https://img.shields.io/badge/macOS-12%2B_Intel_%26_Apple_Silicon-000000?style=for-the-badge&logo=apple&logoColor=white)](https://www.apple.com/macos/)
[![License: MIT](https://img.shields.io/badge/License-MIT-brightgreen?style=for-the-badge)](LICENSE)

<br/>

[🚀 Quick Start](#-quick-start) &nbsp;·&nbsp;
[🏗️ Architecture](#%EF%B8%8F-architecture) &nbsp;·&nbsp;
[📋 Prerequisites](#-prerequisites) &nbsp;·&nbsp;
[🖥️ GUI Setup](#%EF%B8%8F-gui-forwarding-setup-xquartz) &nbsp;·&nbsp;
[🔧 Troubleshooting](#-troubleshooting) &nbsp;·&nbsp;
[❓ FAQ](#-faq)

</div>

---

## 📖 Overview

macOS does not ship with a native X Window System, and Docker Desktop for Mac does not expose the host GPU or display the same way Linux does. This project solves both problems with a carefully wired stack:

| Component | Role |
|:---|:---|
| [![Docker](https://img.shields.io/badge/-Docker_Desktop-2496ED?logo=docker&logoColor=white&style=flat-square)](https://www.docker.com/products/docker-desktop/) | Runs the ROS 2 container in a Linux VM |
| [![XQuartz](https://img.shields.io/badge/-XQuartz_2.8.x-333333?logo=apple&logoColor=white&style=flat-square)](https://www.xquartz.org/) | Provides an X11 server on macOS for display forwarding |
| **Xvfb** | Virtual framebuffer inside the container — OpenGL is rendered here |
| [![VirtualGL](https://img.shields.io/badge/-VirtualGL_3.1.1-FFA500?style=flat-square)](https://virtualgl.org/) | Intercepts OpenGL calls, renders offscreen via Mesa llvmpipe, ships pixels to XQuartz |
| [![Mesa](https://img.shields.io/badge/-Mesa_llvmpipe-6A0572?style=flat-square)](https://www.mesa3d.org/) | Pure software OpenGL 4.5 renderer — no GPU required |
| **docker-compose** | Declares the container, volumes, and environment variables |
| **`run.sh`** | One-stop helper script for every lifecycle operation |
| **`entrypoint.sh`** | Bootstraps the ROS 2 environment and starts Xvfb on container launch |

### 💡 Why this stack?

[XQuartz](https://www.xquartz.org/) only provides **OpenGL 2.1** via indirect GLX, but [RViz2](https://docs.ros.org/en/jazzy/p/rviz2/) and OGRE require **OpenGL 3.3+**. The solution is a three-layer rendering pipeline running entirely inside the container:

```
┌─────────────────────────────────────────────────────┐
│  Mesa llvmpipe  ←  OpenGL 4.5 (software rendered)  │
│          ↓  renders offscreen frames into            │
│  Xvfb :99  ←  virtual framebuffer (never shown)    │
│          ↓  VirtualGL reads pixels, sends as X11     │
│  XQuartz :0  ←  your Mac screen (2D pixels only)   │
└─────────────────────────────────────────────────────┘
```

> XQuartz never receives a single OpenGL call — only 2D pixel data. This is why it works despite XQuartz's OpenGL 2.1 limitation.

**Key environment variables:**

| Variable | Value | Purpose |
|:---|:---:|:---|
| `LIBGL_ALWAYS_SOFTWARE` | `1` | Forces Mesa software renderer |
| `GALLIUM_DRIVER` | `llvmpipe` | Selects Mesa's llvmpipe backend |
| `MESA_GL_VERSION_OVERRIDE` | `3.3` | Tells OGRE/RViz2 the renderer meets the GL 3.3 requirement |
| `MESA_GLSL_VERSION_OVERRIDE` | `330` | Matching GLSL version override |
| `VGL_DISPLAY` | `:99` | Points VirtualGL at the Xvfb display |
| `VGL_COMPRESS` | `proxy` | Forces plain X11 transport — bypasses need for `vglclient` on the Mac |

---

## 🏗️ Architecture

```
macOS Host
├── 🖥️  XQuartz  (X11 server — display :0)
└── 🐳  Docker Desktop
    └── Container: ros_jazzy_dev
        ├── Xvfb :99          ← virtual framebuffer (OpenGL rendered here)
        ├── VirtualGL          ← intercepts GL, composites to XQuartz via X11
        ├── /ros2_ws/src  ←── bind-mount → ./src  (your packages live here)
        ├── /ros2_ws/build     (named volume — fast rebuilds)
        ├── /ros2_ws/install   (named volume)
        └── /ros2_ws/log       (named volume)

./src  ← persisted on your Mac — survives container removal
```

---

## 📋 Prerequisites

| Requirement | Minimum Version | Download |
|:---|:---:|:---|
| **macOS** | 12 Monterey | Intel & Apple Silicon (M1/M2/M3/M4) ✓ |
| **Docker Desktop** | 4.x | [![Download](https://img.shields.io/badge/Download-Docker_Desktop-2496ED?style=flat-square&logo=docker&logoColor=white)](https://www.docker.com/products/docker-desktop/) |
| **XQuartz** | 2.8.x | [![Download](https://img.shields.io/badge/Download-XQuartz-333333?style=flat-square&logo=apple&logoColor=white)](https://www.xquartz.org/) |
| **Git** | Any | [![Download](https://img.shields.io/badge/Download-Git-F05032?style=flat-square&logo=git&logoColor=white)](https://git-scm.com/downloads) |
| **Free Disk Space** | ~6 GB | For the ROS 2 Jazzy image |
| **RAM** | 4 GB+ | 8 GB recommended for Gazebo |

> [!NOTE]
> **Apple Silicon (M1/M2/M3/M4):** The OSRF ROS 2 Jazzy image is `linux/amd64`. Docker Desktop emulates it transparently via [Rosetta 2](https://support.apple.com/en-us/HT211861). Performance is good for development; expect some slowdown in compute-heavy simulations.
> Enable it in: *Docker Desktop → Settings → General → Use Rosetta for x86/amd64 emulation*.

---

## 🚀 Quick Start

```bash
# 1. Clone the repository
git clone https://github.com/your-username/ros2-macos-docker.git
cd ros2-macos-docker

# 2. Make scripts executable
chmod +x run.sh

# 3. Configure XQuartz (first-time only — see GUI Setup section below)
#    Log out and back in after installing XQuartz.

# 4. Build the image and open a shell  (first build: ~5–10 min)
./run.sh start
```

Once inside the container, verify the setup:

```bash
# ✅ Basic X11 check — a small animated window should appear on your Mac
xeyes

# ✅ OpenGL check via VirtualGL + llvmpipe
vglrun glxgears

# ✅ Launch RViz2
rv
```

---

## 📁 Project Structure

```
.
├── 🐳  Dockerfile           # Builds the ROS 2 Jazzy dev image
├── 📦  docker-compose.yml   # Container, volumes & environment configuration
├── 🚪  entrypoint.sh        # Starts Xvfb, sources ROS 2, prints welcome banner
├── ⚙️  run.sh               # Lifecycle helper (start / stop / build / shell / logs …)
├── 📂  src/                 # Your ROS 2 packages — bind-mounted into the container
│   └── (add packages here)
└── 📖  README.md
```

> [!TIP]
> **Your work lives in `./src`.** Everything in `./src` is bind-mounted to `/ros2_ws/src` inside the container and fully persisted on your Mac. Deleting or recreating the container will **not** lose your code.

---

## 🔨 Detailed Usage

### Starting the environment

```bash
./run.sh start
```

This will automatically:
1. Verify Docker is running
2. Start XQuartz and grant display access (`xhost +localhost`)
3. Build the Docker image if it doesn't exist yet
4. Launch the container (which starts Xvfb internally via `entrypoint.sh`)
5. Drop you into an interactive shell with all aliases loaded

### Opening additional shells

```bash
# In a new terminal tab/window — attaches to the already-running container
./run.sh shell
```

### Running GUI applications

```bash
# X11 test (no OpenGL — good first check)
xeyes

# OpenGL test via VirtualGL + llvmpipe
vglrun glxgears

# ROS 2 GUI tools — use the aliases or vglrun directly
rv              # RViz2   →  vglrun rviz2
rq              # rqt     →  vglrun rqt
gz              # Gazebo  →  vglrun gz sim
```

> [!WARNING]
> **Always use `vglrun`** (or the `rv`/`rq`/`gz` aliases) for any OpenGL application. Running `rviz2` directly will **crash** because XQuartz cannot provide OpenGL 3.3+ natively.

### Building ROS 2 packages

```bash
cd /ros2_ws

# Install missing dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build workspace
cb              # alias → colcon build --symlink-install

# Source the workspace overlay
cs              # alias → source install/setup.bash
```

### Stopping the environment

```bash
./run.sh stop
```

### All available commands

| Command | Description |
|:---|:---|
| `./run.sh start` | Build image (if needed) & start container, then open shell |
| `./run.sh shell` | Attach an interactive shell to the running container |
| `./run.sh build` | Force-rebuild the image from scratch (no cache) |
| `./run.sh stop` | Stop & remove the container — `./src` data is preserved |
| `./run.sh status` | Show current container state |
| `./run.sh logs` | Tail container stdout/stderr (Ctrl-C to exit) |
| `./run.sh gui` | Verify XQuartz / display forwarding setup |
| `./run.sh help` | Print usage summary |

---

## 🖥️ GUI Forwarding Setup (XQuartz)

[XQuartz](https://www.xquartz.org/) must be configured **once** to accept connections from Docker.

### Step-by-step setup

**Step 1 — Install XQuartz**

[![Download XQuartz](https://img.shields.io/badge/⬇️_Download_XQuartz_2.8.x-333333?style=for-the-badge&logo=apple&logoColor=white)](https://www.xquartz.org/)

**Step 2 — Log out and log back in (required)**

XQuartz replaces the default X11 socket. This change only takes effect after re-login.

**Step 3 — Enable Network Connections**

Open **XQuartz → Preferences → Security** and check:
- ✅ *Allow connections from network clients*

**Step 4 — Fully restart XQuartz**

Quit via the menu bar icon (→ *Quit X11*), then reopen XQuartz.

**Step 5 — Launch the environment**

```bash
./run.sh start
# run.sh automatically calls: xhost +localhost
```

> [!NOTE]
> **Why is the Security setting needed?** Docker Desktop runs containers inside a Linux VM. The container connects to XQuartz via `host.docker.internal:0`, which XQuartz treats as a network connection. The Security setting permits it.

---

## ⌨️ Aliases Inside the Container

All aliases are available immediately in any shell opened via `./run.sh start` or `./run.sh shell`. They are also installed as system-wide commands in `/usr/local/bin/`.

### 🎮 GUI Shortcuts

| Alias | Expands to | Description |
|:---|:---|:---|
| `rv` | `vglrun rviz2` | [RViz2](https://docs.ros.org/en/jazzy/p/rviz2/) — ROS 2 3D visualizer |
| `rq` | `vglrun rqt` | [rqt](https://docs.ros.org/en/jazzy/Concepts/About-RQt.html) — ROS 2 GUI plugin framework |
| `gz` | `vglrun gz sim` | [Gazebo](https://gazebosim.org/) — robotics simulator |

### 🔧 Build & Workspace

| Alias | Expands to | Description |
|:---|:---|:---|
| `cb` | `colcon build --symlink-install` | Build the workspace |
| `ct` | `colcon test` | Run package tests |
| `cs` | `source install/setup.bash` | Source the workspace overlay |

### 📡 ROS 2 Shortcuts

| Alias | Expands to | Description |
|:---|:---|:---|
| `rl` | `ros2 launch` | Launch a ROS 2 launch file |
| `rr` | `ros2 run` | Run a ROS 2 node |
| `rt` | `ros2 topic` | Topic introspection |
| `rn` | `ros2 node` | Node introspection |

---

## 🔧 Troubleshooting

<details>
<summary><strong>❌ `rviz2` crashes with "Unable to create an OpenGL context"</strong></summary>

Never run `rviz2` directly. Always go through VirtualGL:

```bash
rv            # ✅ correct — uses vglrun alias
vglrun rviz2  # ✅ also correct
rviz2         # ❌ wrong — will crash on macOS + Docker
```

</details>

<details>
<summary><strong>❌ `cannot connect to X server host.docker.internal:0`</strong></summary>

1. Make sure XQuartz is open and running.
2. Confirm **Allow connections from network clients** is enabled in *XQuartz → Preferences → Security*, then **fully restart XQuartz**.
3. On the host (outside the container):
   ```bash
   xhost +localhost
   xhost +127.0.0.1
   ```
4. Run `./run.sh gui` — this calls `xhost +localhost` automatically.

</details>

<details>
<summary><strong>❌ `Authorization required, but no authorization protocol specified`</strong></summary>

Run on the host (outside the container):

```bash
xhost +localhost
xhost +127.0.0.1
```

</details>

<details>
<summary><strong>⚠️ `QStandardPaths: XDG_RUNTIME_DIR not set`</strong></summary>

This is a **harmless warning**. Qt falls back to `/tmp/runtime-<user>`. It does not affect functionality and can be safely ignored.

</details>

<details>
<summary><strong>❌ `[VGL] ERROR: Could not connect to VGL client`</strong></summary>

`VGL_COMPRESS` is not set to `proxy`. VirtualGL classifies `host.docker.internal:0` as a remote display and tries to use binary VGL Transport, which requires a `vglclient` daemon on the Mac.

The fix is already in `docker-compose.yml`:
```yaml
- VGL_COMPRESS=proxy
```

If you are running a custom setup, export this in your shell before launching:
```bash
export VGL_COMPRESS=proxy
```

</details>

<details>
<summary><strong>❌ Xvfb failed to start — GUI apps show no display</strong></summary>

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

</details>

<details>
<summary><strong>❌ `colcon build` fails with missing dependencies</strong></summary>

```bash
rosdep install --from-paths src --ignore-src -r -y
```

See the [colcon docs](https://colcon.readthedocs.io/en/released/) and [rosdep docs](https://docs.ros.org/en/independent/api/rosdep/html/) for more options.

</details>

<details>
<summary><strong>❌ Apple Silicon: `exec format error`</strong></summary>

The image is `linux/amd64`. Enable Rosetta emulation in Docker Desktop:

*Settings → General → Use Rosetta for x86/amd64 emulation on Apple Silicon*

See [Apple's Rosetta documentation](https://support.apple.com/en-us/HT211861) for more context.

</details>

<details>
<summary><strong>❌ Docker image build fails with `no space left on device`</strong></summary>

```bash
docker system prune -af --volumes
```

> ⚠️ This removes all unused Docker images, containers, and volumes. Your `./src` code on the host is unaffected.

</details>

---

## ❓ FAQ

<details>
<summary><strong>Can I use this with ROS 1 (Noetic)?</strong></summary>

Yes. Change the base image in `Dockerfile`:
```dockerfile
FROM osrf/ros:noetic-desktop-full
```
Then update the `setup.bash` source paths in `entrypoint.sh` and `docker-compose.yml` accordingly. See the [ROS Noetic docs](https://wiki.ros.org/noetic) for details.

</details>

<details>
<summary><strong>Why does `xeyes` work but `rviz2` (without `vglrun`) doesn't?</strong></summary>

`xeyes` is a plain X11 app — it uses no OpenGL. `rviz2` requires OpenGL 3.3+, which XQuartz cannot provide directly (it caps at 2.1 via indirect GLX). `vglrun` routes all OpenGL calls through Mesa llvmpipe inside the container, bypassing XQuartz's limitation entirely.

</details>

<details>
<summary><strong>How do I change the ROS Domain ID?</strong></summary>

**Persistent:** Edit `ROS_DOMAIN_ID` in `docker-compose.yml`:
```yaml
- ROS_DOMAIN_ID=42
```

**Temporary (inside the container):**
```bash
export ROS_DOMAIN_ID=42
```

See the [ROS 2 Domain ID docs](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Domain-ID.html) for valid ranges and isolation details.

</details>

<details>
<summary><strong>I updated a package in `./src` — do I need to rebuild the image?</strong></summary>

No. `./src` is a bind mount that reflects changes immediately. Just run `cb` inside the container to rebuild your ROS 2 workspace:

```bash
cb    # colcon build --symlink-install
cs    # source install/setup.bash
```

</details>

<details>
<summary><strong>Can I open multiple terminals connected to the same container?</strong></summary>

Yes — run `./run.sh shell` in as many terminal tabs or windows as you like. All sessions share the same container state.

</details>

<details>
<summary><strong>How do I completely reset everything?</strong></summary>

```bash
./run.sh stop
docker image rm ros2-jazzy-macos:latest
./run.sh start   # rebuilds from scratch (~5–10 min)
```

Your source code in `./src` is **untouched**.

</details>

---

## 🤝 Contributing

Contributions, bug reports, and feature requests are welcome!

1. **Fork** the repository
2. **Create a branch:** `git checkout -b feature/my-improvement`
3. **Commit** with a descriptive message
4. **Push** and open a **Pull Request**

Please include a brief description of *why* the change is useful, especially for users who may be new to Docker or ROS 2.

---

## 📚 Further Reading & Resources

| Resource | Link |
|:---|:---|
| ROS 2 Jazzy Documentation | [![ROS 2 Docs](https://img.shields.io/badge/docs.ros.org-Jazzy-22314E?style=flat-square&logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/) |
| ROS 2 Tutorials | [![Tutorials](https://img.shields.io/badge/ROS_2-Tutorials-22314E?style=flat-square&logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/Tutorials.html) |
| Docker Desktop for Mac | [![Docker](https://img.shields.io/badge/Docker-Desktop_Mac-2496ED?style=flat-square&logo=docker&logoColor=white)](https://docs.docker.com/desktop/install/mac-install/) |
| XQuartz | [![XQuartz](https://img.shields.io/badge/XQuartz-xquartz.org-333333?style=flat-square&logo=apple&logoColor=white)](https://www.xquartz.org/) |
| VirtualGL | [![VirtualGL](https://img.shields.io/badge/VirtualGL-virtualgl.org-FFA500?style=flat-square)](https://virtualgl.org/) |
| Mesa 3D Graphics | [![Mesa](https://img.shields.io/badge/Mesa-mesa3d.org-6A0572?style=flat-square)](https://www.mesa3d.org/) |
| colcon Build Tool | [![colcon](https://img.shields.io/badge/colcon-Build_Tool-blue?style=flat-square)](https://colcon.readthedocs.io/en/released/) |
| rosdep | [![rosdep](https://img.shields.io/badge/rosdep-Dependency_Manager-22314E?style=flat-square&logo=ros&logoColor=white)](https://docs.ros.org/en/independent/api/rosdep/html/) |
| Gazebo Robotics Simulator | [![Gazebo](https://img.shields.io/badge/Gazebo-Simulator-orange?style=flat-square)](https://gazebosim.org/home) |
| OSRF ROS Docker Images | [![OSRF](https://img.shields.io/badge/OSRF-Docker_Hub-2496ED?style=flat-square&logo=docker&logoColor=white)](https://hub.docker.com/r/osrf/ros/) |

---

## 📄 License

This project is released under the [MIT License](LICENSE).

---

<div align="center">

Made with ❤️ for the robotics hobbyist community

[![ROS 2](https://img.shields.io/badge/powered_by-ROS_2_Jazzy-22314E?style=flat-square&logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/)
[![Docker](https://img.shields.io/badge/containerized_with-Docker-2496ED?style=flat-square&logo=docker&logoColor=white)](https://www.docker.com/)

</div>