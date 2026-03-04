<div align="center">

# ROS 2 Jazzy · Cross-Platform Docker Dev Environment

**Run a full ROS 2 Jazzy desktop — including RViz2, rqt and Gazebo Harmonic — on macOS, Linux, and Windows using Docker and a built-in browser-based VNC desktop. Zero host display configuration required.**

<br/>

[![ROS 2 Jazzy](https://img.shields.io/badge/ROS_2-Jazzy_Jalisco-22314E?style=for-the-badge&logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/)
[![Docker](https://img.shields.io/badge/Docker-Desktop_4.x-2496ED?style=for-the-badge&logo=docker&logoColor=white)](https://www.docker.com/products/docker-desktop/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-Noble_24.04-E95420?style=for-the-badge&logo=ubuntu&logoColor=white)](https://releases.ubuntu.com/noble/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-FF6600?style=for-the-badge)](https://gazebosim.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-brightgreen?style=for-the-badge)](LICENSE)

<br/>

[![macOS](https://img.shields.io/badge/macOS-12%2B_Intel_%26_Apple_Silicon-000000?style=flat-square&logo=apple&logoColor=white)](https://www.apple.com/macos/)
[![Linux](https://img.shields.io/badge/Linux-amd64-FCC624?style=flat-square&logo=linux&logoColor=black)](https://kernel.org/)
[![Windows](https://img.shields.io/badge/Windows-10%2F11_WSL2-0078D4?style=flat-square&logo=windows&logoColor=white)](https://docs.microsoft.com/en-us/windows/wsl/)

<br/>

[🚀 Quick Start](#-quick-start) &nbsp;·&nbsp;
[🏗️ Architecture](#%EF%B8%8F-architecture) &nbsp;·&nbsp;
[📋 Prerequisites](#-prerequisites) &nbsp;·&nbsp;
[🖥️ GUI Desktop](#%EF%B8%8F-gui-desktop) &nbsp;·&nbsp;
[⌨️ Aliases](#%EF%B8%8F-aliases-inside-the-container) &nbsp;·&nbsp;
[🔧 Troubleshooting](#-troubleshooting) &nbsp;·&nbsp;
[❓ FAQ](#-faq)

</div>

---

## 📖 Overview

This project provides a fully self-contained ROS 2 Jazzy development environment that works identically on **macOS**, **Linux**, and **Windows** — no host display server, no XQuartz, no VcXsrv, no per-platform configuration needed.

The entire GUI stack runs **inside** the Docker container. You access the desktop through your browser or any VNC client.

| Component | Role |
|:---|:---|
| **Xvfb `:99`** | Virtual framebuffer — OpenGL is rendered entirely here |
| **Mesa llvmpipe** | Pure software OpenGL 4.5 renderer — no GPU required |
| **x11vnc** | Streams the Xvfb framebuffer over VNC (port `5900`) |
| **noVNC + websockify** | Serves the desktop in any browser (port `6080`) |
| **`docker-compose.yml`** | Declares the container, volumes, and environment |
| **`run.sh`** | One-stop lifecycle helper for every OS |
| **`entrypoint.sh`** | Boots Xvfb, x11vnc, noVNC, then drops to the ROS shell |

### 💡 How it works

```
┌─────────────────────────────────────────────────────────────────┐
│  Container (Ubuntu Noble 24.04)                                 │
│                                                                 │
│  RViz2 / rqt / Gazebo                                          │
│       ↓  OpenGL 3.3 (Mesa llvmpipe — software, no GPU)         │
│  Xvfb :99  ←  virtual framebuffer (1280×720)                   │
│       ↓  pixels streamed via                                    │
│  x11vnc  →  port 5900  (raw VNC)                               │
│       ↓  proxied via                                           │
│  websockify / noVNC  →  port 6080  (WebSocket + browser UI)    │
└─────────────────────────────────────────────────────────────────┘
       ↓                              ↓
 VNC client (any OS)         Browser: http://localhost:6080/vnc.html
 vnc://localhost:5900
```

**Key environment variables set automatically:**

| Variable | Value | Purpose |
|:---|:---:|:---|
| `LIBGL_ALWAYS_SOFTWARE` | `1` | Forces Mesa software renderer |
| `GALLIUM_DRIVER` | `llvmpipe` | Selects Mesa's high-performance llvmpipe backend |
| `LP_NUM_THREADS` | `4` | Multi-threaded Mesa rendering |
| `MESA_GL_VERSION_OVERRIDE` | `3.3` | Advertises GL 3.3 to OGRE/RViz2 |
| `MESA_GLSL_VERSION_OVERRIDE` | `330` | Matching GLSL version |
| `OGRE_RTT_MODE` | `Copy` | Avoids GLX pbuffer issues in OGRE |
| `QT_QPA_PLATFORM` | `xcb` | Qt uses the Xvfb X11 display |

---

## 🏗️ Architecture

```
Your Machine (macOS / Linux / Windows)
└── 🐳  Docker Desktop
    └── Container: ros_jazzy_dev
        ├── Xvfb :99            ← virtual display (Mesa renders here)
        ├── x11vnc :5900        ← VNC server
        ├── noVNC/websockify :6080  ← browser VNC client
        ├── /ros2_ws/src   ←── bind-mount → ./src  (your packages)
        ├── /ros2_ws/build      (named volume — fast rebuilds)
        ├── /ros2_ws/install    (named volume)
        └── /ros2_ws/log        (named volume)

Ports exposed to host:
  5900  →  VNC  (any VNC client, no password)
  6080  →  noVNC  (http://localhost:6080/vnc.html)

./src  ← persisted on your machine — survives container removal
```

---

## 📋 Prerequisites

### All Platforms

| Requirement | Minimum Version | Notes |
|:---|:---:|:---|
| **Docker Desktop** | 4.x | See platform-specific install below |
| **Git** | Any | For cloning this repo |
| **Free Disk Space** | ~8 GB | ROS 2 Jazzy + Gazebo Harmonic image |
| **RAM** | 4 GB+ | 8 GB+ recommended for Gazebo simulation |
| **CPU** | 2 cores+ | 4+ cores recommended |

---

### 🍎 macOS

<details open>
<summary><strong>Installation instructions</strong></summary>

**1. Install Docker Desktop for Mac**

[![Download Docker Desktop](https://img.shields.io/badge/⬇️_Download_Docker_Desktop-2496ED?style=for-the-badge&logo=docker&logoColor=white)](https://www.docker.com/products/docker-desktop/)

- Supports Intel and Apple Silicon (M1/M2/M3/M4)
- After install, open Docker Desktop and wait for the engine to start

**2. Apple Silicon (M1/M2/M3/M4) — Enable Rosetta**

The ROS 2 Jazzy image is `linux/amd64`. Docker Desktop emulates it transparently:

*Docker Desktop → Settings → General → ✅ Use Rosetta for x86/amd64 emulation on Apple Silicon*

> Performance is excellent for development. Compute-heavy simulations may be slower than on native amd64 hardware.

**3. Allocate Sufficient Resources**

*Docker Desktop → Settings → Resources*
- Memory: **8 GB** (4 GB minimum; Gazebo needs more)
- CPUs: **4+** recommended

**4. No XQuartz needed**

This project uses in-container VNC — no XQuartz, no X11 forwarding, no host display configuration required.

</details>

---

### 🐧 Linux

> [!TIP]
> See the [**Full Linux Setup Guide →**](docs/linux.md) for a complete step-by-step walkthrough including distro-specific install commands, headless server usage, and troubleshooting.

<details open>
<summary><strong>Installation instructions</strong></summary>

**1. Install Docker Engine**

```bash
# Remove old versions
sudo apt-get remove docker docker-engine docker.io containerd runc

# Install via the official convenience script
curl -fsSL https://get.docker.com | sudo sh

# Add your user to the docker group (avoids using sudo)
sudo usermod -aG docker $USER
newgrp docker
```

Or follow the [official Docker Engine install guide](https://docs.docker.com/engine/install/).

**2. Install Docker Compose plugin**

```bash
sudo apt-get install -y docker-compose-plugin
```

Verify: `docker compose version`

**3. No display server setup needed**

The container runs its own Xvfb display. Your host display is not used at all — this works on headless servers too.

</details>

---

### 🪟 Windows

> [!TIP]
> See the [**Full Windows Setup Guide →**](docs/windows.md) for a complete step-by-step walkthrough including WSL 2 setup, Docker Desktop configuration, resource tuning, and troubleshooting.

<details open>
<summary><strong>Installation instructions</strong></summary>

**1. Enable WSL 2**

Open **PowerShell as Administrator** and run:

```powershell
# Install WSL 2 and Ubuntu
wsl --install

# Verify WSL version
wsl --set-default-version 2
```

Restart your computer when prompted.

**2. Install Docker Desktop for Windows**

[![Download Docker Desktop](https://img.shields.io/badge/⬇️_Download_Docker_Desktop-2496ED?style=for-the-badge&logo=docker&logoColor=white)](https://www.docker.com/products/docker-desktop/)

During installation, ensure **Use WSL 2 based engine** is selected.

After installation, open Docker Desktop and confirm the engine starts.

**3. Configure WSL 2 Integration**

*Docker Desktop → Settings → Resources → WSL Integration → ✅ Enable integration with your WSL distro*

**4. Allocate Resources**

Create or edit `%USERPROFILE%\.wslconfig`:

```ini
[wsl2]
memory=8GB
processors=4
```

Restart WSL: `wsl --shutdown`, then reopen your terminal.

**5. Run commands from WSL 2 terminal**

Open **Ubuntu** (or your WSL distro) from the Start Menu. All commands in this README should be run from the WSL terminal, not PowerShell or CMD.

> No VcXsrv, no X410, no Xming needed. VNC runs inside the container.

</details>

---

## 🚀 Quick Start

> **All three platforms follow the same steps once Docker is installed.**

```bash
# 1. Clone the repository
git clone https://github.com/your-username/ros2-jazzy-docker.git
cd ros2-jazzy-docker

# 2. Make the helper script executable
chmod +x run.sh

# 3. Build and launch  (first build: ~5–10 min — downloads ~4 GB)
./run.sh start
```

Once the container is running, your shell is inside the ROS 2 environment:

```bash
# ✅ Verify X11 is working (a window appears in the VNC desktop)
xeyes

# ✅ Verify OpenGL software rendering
glxgears

# ✅ Check OpenGL version info
glxinfo | grep "OpenGL version"

# ✅ Launch RViz2
rv

# ✅ Launch rqt
rq

# ✅ Launch Gazebo Sim
gz
```

**Open the GUI desktop in your browser:**
```
http://localhost:6080/vnc.html
```

---

## 📁 Project Structure

```
.
├── 🐳  Dockerfile           # ROS 2 Jazzy image with GUI stack
├── 📦  docker-compose.yml   # Container, ports, volumes & environment
├── 🚪  entrypoint.sh        # Boots Xvfb → x11vnc → noVNC, then ROS shell
├── ⚙️  run.sh               # Lifecycle helper (start / stop / build / shell…)
├── 📂  src/                 # Your ROS 2 packages (bind-mounted)
│   └── (add your packages here)
└── 📖  README.md
```

> [!TIP]
> **Your work lives in `./src`.**  It is bind-mounted to `/ros2_ws/src` inside the container and fully persisted on your host machine. Deleting or rebuilding the container will **never** affect your source code.

---

## 🖥️ GUI Desktop

No host display setup is required on any platform. The GUI desktop is served from inside the container over VNC.

### Accessing the desktop

| Method | URL / Address | Notes |
|:---|:---|:---|
| **Browser (recommended)** | `http://localhost:6080/vnc.html` | Works on all OS, no install needed |
| **VNC client** | `vnc://localhost:5900` | No password |
| **macOS built-in** | Finder → Go → Connect to Server → `vnc://localhost:5900` | Native screen sharing |

> [!NOTE]
> The desktop starts as a **black screen** — this is normal. Xvfb provides a bare framebuffer with no window manager. GUI windows appear when you launch applications from the container shell.

### Launching GUI applications

From inside the container shell (opened via `./run.sh start` or `./run.sh shell`):

```bash
# X11 sanity check
xeyes

# OpenGL check — should show spinning gears
glxgears

# ROS 2 GUI tools
rv          # RViz2   — 3D robot visualiser
rq          # rqt     — ROS 2 plugin GUI framework
gz          # Gazebo  — full robotics simulator
```

### Ports

| Port | Service | Access |
|:---:|:---|:---|
| `6080` | noVNC browser client | `http://localhost:6080/vnc.html` |
| `5900` | Raw VNC | `vnc://localhost:5900` |

---

## 🔨 Usage

### Starting the environment

```bash
./run.sh start
```

This automatically:
1. Verifies Docker is running
2. Builds the image if it does not exist yet
3. Starts the container (which boots Xvfb, x11vnc, noVNC via `entrypoint.sh`)
4. Waits for services to start, then prints the access URLs
5. Drops you into an interactive ROS 2 shell with all aliases loaded

### Opening additional shells

```bash
# In any new terminal — attaches to the already-running container
./run.sh shell
```

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

### Force-rebuilding the image

```bash
./run.sh build
```

### All available commands

| Command | Description |
|:---|:---|
| `./run.sh start` | Build image (if needed), start container, open shell |
| `./run.sh shell` | Attach an interactive shell to the running container |
| `./run.sh build` | Force-rebuild the image from scratch (no cache) |
| `./run.sh stop` | Stop & remove the container — `./src` data is preserved |
| `./run.sh status` | Show current container state |
| `./run.sh logs` | Tail container stdout/stderr (Ctrl-C to exit) |
| `./run.sh gui` | Print GUI access info and diagnostic tips |
| `./run.sh help` | Print usage summary |

---

## ⌨️ Aliases Inside the Container

All aliases are sourced automatically from `~/.bashrc` in every shell session.

### 🎮 GUI Shortcuts

| Alias | Expands to | Description |
|:---|:---|:---|
| `rv` | `rviz2` (with Mesa env) | [RViz2](https://docs.ros.org/en/jazzy/p/rviz2/) — ROS 2 3D visualiser |
| `rq` | `rqt` (with Mesa env) | [rqt](https://docs.ros.org/en/jazzy/Concepts/About-RQt.html) — ROS 2 GUI plugin framework |
| `gz sim` | `gz sim` (with Mesa env) | [Gazebo Harmonic](https://gazebosim.org/) — robotics simulator |

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
<summary><strong>❌ Black screen in noVNC browser</strong></summary>

This is expected on first connect. The Xvfb framebuffer is empty until you launch an application.

From the container shell, run any GUI app:
```bash
xeyes    # lightweight test
rv       # RViz2
```

If it stays black even after launching an app, check that Xvfb and x11vnc are running:
```bash
ps aux | grep Xvfb
ps aux | grep x11vnc
cat /tmp/x11vnc.log
cat /tmp/xvfb.log
```

</details>

<details>
<summary><strong>❌ "Failed to connect to server" in noVNC</strong></summary>

noVNC starts before x11vnc finishes binding. Wait 5–10 seconds after `./run.sh start`, then refresh the browser page.

If the issue persists:
```bash
./run.sh logs
```

Look for `x11vnc` startup errors. Common causes:
- x11vnc crashed due to an unsupported flag — check `/tmp/x11vnc.log` in the container
- Port 5900 or 6080 already in use on the host — change the port mapping in `docker-compose.yml`

</details>

<details>
<summary><strong>❌ Container exits immediately after starting</strong></summary>

```bash
./run.sh stop
docker logs ros_jazzy_dev
```

Common causes:
- Xvfb socket permission issue → resolved by ensuring the entrypoint runs as root
- A previous container left a stale X lock: the entrypoint cleans `/tmp/.X99-lock` automatically

If the problem persists, rebuild:
```bash
./run.sh build
./run.sh start
```

</details>

<details>
<summary><strong>❌ RViz2 / Gazebo crash or show OpenGL errors</strong></summary>

Verify software rendering is active:
```bash
glxinfo | grep "OpenGL renderer"
# Expected: Mesa llvmpipe (LLVM 17.0.6, 256 bits)
```

If it shows a GPU renderer instead, the Mesa env vars are not being picked up. Use the `rv`/`gz` aliases which explicitly set all required variables.

</details>

<details>
<summary><strong>❌ Gazebo is very slow</strong></summary>

Gazebo running on software rendering is CPU-bound. Recommended steps:

1. **Reduce physics update rate** in your world file: `<max_step_size>0.01</max_step_size>` and `<real_time_update_rate>500</real_time_update_rate>`

2. **Use a simpler world** — avoid worlds with many meshes or lights

3. **Allocate more CPU** to Docker:
   - macOS/Windows: *Docker Desktop → Settings → Resources → CPUs → increase*
   - Linux: no limit by default

4. **Disable shadows and reflections** in Gazebo's render settings

</details>

<details>
<summary><strong>❌ `colcon build` fails with missing dependencies</strong></summary>

```bash
cd /ros2_ws
rosdep install --from-paths src --ignore-src -r -y
cb
```

</details>

<details>
<summary><strong>❌ Apple Silicon — `exec format error` or very slow performance</strong></summary>

Enable Rosetta emulation in Docker Desktop:

*Settings → General → ✅ Use Rosetta for x86/amd64 emulation on Apple Silicon*

Restart Docker Desktop after changing this setting.

</details>

<details>
<summary><strong>❌ Windows — container can't reach the internet (DNS failure)</strong></summary>

The `docker-compose.yml` already sets explicit DNS (`8.8.8.8`, `8.8.4.4`). If you still have issues:

```bash
# Test DNS inside the container
docker exec ros_jazzy_dev ping -c 2 8.8.8.8
```

If that works but hostnames don't resolve, check your WSL 2 network settings or corporate firewall/VPN configuration.

</details>

<details>
<summary><strong>❌ Port 5900 or 6080 already in use</strong></summary>

Edit `docker-compose.yml` and remap the ports:
```yaml
ports:
  - "5901:5900"   # host:container
  - "6081:6080"
```

Then access noVNC at `http://localhost:6081/vnc.html`.

</details>

<details>
<summary><strong>❌ Docker image build fails with `no space left on device`</strong></summary>

```bash
docker system prune -af --volumes
```

> ⚠️ This removes all unused Docker images, containers, and volumes. Your `./src` code on the host is completely unaffected.

</details>

---

## ❓ FAQ

<details>
<summary><strong>Do I need XQuartz on macOS?</strong></summary>

**No.** XQuartz is not used at all. The GUI runs inside the container via Xvfb and is served to your browser via noVNC. This is a key design goal — zero host display configuration on every platform.

</details>

<details>
<summary><strong>Does this work on a headless Linux server (no monitor)?</strong></summary>

**Yes.** Xvfb is a virtual framebuffer — it does not require a physical display. You can run this setup on a headless cloud VM or CI server and access the GUI via the noVNC browser client just like on a desktop machine.

</details>

<details>
<summary><strong>Can I use a physical GPU for rendering?</strong></summary>

This setup is intentionally GPU-free for maximum cross-platform compatibility. If you are on Linux and want to pass through a GPU, you would need to replace the Mesa llvmpipe stack with a real OpenGL driver and remove `LIBGL_ALWAYS_SOFTWARE=1`. This is not supported in the current configuration.

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

**No.** `./src` is a bind mount — changes on the host are reflected inside the container immediately. Just rebuild your ROS 2 workspace:

```bash
cb    # colcon build --symlink-install
cs    # source install/setup.bash
```

</details>

<details>
<summary><strong>Can I open multiple shells connected to the same container?</strong></summary>

**Yes.** Run `./run.sh shell` in as many terminal tabs or windows as you like. All sessions share the same running container and filesystem state.

</details>

<details>
<summary><strong>How do I completely reset everything?</strong></summary>

```bash
./run.sh stop
docker image rm ros2-jazzy-dev:latest
docker volume rm ros_ros2_build_cache ros_ros2_install_cache ros_ros2_log_cache
./run.sh start   # rebuilds from scratch (~5–10 min)
```

Your source code in `./src` is **untouched**.

</details>

<details>
<summary><strong>Can I use this with ROS 1 (Noetic)?</strong></summary>

Yes. Change the base image in `Dockerfile`:
```dockerfile
FROM osrf/ros:noetic-desktop-full
```

Then update the `setup.bash` paths in `entrypoint.sh` accordingly. See the [ROS Noetic install docs](https://wiki.ros.org/noetic/Installation/Ubuntu) for details.

</details>

---

## 📚 Resources

| Resource | Link |
|:---|:---|
| ROS 2 Jazzy Documentation | [![ROS 2 Docs](https://img.shields.io/badge/docs.ros.org-Jazzy-22314E?style=flat-square&logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/) |
| ROS 2 Tutorials | [![Tutorials](https://img.shields.io/badge/ROS_2-Tutorials-22314E?style=flat-square&logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/Tutorials.html) |
| Gazebo Harmonic | [![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-FF6600?style=flat-square)](https://gazebosim.org/docs/harmonic/) |
| Docker Desktop | [![Docker](https://img.shields.io/badge/Docker-Desktop-2496ED?style=flat-square&logo=docker&logoColor=white)](https://docs.docker.com/desktop/) |
| noVNC | [![noVNC](https://img.shields.io/badge/noVNC-Web_VNC_Client-4A90D9?style=flat-square)](https://novnc.com/) |
| Mesa 3D (llvmpipe) | [![Mesa](https://img.shields.io/badge/Mesa-llvmpipe-6A0572?style=flat-square)](https://www.mesa3d.org/) |
| colcon Build Tool | [![colcon](https://img.shields.io/badge/colcon-Build_Tool-blue?style=flat-square)](https://colcon.readthedocs.io/en/released/) |
| rosdep | [![rosdep](https://img.shields.io/badge/rosdep-Dependency_Manager-22314E?style=flat-square&logo=ros&logoColor=white)](https://docs.ros.org/en/independent/api/rosdep/html/) |
| OSRF Docker Images | [![OSRF](https://img.shields.io/badge/OSRF-Docker_Hub-2496ED?style=flat-square&logo=docker&logoColor=white)](https://hub.docker.com/r/osrf/ros/) |

---

## 🤝 Contributing

Contributions, bug reports, and feature requests are welcome!

1. **Fork** the repository
2. **Create a branch:** `git checkout -b feature/my-improvement`
3. **Commit** with a descriptive message
4. **Push** and open a **Pull Request**

Please include a brief description of *why* the change is useful, especially for users who may be new to Docker or ROS 2.

---

## 📄 License

This project is released under the [MIT License](LICENSE).

---

<div align="center">

Made with ❤️ for the robotics community

[![ROS 2](https://img.shields.io/badge/powered_by-ROS_2_Jazzy-22314E?style=flat-square&logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/)
[![Docker](https://img.shields.io/badge/containerised_with-Docker-2496ED?style=flat-square&logo=docker&logoColor=white)](https://www.docker.com/)
[![Works on](https://img.shields.io/badge/works_on-macOS_%7C_Linux_%7C_Windows-lightgrey?style=flat-square)](.)

</div>
