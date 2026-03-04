<div align="center">

# ROS 2 Jazzy on Linux · Step-by-Step Setup Guide

**Run a full ROS 2 Jazzy desktop — including RViz2, rqt and Gazebo — on any Linux host using Docker and the browser-based VNC desktop. Works on Ubuntu, Debian, Fedora, Arch and headless servers. No host-side ROS installation required.**

<br/>

[![Linux](https://img.shields.io/badge/Linux-Ubuntu%20%7C%20Debian%20%7C%20Fedora%20%7C%20Arch-FCC624?style=for-the-badge&logo=linux&logoColor=black)](https://www.kernel.org/)
[![Docker](https://img.shields.io/badge/Docker-Engine-2496ED?style=for-the-badge&logo=docker&logoColor=white)](https://docs.docker.com/engine/install/)
[![ROS 2 Jazzy](https://img.shields.io/badge/ROS_2-Jazzy-22314E?style=for-the-badge&logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/)

<br/>

[Step 1 — Docker](#-step-1--install-docker-engine) &nbsp;·&nbsp;
[Step 2 — Clone & Run](#-step-2--clone-and-run) &nbsp;·&nbsp;
[Step 3 — GUI Desktop](#-step-3--open-the-gui-desktop) &nbsp;·&nbsp;
[Headless Servers](#-headless-server-usage) &nbsp;·&nbsp;
[Troubleshooting](#-troubleshooting)

</div>

---

## 📋 Requirements

| Requirement | Version | Notes |
|:---|:---:|:---|
| **Linux kernel** | 4.15+ | Most distros since 2018 qualify |
| **Docker Engine** | 24.x+ | Docker Desktop also works but Engine is lighter |
| **Docker Compose** | v2 (plugin) | `docker compose` (not `docker-compose`) |
| **Architecture** | `amd64` (x86-64) | ARM not currently supported |
| **RAM** | 8 GB+ | 4 GB minimum; Gazebo benefits from more |
| **Disk Space** | ~10 GB free | For Docker image + build cache |

> [!NOTE]
> **No GPU required.** The container uses Mesa llvmpipe for pure software OpenGL rendering. All GUI apps run through an internal Xvfb virtual display and are accessible via browser-based VNC — no display server or GPU driver on the host is needed.

---

## 🐳 Step 1 — Install Docker Engine

> [!TIP]
> Already have Docker? Skip to [Step 2](#-step-2--clone-and-run). Verify with `docker --version` and `docker compose version`.

<details>
<summary><strong>Ubuntu / Debian</strong></summary>

The recommended method uses the official Docker apt repository. Do **not** use the snap version — it has known socket permission issues with Docker Compose.

```bash
# Remove any old or snap-installed versions
sudo apt remove -y docker docker-engine docker.io containerd runc docker-desktop 2>/dev/null || true
sudo snap remove docker 2>/dev/null || true

# Install prerequisites
sudo apt update
sudo apt install -y ca-certificates curl gnupg lsb-release

# Add Docker's official GPG key
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | \
  sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg

# Add the Docker apt repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] \
  https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list

# Install Docker Engine + Compose plugin
sudo apt update
sudo apt install -y \
  docker-ce \
  docker-ce-cli \
  containerd.io \
  docker-buildx-plugin \
  docker-compose-plugin
```

</details>

<details>
<summary><strong>Fedora / RHEL / CentOS</strong></summary>

```bash
# Add Docker repo
sudo dnf -y install dnf-plugins-core
sudo dnf config-manager --add-repo https://download.docker.com/linux/fedora/docker-ce.repo

# Install Docker Engine + Compose
sudo dnf install -y \
  docker-ce \
  docker-ce-cli \
  containerd.io \
  docker-buildx-plugin \
  docker-compose-plugin

# Enable and start Docker
sudo systemctl enable --now docker
```

</details>

<details>
<summary><strong>Arch Linux / Manjaro</strong></summary>

```bash
# Install from the official repos
sudo pacman -S docker docker-compose

# Enable and start Docker
sudo systemctl enable --now docker
```

</details>

<details>
<summary><strong>Any Linux — convenience script (quick install)</strong></summary>

```bash
curl -fsSL https://get.docker.com | sh
```

This script automatically detects your distro and installs Docker Engine. Suitable for CI and development machines; not recommended for production.

</details>

---

### Post-install: run Docker without sudo

```bash
# Add your user to the docker group
sudo usermod -aG docker $USER

# Apply group membership (or log out and back in)
newgrp docker

# Verify
docker run hello-world
```

Expected output ends with: `Hello from Docker!`

---

### Enable and start the Docker service

```bash
sudo systemctl enable --now docker
```

Verify everything is running:

```bash
docker --version
docker compose version
```

---

## 🚀 Step 2 — Clone and Run

### 2.1 Install Git

```bash
# Ubuntu / Debian
sudo apt install -y git

# Fedora
sudo dnf install -y git

# Arch
sudo pacman -S git
```

### 2.2 Clone the Repository

```bash
cd ~
git clone https://github.com/your-username/ros2-jazzy-docker.git
cd ros2-jazzy-docker
chmod +x run.sh
```

### 2.3 Build and Start

```bash
# First build takes 5–10 minutes (downloads ~4 GB)
./run.sh start
```

You will see:

```
[OK]    Docker is running.
[INFO]  Building Docker image 'ros2-jazzy-dev:latest'...
...
[OK]    Container is up.

  GUI Desktop (open in your browser):
    http://localhost:6080/vnc.html

  VNC client (optional):
    vnc://localhost:5900  (no password)
```

The command builds the image (first time only), starts the container, and drops you into an interactive shell inside the ROS 2 environment.

---

## 🖥️ Step 3 — Open the GUI Desktop

### Browser (recommended — works everywhere, including headless servers with port-forwarding)

```
http://localhost:6080/vnc.html
```

Click **Connect**. A black/grey desktop appears — it becomes active once you launch a GUI application from the shell.

### VNC Client (optional)

| Client | Connection |
|:---|:---|
| **TigerVNC** (`sudo apt install tigervnc-viewer`) | `localhost:5900` |
| **Remmina** (pre-installed on Ubuntu) | `vnc://localhost:5900` |
| **Any VNC client** | `localhost:5900` — no password |

---

## 🤖 Step 4 — Launch ROS 2 Applications

From inside the container shell:

### Test the display

```bash
# X11 sanity check — animated eyes in the VNC desktop
xeyes

# OpenGL test — spinning gears via Mesa llvmpipe
glxgears

# Confirm software rendering is active
glxinfo | grep "OpenGL renderer"
# Expected: Mesa Software Rasterizer / llvmpipe (LLVM ...)
```

### Launch ROS GUI tools

```bash
rv          # RViz2  — 3D robot visualiser
rq          # rqt    — ROS 2 GUI framework
gz          # Gazebo Harmonic simulator
```

### Test ROS 2

```bash
ros2 --version
ros2 topic list

# Classic talker / listener demo
ros2 run demo_nodes_cpp talker &
ros2 run demo_nodes_cpp listener
```

### Build your packages

```bash
cd /ros2_ws
rosdep install --from-paths src --ignore-src -r -y
cb          # colcon build --symlink-install
cs          # source install/setup.bash
```

---

## ⌨️ Quick Reference — Aliases

| Alias | Command | Description |
|:---|:---|:---|
| `rv` | `rviz2` | RViz2 3D visualiser |
| `rq` | `rqt` | rqt GUI framework |
| `gz` | `gz sim` | Gazebo Harmonic simulator |
| `cb` | `colcon build --symlink-install` | Build workspace |
| `ct` | `colcon test` | Run tests |
| `cs` | `source install/setup.bash` | Source workspace |
| `rl` | `ros2 launch` | Launch a file |
| `rr` | `ros2 run` | Run a node |
| `rt` | `ros2 topic` | Topic tools |
| `rn` | `ros2 node` | Node tools |

---

## ⚙️ Useful `run.sh` Commands

```bash
./run.sh start    # Build (if needed) + start container + open shell
./run.sh shell    # Open another shell in the running container
./run.sh stop     # Stop and remove the container
./run.sh build    # Force-rebuild the image from scratch
./run.sh logs     # Tail container logs
./run.sh status   # Show container state
./run.sh gui      # Print GUI access info and diagnostics
```

---

## 🖥️ Headless Server Usage

The VNC architecture makes this project uniquely suited for headless Linux servers (cloud VMs, Raspberry Pi, CI runners, lab machines). No physical display, GPU, or desktop environment is needed on the host.

### Access via SSH tunnel

On your **laptop**:

```bash
# Forward remote ports 6080 (noVNC) and 5900 (VNC) to localhost
ssh -L 6080:localhost:6080 -L 5900:localhost:5900 user@your-server-ip

# Leave this SSH connection open
```

Then open your laptop's browser and navigate to:

```
http://localhost:6080/vnc.html
```

You will see the full desktop running on the remote server.

### Automated / non-interactive startup

```bash
# Start the container detached (no interactive shell)
docker compose up -d

# Confirm it is running
docker compose ps

# Tail logs
docker compose logs -f
```

### Running ROS nodes without a GUI (pure headless)

The `DISPLAY=:99` environment variable is set automatically. Every node in the container can use the virtual Xvfb display for GUI elements even when you have never opened a VNC viewer. This is useful for automated tests that use rqt or RViz fixtures.

```bash
# Run a complete simulation headlessly
docker exec -it ros_jazzy_dev bash -c "gz sim -s /path/to/world.sdf"
```

---

## 🔧 Troubleshooting

<details>
<summary><strong>❌ "permission denied while trying to connect to the Docker daemon socket"</strong></summary>

You are not in the `docker` group yet, or the group change hasn't been picked up.

```bash
sudo usermod -aG docker $USER
newgrp docker          # refresh group without logging out

# Or log out and back in fully, then verify:
groups | grep docker
docker run hello-world
```

</details>

<details>
<summary><strong>❌ "Got permission denied" when running `./run.sh`</strong></summary>

The script is not executable:

```bash
chmod +x run.sh
./run.sh start
```

</details>

<details>
<summary><strong>❌ Container exits immediately after starting</strong></summary>

Check the logs:

```bash
./run.sh logs
```

Common causes:

| Error in logs | Fix |
|:---|:---|
| `chown: changing ownership of '/tmp/.X11-unix'` | Rebuild the image: `./run.sh build` |
| `Xvfb did not start in time` | Run `./run.sh stop && ./run.sh start` to retry |
| `address already in use :5900` | A VNC server is already running on the host — stop it or remap ports |

</details>

<details>
<summary><strong>❌ noVNC "Failed to connect to server"</strong></summary>

Wait 5–10 seconds and refresh the browser. The `x11vnc` service takes a moment to bind after Xvfb starts.

Check whether port 5900 is listening:

```bash
ss -tlnp | grep 5900
# or
docker exec ros_jazzy_dev ss -tlnp | grep 5900
```

If nothing is listening, examine the container logs for VNC errors:

```bash
./run.sh logs | grep -i vnc
```

</details>

<details>
<summary><strong>❌ Port 5900 or 6080 already in use on the host</strong></summary>

Find what is using the port:

```bash
ss -tlnp | grep -E '5900|6080'
```

Option 1 — stop the conflicting service:

```bash
sudo systemctl stop vncserver   # example
```

Option 2 — remap ports in `docker-compose.yml`:

```yaml
ports:
  - "5901:5900"
  - "6081:6080"
```

Then access noVNC at `http://localhost:6081/vnc.html`.

</details>

<details>
<summary><strong>❌ Slow performance / Gazebo is sluggish</strong></summary>

This uses Mesa llvmpipe — CPU-only software rendering. It is not as fast as GPU-accelerated rendering, but is highly portable.

Improvements you can make:

1. **Allocate more CPUs to the container** — edit `docker-compose.yml`:
   ```yaml
   deploy:
     resources:
       limits:
         cpus: '4.0'
   ```

2. **Check that `LP_NUM_THREADS` is set** — inside the container:
   ```bash
   echo $LP_NUM_THREADS   # should be 4
   glxinfo | grep renderer
   ```

3. **Reduce the VNC resolution** — in `entrypoint.sh`, change `1280x720x16` to `1024x768x16`

4. **Use a simpler Gazebo world** when testing to reduce polygon count

</details>

<details>
<summary><strong>❌ DNS / package download failures inside the container</strong></summary>

The `docker-compose.yml` sets explicit DNS servers (`8.8.8.8`, `8.8.4.4`).

Test connectivity:

```bash
docker exec ros_jazzy_dev ping -c 2 8.8.8.8
docker exec ros_jazzy_dev apt-get update
```

If you are on a network that blocks `8.8.8.8`, update `docker-compose.yml` to use your local DNS:

```yaml
dns:
  - 192.168.1.1    # your router/nameserver
  - 8.8.8.8
```

</details>

<details>
<summary><strong>❌ ROS_DOMAIN_ID clashes with another machine on the same network</strong></summary>

By default, all ROS 2 nodes use `ROS_DOMAIN_ID=0` and will see each other over the local network if using the default DDS discovery.

Inside the container, set a unique domain:

```bash
export ROS_DOMAIN_ID=42
ros2 topic list
```

To make it permanent, add it to your `~/.bashrc` inside the container:

```bash
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
```

</details>

<details>
<summary><strong>⚠️ `docker build` fails with "no space left on device"</strong></summary>

Docker's build cache and images can fill up quickly.

Clean up unused resources:

```bash
docker system prune -a       # removes stopped containers, unused images, dangling build cache
docker volume prune          # removes unused volumes (WARNING: this deletes data)
```

Check disk usage:

```bash
docker system df
```

</details>

---

## 🏗️ Architecture on Linux

```
Linux Host (Ubuntu / Fedora / Arch / Headless Server)
└── 🐋  Docker Engine
    └── Container: ros_jazzy_dev
        ├── Xvfb :99             ← virtual X11 display (Mesa software render)
        ├── x11vnc :5900         ← VNC server for Xvfb
        ├── noVNC/websockify :6080   ← browser-based VNC UI
        ├── /ros2_ws/src  ←──────── bind-mount → ./src on host
        └── /ros2_ws/build, install, log  (named volumes, persist across restarts)

Browser (local or via SSH tunnel):
  http://localhost:6080/vnc.html  →  full GUI desktop
```

---

## 📚 Further Reading

| Resource | Link |
|:---|:---|
| Docker Engine Install | [![Docker](https://img.shields.io/badge/Docker-Engine_Install-2496ED?style=flat-square&logo=docker&logoColor=white)](https://docs.docker.com/engine/install/) |
| ROS 2 Jazzy Tutorials | [![ROS 2](https://img.shields.io/badge/ROS_2-Tutorials-22314E?style=flat-square&logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/Tutorials.html) |
| Gazebo Harmonic | [![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-FF6600?style=flat-square)](https://gazebosim.org/docs/harmonic/) |
| Mesa llvmpipe | [![Mesa](https://img.shields.io/badge/Mesa-llvmpipe_Docs-4A90D9?style=flat-square)](https://docs.mesa3d.org/drivers/llvmpipe.html) |
| noVNC | [![noVNC](https://img.shields.io/badge/noVNC-Web_VNC_Client-4A90D9?style=flat-square)](https://novnc.com/) |

---

<div align="center">

[← Back to Main README](../README.md) &nbsp;·&nbsp; [Windows Setup Guide →](windows.md)

[![ROS 2](https://img.shields.io/badge/powered_by-ROS_2_Jazzy-22314E?style=flat-square&logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/)
[![Docker](https://img.shields.io/badge/containerised_with-Docker-2496ED?style=flat-square&logo=docker&logoColor=white)](https://www.docker.com/)

</div>
