<div align="center">

# ROS 2 Jazzy on Windows · Step-by-Step Setup Guide

**Run a full ROS 2 Jazzy desktop — including RViz2, rqt and Gazebo — on Windows 10/11 using WSL 2, Docker, and the browser-based VNC desktop. No VcXsrv, no Xming, no manual X11 configuration.**

<br/>

[![Windows](https://img.shields.io/badge/Windows-10%2F11-0078D4?style=for-the-badge&logo=windows&logoColor=white)](https://www.microsoft.com/windows)
[![WSL 2](https://img.shields.io/badge/WSL-2-FCC624?style=for-the-badge&logo=linux&logoColor=black)](https://docs.microsoft.com/en-us/windows/wsl/)
[![Docker](https://img.shields.io/badge/Docker-Desktop-2496ED?style=for-the-badge&logo=docker&logoColor=white)](https://www.docker.com/products/docker-desktop/)
[![ROS 2 Jazzy](https://img.shields.io/badge/ROS_2-Jazzy-22314E?style=for-the-badge&logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/)

<br/>

[Step 1 — WSL 2](#-step-1--enable-wsl-2) &nbsp;·&nbsp;
[Step 2 — Ubuntu](#-step-2--install-ubuntu) &nbsp;·&nbsp;
[Step 3 — Docker](#-step-3--install-docker-desktop) &nbsp;·&nbsp;
[Step 4 — Clone & Run](#-step-4--clone-and-run) &nbsp;·&nbsp;
[Step 5 — GUI Desktop](#-step-5--open-the-gui-desktop) &nbsp;·&nbsp;
[Troubleshooting](#-troubleshooting)

</div>

---

## 📋 Requirements

| Requirement | Version | Notes |
|:---|:---:|:---|
| **Windows** | 10 (21H2+) or 11 | Windows 11 recommended for WSLg GUI support |
| **WSL** | 2 | Version 1 is not supported |
| **Docker Desktop** | 4.x | Must use the WSL 2 backend |
| **Ubuntu (WSL)** | 22.04 or 24.04 | Other Debian-based distros also work |
| **RAM** | 8 GB+ | 4 GB minimum; Gazebo needs more |
| **Disk Space** | ~10 GB free | For Docker image + build cache |

> [!NOTE]
> **Windows 11 users:** WSLg (built-in GUI support) is available and allows GUI apps to render natively without VNC. However, this project's VNC-based approach also works perfectly on Windows 11 and is the recommended method for consistency across all platforms.

---

## 🪟 Step 1 — Enable WSL 2

Open **PowerShell as Administrator** (right-click → *Run as Administrator*):

```powershell
# Install WSL and set version 2 as default
wsl --install
wsl --set-default-version 2
```

**Restart your computer** when prompted — this is required.

After restarting, verify WSL 2 is active:

```powershell
wsl --status
```

You should see:

```
Default Version: 2
```

> [!TIP]
> If you already have WSL installed, upgrade an existing distro to version 2 with:
> ```powershell
> wsl --set-version Ubuntu 2
> ```

---

## 🐧 Step 2 — Install Ubuntu

### Option A — From the Microsoft Store (recommended)

1. Open the **Microsoft Store**
2. Search for **Ubuntu 24.04 LTS**
3. Click **Get** → **Install**
4. Launch Ubuntu from the Start Menu
5. Create a **username** and **password** when prompted

### Option B — From PowerShell

```powershell
wsl --install -d Ubuntu-24.04
```

### Verify Ubuntu is running WSL 2

```powershell
wsl --list --verbose
```

Expected output:

```
  NAME            STATE           VERSION
* Ubuntu-24.04    Running         2
```

---

## 🐳 Step 3 — Install Docker Desktop

### 3.1 Download and Install

[![Download Docker Desktop for Windows](https://img.shields.io/badge/⬇️_Download_Docker_Desktop_for_Windows-2496ED?style=for-the-badge&logo=docker&logoColor=white)](https://www.docker.com/products/docker-desktop/)

During installation:
- ✅ **Use WSL 2 instead of Hyper-V** — keep this checked
- ✅ **Add shortcut to desktop** — optional

**Restart your computer** after installation completes.

### 3.2 Enable WSL 2 Integration

After Docker Desktop starts:

1. Open Docker Desktop
2. Go to **Settings → Resources → WSL Integration**
3. Enable integration for your Ubuntu distro:

```
✅ Enable integration with my default WSL distro
✅ Ubuntu-24.04
```

4. Click **Apply & Restart**

### 3.3 Allocate Sufficient Resources

*Settings → Resources → Advanced*

| Resource | Recommended | Minimum |
|:---|:---:|:---:|
| CPUs | 4 | 2 |
| Memory | 8 GB | 4 GB |
| Swap | 2 GB | 1 GB |
| Disk | 60 GB | 20 GB |

Click **Apply & Restart**.

---

## 🚀 Step 4 — Clone and Run

Open **Ubuntu** from the Start Menu (or run `wsl` in PowerShell).

### 4.1 Update Ubuntu

```bash
sudo apt update && sudo apt upgrade -y
```

### 4.2 Install Git

```bash
sudo apt install git -y
```

### 4.3 Verify Docker is accessible from WSL

```bash
docker --version
docker run hello-world
```

Expected:
```
Docker version 27.x.x ...
Hello from Docker!
```

If `docker` is not found, ensure Docker Desktop is running and WSL integration is enabled (Step 3.2).

### 4.4 Clone the Repository

```bash
# Navigate to your home directory
cd ~

# Clone the project
git clone https://github.com/your-username/ros2-jazzy-docker.git
cd ros2-jazzy-docker

# Make the helper script executable
chmod +x run.sh
```

### 4.5 Build and Start

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

---

## 🖥️ Step 5 — Open the GUI Desktop

### Browser (recommended — no install needed)

Open any browser on Windows and navigate to:

```
http://localhost:6080/vnc.html
```

Click **Connect**. You will see the desktop (black screen is normal until you launch an app).

### VNC Client (optional)

| Client | Download | Connection |
|:---|:---|:---|
| **TigerVNC** | [tigervnc.org](https://tigervnc.org/) | `localhost:5900` |
| **RealVNC Viewer** | [realvnc.com](https://www.realvnc.com/en/connect/download/viewer/) | `localhost:5900` |
| **Built-in** | Windows 11 Remote Desktop | `localhost:5900` |

No password required.

---

## 🤖 Step 6 — Launch ROS 2 Applications

From inside the container shell (opened automatically by `./run.sh start`):

### Test the display

```bash
# X11 sanity check — a small animated window appears in the VNC desktop
xeyes

# OpenGL check — spinning gears via Mesa llvmpipe
glxgears

# Check OpenGL renderer
glxinfo | grep "OpenGL renderer"
# Expected: Mesa llvmpipe (LLVM ...)
```

### Launch ROS GUI tools

```bash
rv          # RViz2 — 3D robot visualiser
rq          # rqt   — ROS 2 GUI framework
gz          # Gazebo Harmonic simulator
```

### Test ROS 2

```bash
# Check ROS version
ros2 --version

# List active topics
ros2 topic list

# Run the talker/listener demo
ros2 run demo_nodes_cpp talker &
ros2 run demo_nodes_cpp listener
```

### Build your own packages

```bash
cd /ros2_ws

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build
cb              # alias → colcon build --symlink-install

# Source
cs              # alias → source install/setup.bash
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

## 🔧 Troubleshooting

<details>
<summary><strong>❌ Docker command not found in WSL</strong></summary>

Docker Desktop must be running on Windows before you open WSL.

1. Start Docker Desktop from the Windows Start Menu
2. Wait for the engine to start (whale icon in system tray turns solid)
3. Verify WSL integration is enabled: *Docker Desktop → Settings → Resources → WSL Integration → ✅ Ubuntu-24.04*
4. Close and reopen your WSL terminal

```bash
docker --version   # should now work
```

</details>

<details>
<summary><strong>❌ "permission denied" when running docker commands</strong></summary>

Add your user to the docker group:

```bash
sudo usermod -aG docker $USER
```

Close Ubuntu completely and reopen. Then verify:

```bash
docker run hello-world
```

</details>

<details>
<summary><strong>❌ Container exits immediately / black screen in VNC</strong></summary>

Check the container logs:

```bash
./run.sh logs
```

Common fixes:

1. **Increase Docker memory** — Gazebo needs at least 4 GB. Go to *Docker Desktop → Settings → Resources → Memory*
2. **Restart the container** — `./run.sh stop && ./run.sh start`
3. **Rebuild the image** — `./run.sh build`

The black screen is **normal** — launch a GUI app from the container shell to see something:
```bash
xeyes
```

</details>

<details>
<summary><strong>❌ noVNC "Failed to connect to server"</strong></summary>

Wait 5–10 seconds after starting and then refresh the browser. The `x11vnc` service needs a moment to bind to port 5900.

If it persists:

```bash
./run.sh logs
# look for x11vnc errors
```

Also check Docker Desktop is not blocking ports 5900/6080:
*Docker Desktop → Settings → General → uncheck "Use kernel networking for UDP"* (Windows 11 only if it appears)

</details>

<details>
<summary><strong>❌ Ports 5900 or 6080 already in use</strong></summary>

Check what is using the port:

```powershell
# Run in PowerShell on Windows
netstat -ano | findstr :5900
netstat -ano | findstr :6080
```

Or remap the ports in `docker-compose.yml`:

```yaml
ports:
  - "5901:5900"   # host:container
  - "6081:6080"
```

Then access noVNC at `http://localhost:6081/vnc.html`.

</details>

<details>
<summary><strong>❌ Slow performance / Gazebo is sluggish</strong></summary>

This setup uses software rendering (Mesa llvmpipe) — it is CPU-bound.

1. **Increase CPU allocation** — *Docker Desktop → Settings → Resources → CPUs → 4+*
2. **Increase memory** — *Settings → Resources → Memory → 8 GB*
3. **Close other heavy apps** on Windows while running Gazebo
4. **Tune WSL memory** — create or edit `%USERPROFILE%\.wslconfig`:

```ini
[wsl2]
memory=8GB
processors=4
```

Then restart WSL: run `wsl --shutdown` in PowerShell, then reopen Ubuntu.

</details>

<details>
<summary><strong>❌ DNS / network failures inside the container</strong></summary>

The `docker-compose.yml` already sets explicit DNS servers (`8.8.8.8`, `8.8.4.4`).

If you are on a corporate network or VPN, these may be blocked. Try:

```bash
# Inside the container
ping -c 2 8.8.8.8          # test raw connectivity
ping -c 2 google.com       # test DNS resolution
```

If only DNS fails, add your corporate DNS to `docker-compose.yml`:
```yaml
dns:
  - 10.x.x.x    # your corporate DNS
  - 8.8.8.8
```

</details>

<details>
<summary><strong>⚠️ WSL 2 using too much memory on Windows</strong></summary>

WSL 2 can consume a large portion of RAM. Limit it by creating `%USERPROFILE%\.wslconfig`:

```ini
[wsl2]
memory=8GB
processors=4
swap=2GB
```

Apply changes:
```powershell
wsl --shutdown
```

Then reopen Ubuntu.

</details>

<details>
<summary><strong>❌ `./run.sh: line X: docker: command not found` inside WSL</strong></summary>

Make sure you are running the command from within WSL (Ubuntu terminal), not from PowerShell or CMD. Docker Desktop exposes the `docker` CLI into WSL — it does not expose it in PowerShell unless Docker Desktop for Windows CLI is installed separately.

</details>

---

## 🏗️ Architecture on Windows

```
Windows 10 / 11
├── 🐋  Docker Desktop (WSL 2 backend)
└── 🐧  WSL 2 — Ubuntu 24.04
    └── Container: ros_jazzy_dev
        ├── Xvfb :99            ← virtual display (Mesa software render)
        ├── x11vnc :5900        ← VNC server
        ├── noVNC/websockify :6080  ← browser VNC UI
        ├── /ros2_ws/src   ←── bind-mount → ~/ros2-jazzy-docker/src
        └── /ros2_ws/build, install, log  (named volumes)

Browser on Windows:
  http://localhost:6080/vnc.html  →  full GUI desktop
```

---

## 📚 Further Reading

| Resource | Link |
|:---|:---|
| WSL 2 Documentation | [![WSL Docs](https://img.shields.io/badge/Microsoft-WSL_2_Docs-0078D4?style=flat-square&logo=windows&logoColor=white)](https://docs.microsoft.com/en-us/windows/wsl/) |
| Docker Desktop for Windows | [![Docker](https://img.shields.io/badge/Docker-Desktop_Windows-2496ED?style=flat-square&logo=docker&logoColor=white)](https://docs.docker.com/desktop/install/windows-install/) |
| ROS 2 Jazzy Tutorials | [![ROS 2](https://img.shields.io/badge/ROS_2-Tutorials-22314E?style=flat-square&logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/Tutorials.html) |
| Gazebo Harmonic | [![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-FF6600?style=flat-square)](https://gazebosim.org/docs/harmonic/) |
| noVNC | [![noVNC](https://img.shields.io/badge/noVNC-Web_VNC_Client-4A90D9?style=flat-square)](https://novnc.com/) |

---

<div align="center">

[← Back to Main README](../README.md) &nbsp;·&nbsp; [Linux Setup Guide →](linux.md)

[![ROS 2](https://img.shields.io/badge/powered_by-ROS_2_Jazzy-22314E?style=flat-square&logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/)
[![Docker](https://img.shields.io/badge/containerised_with-Docker-2496ED?style=flat-square&logo=docker&logoColor=white)](https://www.docker.com/)

</div>
