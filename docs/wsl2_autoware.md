# Windows / WSL2 / Ubuntu 24.04 / Autoware / Docker CARLA

This is the repository's primary deployment target:

```text
Windows host
└─ WSL2: Ubuntu 24.04, Python 3.12, ROS 2 Jazzy, existing Autoware workspace
   ├─ ADAS comparison tools and ROS nodes
   └─ Docker CARLA 0.10.0 → published RPC/streaming ports 2000/2001
```

## 1. WSL and the existing Autoware environment

In PowerShell, inspect the existing distribution:

```powershell
wsl --list --verbose
wsl --version
```

It should show Ubuntu 24.04 using WSL version 2. For a new installation, use
`wsl --install -d Ubuntu-24.04`; existing distributions do not need reinstalling.
See [Microsoft's WSL installation guide](https://learn.microsoft.com/en-us/windows/wsl/install).

Keep this checkout in the Linux filesystem, for example
`~/projects/ADAS-in-Autonomous-Car`. Run Linux scripts from the Ubuntu shell.
The repository enforces LF line endings for scripts and CSVs to keep Windows
Git settings from changing recorded sample hashes.

Use ROS 2 Jazzy for Ubuntu 24.04. Install missing ROS tools following the
[official Jazzy Debian-package instructions](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).
In an existing ROS installation, the project dependencies are:

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions python3-rosdep python3-venv \
  python3-matplotlib ros-jazzy-rosidl-default-generators
source /opt/ros/jazzy/setup.bash
export AUTOWARE_SETUP="$HOME/autoware/install/setup.bash"
```

Set AUTOWARE_SETUP to the actual setup file from your existing workspace or
container mount. Autoware 1.9.0 supports both Humble and Jazzy; select its Jazzy
environment for this project. Do not combine Humble and Jazzy overlays in one
shell. Preserve the Autoware version and dependency lockfiles already working
on your machine. See [Autoware source installation](https://docs.autoware.org/main/installation/autoware/source-installation/).

The wrapper sources ROS first, then AUTOWARE_SETUP, then this repository's overlay.
It does not install or upgrade Autoware and does not alter your shell startup files.

## 2. Docker CARLA

For Docker Desktop, enable the WSL2 engine and integration for the Ubuntu
distribution. Check `docker version` inside Ubuntu. GPU access must already work
with your Docker engine. See [Docker's WSL backend guide](https://docs.docker.com/desktop/features/wsl/).

If your CARLA container already exists, keep it and inspect its published ports:

```bash
docker ps --format 'table {{.Names}}\t{{.Image}}\t{{.Ports}}'
docker port YOUR_CARLA_CONTAINER
```

For a new container, an optional
[Compose configuration](../docker/carla.compose.yaml) pins the existing project
target, CARLA 0.10.0, with GPU access and offscreen rendering:

```bash
docker compose -f docker/carla.compose.yaml config
docker compose -f docker/carla.compose.yaml up -d
docker compose -f docker/carla.compose.yaml logs -f carla
```

These commands download/start CARLA only when you run them. Do not start a second
container on ports already used by an existing server. Override CARLA_IMAGE only
with an image compatible with your Python API and maps. This is the UE5 image
with CarlaUnreal.sh; UE4 images use a different launcher. See the
[CARLA UE5 Docker guide](https://carla-ue5.readthedocs.io/en/latest/build_docker/).

Default port publication is 127.0.0.1:2000 for RPC and 127.0.0.1:2001 for streaming.
The API runs in WSL and normally connects to this published host endpoint.
If Autoware and the ADAS scripts are also containerized, use a shared Docker
network's CARLA service name, or an appropriately published host endpoint.
host.docker.internal identifies the host from a Docker Desktop container; it is
not the default address for native WSL programs. See
[Docker Desktop networking](https://docs.docker.com/desktop/features/networking/).

If localhost is unreachable with your engine/WSL networking arrangement, supply
the explicit reachable host address with --host. WSL NAT and mirrored networking
differ; Windows-host access in NAT uses the default gateway, while mirrored
networking supports localhost. Consult
[Microsoft's networking guide](https://learn.microsoft.com/en-us/windows/wsl/networking).
The scripts do not change Windows firewall or WSL network settings.

## 3. Python API and connectivity

The model benchmark needs no CARLA package. Live runs need a Python 3.12 Linux
CARLA API matching the server. Use the compatible wheel from your existing CARLA
installation; installing an arbitrary latest package can mix server/client versions.

A separate API virtual environment can be selected without replacing ROS Python:

```bash
export ADAS_CARLA_PYTHON="$HOME/carla-py312/bin/python"
bash scripts/wsl_workflow.sh probe --host 127.0.0.1 --port 2000
```

The probe prints server/client versions, map and world settings without modifying
the simulator. Confirm the configured Town10HD_Opt map is available and loaded.
GPU/container startup failures and version mismatches must be resolved before
collecting comparison results.

## 4. Build and run

From the repository root:

```bash
bash scripts/wsl_workflow.sh doctor --require-wsl --require-autoware
bash scripts/wsl_workflow.sh build
bash scripts/wsl_workflow.sh replay output_csv:=/tmp/pid_shadow.csv controller:=pid
# Stop with Ctrl-C, then use a new recording filename:
bash scripts/wsl_workflow.sh replay output_csv:=/tmp/mpc_shadow.csv controller:=mpc
```

Use `standalone` instead of `build` if you only need replay without Autoware.
The doctor uses /usr/bin/python3 to check the Jazzy ABI; a separate CARLA venv is
checked by the probe. Docker/TCP status in the doctor is diagnostic unless
--require-carla is specified.

To monitor an already running Autoware graph:

```bash
bash scripts/wsl_workflow.sh monitor --ros-args \
  -p velocity_topic:=/vehicle/status/velocity_status
```

Use the same ROS_DOMAIN_ID and RMW_IMPLEMENTATION as Autoware in all participating
shells. The wrapper preserves these values. The monitor uses best-effort sensor
QoS. Replay defaults to reliable QoS and performs no vehicle actuation.

To run a direct CARLA comparison on a separate world/control session:

```bash
bash scripts/wsl_workflow.sh carla --controller both --host 127.0.0.1 --spawn-index 0
```

This runner owns synchronous ticking and its scenario actors. Do not run it
against a world being ticked by Autoware's CARLA bridge. The ROS shadow nodes
remain a separate evaluation path; they do not replace Autoware control.

## Validation status

Validated here: Ubuntu 24.04/Jazzy package build, shared controller tests, both
DDS replay flows, and the installed launch file with clean shutdown. The Autoware
monitor was also built and received an Autoware VelocityReport over DDS.

Windows/WSL GPU rendering and a live Docker CARLA comparison have not been
executed here: the CARLA endpoint refused connections. The Compose file is a
configuration template, not evidence of successful GPU startup.
