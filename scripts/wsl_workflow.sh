#!/usr/bin/env bash
# Ubuntu 24.04 / WSL2 workflow, using an existing Jazzy Autoware installation.
set -euo pipefail
ADAS_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
ADAS_ACTION="${1:-help}"
if [[ $# -gt 0 ]]; then shift; fi
if [[ "$ADAS_ACTION" == "help" || "$ADAS_ACTION" == "--help" ]]; then
  cat <<'USAGE'
Usage: bash scripts/wsl_workflow.sh ACTION [arguments]
  doctor      Environment report; add --require-wsl --require-autoware --require-carla as needed
  build       Build standalone ROS nodes plus the Autoware monitor
  standalone  Build only the standalone ROS nodes
  replay      Launch replay; pass output_csv:=/absolute/new.csv controller:=pid|mpc
  monitor     Subscribe to Autoware velocity status; ROS arguments forwarded
  probe       Read-only CARLA connection check; --host/--port supported
  carla       Run CARLA comparison; --host/--port/--spawn-index supported
Set AUTOWARE_SETUP to your existing Jazzy Autoware install/setup.bash.
USAGE
  exit 0
fi
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "ROS 2 Jazzy is missing: see docs/wsl2_autoware.md" >&2; exit 1
fi
if [[ -n "${ROS_DISTRO:-}" && "$ROS_DISTRO" != jazzy ]]; then
  echo "Open a clean shell: a different ROS distribution is sourced." >&2; exit 1
fi
# ROS setup files are not nounset-safe.
set +u
source /opt/ros/jazzy/setup.bash
if [[ -n "${AUTOWARE_SETUP:-}" ]]; then
  if [[ ! -f "$AUTOWARE_SETUP" ]]; then echo "AUTOWARE_SETUP does not exist." >&2; exit 1; fi
  source "$AUTOWARE_SETUP"
fi
set -u
if [[ "${ROS_DISTRO:-}" != jazzy ]]; then echo "Autoware overlay must use Jazzy." >&2; exit 1; fi
case "$ADAS_ACTION" in
  doctor) exec /usr/bin/python3 "$ADAS_ROOT/scripts/check_environment.py" "$@" ;;
  build|standalone)
    ADAS_PACKAGES=(town10_adas_ros)
    if [[ "$ADAS_ACTION" == build ]]; then
      ros2 pkg prefix autoware_vehicle_msgs >/dev/null || { echo "Set AUTOWARE_SETUP to your Jazzy overlay." >&2; exit 1; }
      ADAS_PACKAGES+=(town10_adas_monitor)
    fi
    cd "$ADAS_ROOT/ros2_ws"
    exec colcon build --symlink-install --packages-up-to "${ADAS_PACKAGES[@]}" "$@" ;;
  probe) exec "${ADAS_CARLA_PYTHON:-/usr/bin/python3}" "$ADAS_ROOT/scripts/check_carla_integration.py" "$@" ;;
  carla)
    cd "$ADAS_ROOT"
    exec "${ADAS_CARLA_PYTHON:-/usr/bin/python3}" scripts/run_town10_adas.py "$@" ;;
  replay|monitor)
    if [[ ! -f "$ADAS_ROOT/ros2_ws/install/setup.bash" ]]; then echo "Build the workspace first." >&2; exit 1; fi
    set +u
    source "$ADAS_ROOT/ros2_ws/install/setup.bash"
    set -u
    if [[ "$ADAS_ACTION" == monitor ]]; then exec ros2 run town10_adas_monitor vehicle_monitor "$@"; fi
    exec ros2 launch town10_adas_ros replay.launch.py "input_csv:=$ADAS_ROOT/results/model/urban_15_10/pid.csv" "$@" ;;
  *) echo "Unknown action: $ADAS_ACTION (use help)" >&2; exit 2 ;;
esac
