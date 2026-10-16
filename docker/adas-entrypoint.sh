#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
if [[ -f /opt/autoware/setup.bash ]]; then
  source /opt/autoware/setup.bash
fi
if [[ -f /workspace/do18-ros/autoware/install/setup.bash ]]; then
  source /workspace/do18-ros/autoware/install/setup.bash
fi
export PATH="/opt/adas-venv/bin:$PATH"
export PYTHONPATH="/workspace/adas/src${PYTHONPATH:+:$PYTHONPATH}"
exec "$@"
