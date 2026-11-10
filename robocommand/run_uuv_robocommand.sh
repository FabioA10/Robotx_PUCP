#!/usr/bin/env bash
# Start the UUV RoboCommand bridge after MAVROS has been built in its isolated
# workspace. Run this only when uuv_mavlink/Point-to-Point command output is
# stopped, so ArduSub has a single command authority.
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$repo_root/UUV_ws/mavros_ardusub_ws/install/setup.bash"
source "$(dirname "${BASH_SOURCE[0]}")/.venv/bin/activate"
source /opt/ros/jazzy/setup.bash
source "$(dirname "${BASH_SOURCE[0]}")/.install/setup.bash"

exec ros2 launch robocommand uuv_robocommand.launch.py "$@"
