#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"

if [[ ! -f "$ROS_SETUP" ]]; then
  echo "ROS setup file not found: $ROS_SETUP" >&2
  exit 1
fi

set +u
source "$ROS_SETUP"
set -u
cd "$PROJECT_DIR"
colcon build --symlink-install --event-handlers console_direct+

echo
echo "Build complete. Run: source install/setup.bash"
