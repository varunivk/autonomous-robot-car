#!/usr/bin/env bash
set -euo pipefail

# VS Code installed through Snap can pass Ubuntu 20.04 runtime libraries to its
# integrated terminal. ROS 2 Jazzy runs on the host Ubuntu runtime; retaining
# those paths makes RViz and Gazebo load an incompatible libpthread / glibc.
strip_snap_paths() {
  local variable_name="$1"
  local original_value="${!variable_name-}"
  local filtered_value=""
  local entry

  IFS=':' read -ra path_entries <<< "$original_value"
  for entry in "${path_entries[@]}"; do
    [[ -z "$entry" ]] && continue
    case "$entry" in
      /snap/*|*/snap/*|/var/lib/snapd/*) continue ;;
    esac
    if [[ -n "$filtered_value" ]]; then
      filtered_value+=":"
    fi
    filtered_value+="$entry"
  done
  printf -v "$variable_name" '%s' "$filtered_value"
  export "$variable_name"
}

for path_variable in \
  LD_LIBRARY_PATH LIBRARY_PATH PYTHONPATH PKG_CONFIG_PATH \
  CMAKE_PREFIX_PATH AMENT_PREFIX_PATH XDG_DATA_DIRS; do
  strip_snap_paths "$path_variable"
done

# These variables point directly at Snap's GTK / Qt plugin ABI. Let the host
# desktop and ROS vendor packages discover their matching plugins instead.
unset GDK_PIXBUF_MODULEDIR GDK_PIXBUF_MODULE_FILE GIO_MODULE_DIR
unset GTK_EXE_PREFIX GTK_IM_MODULE_FILE GTK_PATH
unset QT_PLUGIN_PATH QT_QPA_PLATFORM_PLUGIN_PATH QML2_IMPORT_PATH

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"

# Only one project launch may own the ROS clock at a time. A unique Gazebo
# transport partition also isolates a new run from any orphaned gz server.
exec 9>"/tmp/autonomous_robot_car_${UID}.lock"
if ! flock -n 9; then
  echo "Autonomous Robot Car is already running." >&2
  echo "Stop the other ./run_sim.sh with Ctrl+C before starting another." >&2
  exit 2
fi
export GZ_PARTITION="autonomous_robot_car_${UID}_${BASHPID}"

set +u
source "$ROS_SETUP"
set -u
if [[ ! -f "$PROJECT_DIR/install/setup.bash" ]]; then
  echo "Workspace is not built. Run ./build.sh first." >&2
  exit 1
fi
set +u
source "$PROJECT_DIR/install/setup.bash"
set -u

exec ros2 launch autonomous_robot_car sim.launch.py "$@"
