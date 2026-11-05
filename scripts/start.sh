#!/usr/bin/env bash
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"
mkdir -p "$PKG/rbnx-build/ros_logs" "$PKG/rbnx-build/ros_home"
export ROS_LOG_DIR="$PKG/rbnx-build/ros_logs" ROS_HOME="$PKG/rbnx-build/ros_home"
set +u
if [[ -n "${ROS_SETUP:-}" ]]; then
    source "$ROS_SETUP"
elif [[ -f "/opt/ros/${ROS_DISTRO:-humble}/setup.bash" ]]; then
    source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
elif [[ -z "${AMENT_PREFIX_PATH:-}" ]]; then
    echo "Source a ROS 2 environment or set ROS_SETUP to its setup.bash" >&2
    exit 1
fi
set -u
export PYTHONPATH="$(rbnx path robonix-api):$PKG:${PYTHONPATH:-}"
exec "${G70_PYTHON:-/usr/bin/python3}" -u -m g70_gnss.atlas_bridge
