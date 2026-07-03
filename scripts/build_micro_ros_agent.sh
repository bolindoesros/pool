#!/usr/bin/env bash
set -e

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
EXTERNAL_DIR="$REPO_ROOT/external"

mkdir -p "$EXTERNAL_DIR"
cd "$EXTERNAL_DIR"

clone_or_update() {
local repo_url="$1"
local branch="$2"
local dir_name="$3"

if [ -d "$dir_name/.git" ]; then
echo "[INFO] Updating $dir_name..."
cd "$dir_name"
git fetch origin
git checkout "$branch"
git pull origin "$branch"
cd "$EXTERNAL_DIR"
else
echo "[INFO] Cloning $dir_name..."
git clone -b "$branch" "$repo_url" "$dir_name"
fi
}

clone_or_update "https://github.com/micro-ROS/micro-ROS-Agent.git" "humble" "micro-ROS-Agent"
clone_or_update "https://github.com/micro-ROS/micro_ros_msgs.git" "humble" "micro_ros_msgs"

echo "[INFO] External repositories ready in: $EXTERNAL_DIR"

source /opt/ros/humble/setup.bash

echo "[INFO] Installing dependencies..."
rosdep install --from-paths . --ignore-src -r -y --rosdistro humble || true

echo "[INFO] Building micro-ROS Agent workspace..."
colcon build --merge-install

echo "[INFO] Checking build output..."
if [ ! -f "$EXTERNAL_DIR/install/setup.bash" ]; then
echo "ERROR: Build finished, but install/setup.bash was not created."
exit 1
fi

source "$EXTERNAL_DIR/install/setup.bash"

echo "[INFO] Checking micro_ros_agent package..."
ros2 pkg prefix micro_ros_agent

echo "[DONE] micro-ROS Agent built successfully."
echo "Setup file:"
echo "$EXTERNAL_DIR/install/setup.bash"
