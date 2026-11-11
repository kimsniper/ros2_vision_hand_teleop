#!/usr/bin/env bash

set -e
set -o pipefail

if [ -z "${ROS_DISTRO:-}" ]; then
    echo "ERROR: ROS_DISTRO is not set."
    echo "Source ROS 2 first, for example:"
    echo "  source /opt/ros/humble/setup.bash"
    exit 1
fi

WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SOURCE_ROOT="${WORKSPACE_ROOT}/src"

if [ ! -d "${SOURCE_ROOT}" ]; then
    echo "ERROR: Workspace source directory does not exist: ${SOURCE_ROOT}"
    exit 1
fi

if ! command -v rosdep >/dev/null 2>&1; then
    echo "ERROR: rosdep is not installed."
    echo "Install it with:"
    echo "  sudo apt install python3-rosdep"
    exit 1
fi

python3 -m pip install --user \
    mediapipe \
    numpy \
    opencv-python

rosdep update
rosdep install \
    --from-paths "${SOURCE_ROOT}" \
    --ignore-src \
    --rosdistro "${ROS_DISTRO}" \
    -r \
    -y

echo "Dependencies installed successfully."
echo "Build the workspace with:"
echo "  cd ${WORKSPACE_ROOT}"
echo "  colcon build --symlink-install"
