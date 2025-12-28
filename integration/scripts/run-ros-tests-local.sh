#!/usr/bin/env bash
set -euo pipefail

# Run ROS integration tests locally (non-Docker).
# This will attempt to build ROS client crates and may require a sourced ROS env.
# Usage: ./scripts/run-ros-tests-local.sh <feature>
# Example: ./scripts/run-ros-tests-local.sh ros2-interfaces-jazzy-serde

FEATURE=${1:-}
if [[ -z "$FEATURE" ]]; then
  echo "Usage: $0 <feature>"
  exit 2
fi

echo "Running integration tests locally with feature: $FEATURE"

# Note: The user must ensure ROS env is prepared (e.g. source /opt/ros/<distro>/setup.bash)
# and any env vars required by the ROS crates (ROS_DISTRO, CMAKE_PREFIX_PATH) are set.

echo "Make sure the ROS environment is sourced (e.g. 'source /opt/ros/$(printenv ROS_DISTRO || echo <distro>)/setup.bash')"

cargo test -p ros_integration_tests --features "$FEATURE" -- --nocapture
