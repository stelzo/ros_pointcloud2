#!/usr/bin/env bash
set -euo pipefail

# Run ROS integration tests locally (non-Docker).
# This will attempt to build ROS client crates and may require a sourced ROS env.
# Usage: ./scripts/run-ros-tests-local.sh [test-filter]
# Example: ./scripts/run-ros-tests-local.sh "ros2_*"

ARG1=${1:-}
ARG2=${2:-}

# Map common targets to cargo feature flags
FEATURES=""
case "$ARG1" in
  r2r) FEATURES="r2r" ;;
  rosrust) FEATURES="rosrust" ;;
  rclrs) FEATURES="rclrs" ;;
  ros2-interfaces-jazzy-serde|ros2_jazzy_serde) FEATURES="ros2-interfaces-jazzy-serde" ;;
  ros2-interfaces-jazzy-rkyv|ros2_jazzy_rkyv) FEATURES="ros2-interfaces-jazzy-rkyv" ;;
  nalgebra) FEATURES="nalgebra" ;;
  "") FEATURES="" ;;
  *) FEATURES="" ;;
esac

echo "Make sure the ROS environment is sourced (e.g. 'source /opt/ros/$(printenv ROS_DISTRO || echo <distro>)/setup.bash')"

if [[ -n "$FEATURES" ]]; then
  echo "Running integration tests locally with features: $FEATURES"
  if [[ -n "$ARG2" ]]; then
    echo "Applying test filter: $ARG2"
    cargo test -p ros_integration_tests --features "$FEATURES" -- "$ARG2" -- --nocapture
  else
    cargo test -p ros_integration_tests --features "$FEATURES" -- --nocapture
  fi
elif [[ -n "$ARG1" ]]; then
  # ARG1 treated as a test filter if it's not a known target
  echo "Running integration tests locally (filter: $ARG1)"
  cargo test -p ros_integration_tests -- "$ARG1" -- --nocapture
else
  echo "Running all integration tests locally"
  cargo test -p ros_integration_tests -- --nocapture
fi
