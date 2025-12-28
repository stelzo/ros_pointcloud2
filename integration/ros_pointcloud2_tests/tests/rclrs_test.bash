#!/usr/bin/env bash
set -euo pipefail

# Run tests inside the test node crate with the requested features.
FEATURES="${1:-rclrs}"

# Source ROS installation
for s in /opt/ros/*/setup.sh; do [ -f "$s" ] && . "$s"; done

cd /ros2_rust_build/ros_pointcloud2_tests

# Run tests (pass -- --nocapture so we see output) using the crate's manifest to avoid workspace root interference
cargo test --manifest-path Cargo.toml -p ros_pointcloud2_tests --features "$FEATURES" --verbose -- --nocapture
