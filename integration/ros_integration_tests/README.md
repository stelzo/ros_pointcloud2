# ros_integration_tests

Integration test crate for `ros_pointcloud2` that tests conversion macros from and to ROS client crates.

To run integration tests for the various ROS clients:

## Locally (from repo root):
```sh
# Run all integration tests
./integration/scripts/run-ros-tests-local.sh

# Or run only tests matching a pattern (optional)
./integration/scripts/run-ros-tests-local.sh "ros2_*"
```

## In Docker (optional target selects a sensible Dockerfile):
```sh
# Jazzy ROS2 (serde)
./integration/scripts/run-ros-tests-docker.sh r2r integration/docker/Dockerfile_r2r_humble

# r2r
./integration/scripts/run-ros-tests-docker.sh r2r integration/docker/Dockerfile_r2r_humble

# rosrust
./integration/scripts/run-ros-tests-docker.sh rosrust integration/docker/Dockerfile_ros1_noetic
```

**rclrs special case:** the `rclrs` images perform their own build + run via an ENTRYPOINT script and expect the workspace at `/ros2_rust_ws`. Use the repo-root-relative Dockerfile for `rclrs` (e.g. `integration/docker/Dockerfile_rclrs_humble`) and prefer `copy` mode or `bind` mode that mounts the repo root.

## Equivalent Cargo command:
```sh
cargo test -p ros_integration_tests -- --nocapture
```
