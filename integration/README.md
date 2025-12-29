# Integration Tests (ros_integration_tests)

Summary: this directory contains integration tests, Dockerfiles and helper scripts to run ROS-related tests either locally or in Docker.

## 🔧 Overview

- Integration test crate: `integration/ros_integration_tests`
- Dockerfiles: `integration/docker/` (multiple ROS distributions / setups)
- Helper scripts: `integration/scripts/run-ros-tests-local.sh` and `integration/scripts/run-ros-tests-docker.sh`


## Local execution (non-Docker)

Local integration tests build ROS client crates — make sure your ROS environment is sourced and ready.

Examples:

```bash
# From repository root — run all integration tests
./integration/scripts/run-ros-tests-local.sh
# Or run only tests matching a pattern (optional):
./integration/scripts/run-ros-tests-local.sh "ros2_*"

# Or run tests directly:
cargo test -p ros_integration_tests -- --nocapture
```

Note: export `ROS_DISTRO` and any other env vars (e.g. `CMAKE_PREFIX_PATH`) if required before running the script.

## Docker-backed execution

```bash
# Usage: <target> (optional) picks a sensible Dockerfile; pass a repo-root-relative Dockerfile path if you prefer
./integration/scripts/run-ros-tests-docker.sh [target] [integration/docker/Dockerfile_*] [mode]

# Example (repo-root-relative Dockerfile path):
./integration/scripts/run-ros-tests-docker.sh r2r integration/docker/Dockerfile_r2r_humble
```

**rclrs special case:** the `rclrs` images expect the repository to be mounted at `/ros2_rust_ws` and use an ENTRYPOINT script to build and run tests. When running the `rclrs` target, the script will mount the repo root at `/ros2_rust_ws` in `bind` mode, or copy the repo into `/ros2_rust_ws` in `copy` mode so the image entrypoint can find and run the test script.

Mode description:

- `copy` (default): creates a tarball of the repository and copies it into a temporary container (suitable for reproducible CI runs and the default behavior).
- `bind`: mounts the repository root into the container and runs tests on the real workspace. Useful for iterative development.

## Available Dockerfiles (in `integration/docker`) 🧾

- `Dockerfile_r2r_jazzy`
- `Dockerfile_r2r_galactic`
- `Dockerfile_r2r_humble`
- `Dockerfile_r2r_iron`
- `Dockerfile_rclrs_jazzy`
- `Dockerfile_rclrs_humble`
- `Dockerfile_rclrs_iron`
- `Dockerfile_ros1_noetic`
- `Dockerfile_safe_drive_jazzy`

Example for ROS1 (rosrust):

```bash
./integration/scripts/run-ros-tests-docker.sh rosrust integration/docker/Dockerfile_ros1_noetic
```

## Notes about test layout

- Some tests have been moved to the integration crate (e.g. `r2r`, `rosrust`, `ros2-interfaces-jazzy-*`). Use `cargo test -p ros_integration_tests -- --nocapture` or the helper scripts above.
- Crate unit tests remain in `ros_pointcloud2/src/tests.rs` and run with `cargo test -p ros_pointcloud2`.
