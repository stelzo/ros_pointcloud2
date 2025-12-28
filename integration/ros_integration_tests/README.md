# ros_integration_tests

Integration test crate for `ros_pointcloud2` that tests conversion macros from and to ROS client crates.

Usage

- To run tests against ROS2 Jazzy serde messages (feature: `ros2-interfaces-jazzy-serde`):

  # Locally (from repo root):
  ./integration/scripts/run-ros-tests-local.sh ros2-interfaces-jazzy-serde

  # In Docker:
  ./integration/scripts/run-ros-tests-docker.sh ros2-interfaces-jazzy-serde integration/docker/Dockerfile_r2r_jazzy

  # Equivalent Cargo command:
  cargo test -p ros_integration_tests --features ros2-interfaces-jazzy-serde

- To run r2r tests (feature: `r2r`):

  ./integration/scripts/run-ros-tests-local.sh r2r
  ./integration/scripts/run-ros-tests-docker.sh r2r integration/docker/Dockerfile_r2r_jazzy

- To run ROS1 (rosrust) tests (feature: `rosrust`):

  ./integration/scripts/run-ros-tests-local.sh rosrust
  ./integration/scripts/run-ros-tests-docker.sh rosrust integration/docker/Dockerfile_ros1_noetic

Notes

- These tests are intentionally feature-gated so that normal `cargo test` in the workspace
  does not attempt to build ROS client crates or their native build scripts.
- Prefer running the tests in a Docker image or CI job that has ROS installed and sourced.
