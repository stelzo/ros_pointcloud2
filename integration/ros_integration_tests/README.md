# ros_integration_tests

Integration test crate for `ros_pointcloud2` that tests conversion macros from and to ROS client crates.

To run tests against ROS2 Jazzy serde messages (feature: `ros2-interfaces-jazzy-serde`):

## Locally (from repo root):
```sh
./integration/scripts/run-ros-tests-local.sh ros2-interfaces-jazzy-serde
```

## In Docker:
```sh
# ros2-interfaces-jazzy-serde
./integration/scripts/run-ros-tests-docker.sh ros2-interfaces-jazzy-serde docker/Dockerfile_r2r_jazzy

# r2r
./integration/scripts/run-ros-tests-docker.sh r2r docker/Dockerfile_r2r_jazzy

# rosrust
./integration/scripts/run-ros-tests-docker.sh rosrust docker/Dockerfile_ros1_noetic
```

## Equivalent Cargo command:
```sh
cargo test -p ros_integration_tests --features ros2-interfaces-jazzy-serde
```
