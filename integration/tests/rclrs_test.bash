#!/bin/bash

. "$HOME/.cargo/env"

if [ -e "/opt/ros/jazzy/setup.bash" ]; then
    . "/opt/ros/jazzy/setup.bash"
fi

if [ -e "/opt/ros/rolling/setup.bash" ]; then
    . "/opt/ros/rolling/setup.bash"
fi

if [ -e "/opt/ros/humble/setup.bash" ]; then
    . "/opt/ros/humble/setup.bash"
fi

cd /ros2_rust_ws || exit

colcon build --cargo-args --features rclrs

. "/ros2_rust_ws/install/local_setup.bash"

ros2 run ros_pointcloud2_tests rclrs_tests

"$@"
