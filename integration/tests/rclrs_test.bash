#!/bin/bash

. "$HOME/.cargo/env"

# run rustup to test with latest rust version
rustup update

if [ -e "/opt/ros/jazzy/setup.bash" ]; then
    . "/opt/ros/jazzy/setup.bash"
fi

if [ -e "/opt/ros/iron/setup.bash" ]; then
    . "/opt/ros/iron/setup.bash"
fi

if [ -e "/opt/ros/humble/setup.bash" ]; then
    . "/opt/ros/humble/setup.bash"
fi

. "/ros2_rust_build/install/local_setup.bash"
cd /ros2_rust_build/src/ros_pointcloud2_tests/ || exit

"$@"
