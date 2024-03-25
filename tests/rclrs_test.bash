#!/bin/bash

. "$HOME/.cargo/env"

# run rustup to test with latest rust version
rustup update

if [ -e "/opt/ros/iron/setup.bash" ]; then
    . "/opt/ros/iron/setup.bash"
    . "/ros2_rust_build/install/local_setup.bash"
fi

if [ -e "/opt/ros/humble/setup.bash" ]; then
    . "/opt/ros/humble/setup.bash"
    . "/ros2_rust_build/install/local_setup.bash"
fi

if [ -e "/opt/ros/galactic/setup.bash" ]; then
    . "/opt/ros/galactic/setup.bash"
    . "/ros2_rust_build/install/local_setup.bash"
fi

cd /ros2_rust_build/src/ros_pointcloud2_tests/ || exit

"$@"
