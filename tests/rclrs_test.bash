#!/bin/bash

. "$HOME/.cargo/env"

# run rustup to test with latest rust version
rustup update

if [ -e "/opt/ros/iron/setup.bash" ]; then
    source "/opt/ros/iron/setup.bash"
    source "/ros2_rust_build/install/local_setup.bash"
fi

if [ -e "/opt/ros/humble/setup.bash" ]; then
    source "/opt/ros/humble/setup.bash"
    source "/ros2_rust_build/install/local_setup.bash"
fi

if [ -e "/opt/ros/galactic/setup.bash" ]; then
    source "/opt/ros/galactic/setup.bash"
    source "/ros2_rust_build/install/local_setup.bash"
fi

cd /rclrs/ || exit

"$@"
