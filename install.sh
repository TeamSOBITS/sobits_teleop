#!/bin/bash

echo "╔══╣ Setup: SOBITS TELEOP (STARTING) ╠══╗"


# Keep track of the current directory
DIR=`pwd`
cd ..

# Download required packages
ros_packages=(
    "keyboard_joy"
    "ros_tcp_endpoint"
)

#Clone all packages
for ((i = 0; i < ${#ros_packages[@]}; i++)) {
    echo "Clonning: ${ros_packages[i]}"
    git clone --recurse-submodules -b $ROS_DISTRO-devel https://github.com/TeamSOBITS/${ros_packages[i]}.git

    # Check if install.sh exists in each package
    if [ -f ${ros_packages[i]}/install.sh ]; then
        echo "Running install.sh in ${ros_packages[i]}."
        cd ${ros_packages[i]}
        bash install.sh
        cd ..
    fi
}

# Download ROS packages
sudo apt-get update
sudo apt-get install -y \
    jstest-gtk \
    ros-$ROS_DISTRO-joy-linux

# Go back to previous directory
cd ${DIR}

echo "╚══╣ Setup: SOBITS TELEOP (FINISHED) ╠══╝"