# !/bin/bash

echo "╔══╣ Setup: SOBITS TELEOP (STARTING) ╠══╗"


# Keep track of the current directory
DIR=`pwd`
cd ..

# Download required packages
ros_packages=(
    "keyboard_joy"
    "ros_tcp_endpoint"
    "sobits_interfaces"
    "sobits_robot_descriptor"
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

# Every ROS dependency is declared in the package.xml files and comes from
# rosdep; jstest-gtk is the only apt package here (it has no rosdep key).
sudo apt-get update
sudo apt-get install -y jstest-gtk

# Rosdep also resolves the cloned sibling packages' dependencies
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update
rosdep install -r -y -i --from-paths ${DIR}
for pkg in "${ros_packages[@]}"; do
    rosdep install -r -y -i --from-paths ${DIR}/../${pkg}
done

# Download ds4drv for dualshock 4
sudo pip install ds4drv --break-system-packages
sudo apt update
sudo apt install bluez -y
PYTHON_VER=$(python3 -c "import sys; print(f'python{sys.version_info.major}.{sys.version_info.minor}')")
sudo sed -i 's/SafeConfigParser/ConfigParser/g' /usr/local/lib/${PYTHON_VER}/dist-packages/ds4drv/config.py
sudo pip install evdev==1.8.0 --break-system-packages


# Download adb for android controller support
sudo apt install adb -y
sudo usermod -aG plugdev $USERNAME
cat <<'EOF' | sudo tee /etc/udev/rules.d/50-oculus.rules
SUBSYSTEM=="usb", ATTR{idVendor}=="2833", ATTR{idProduct}=="5013", MODE="0666", GROUP="plugdev", SYMLINK+="oculus%n"
SUBSYSTEM=="usb", ATTR{idVendor}=="2833", ATTR{idProduct}=="5012", MODE="0666", GROUP="plugdev", SYMLINK+="oculus%n"
SUBSYSTEM=="usb", ATTR{idVendor}=="2833", ATTR{idProduct}=="5011", MODE="0666", GROUP="plugdev", SYMLINK+="oculus%n"
SUBSYSTEM=="usb", ATTR{idVendor}=="2833", ATTR{idProduct}=="5010", MODE="0666", GROUP="plugdev", SYMLINK+="oculus%n"
EOF
sudo udevadm control --reload-rules
sudo udevadm trigger

# Install udev rules for ds4drv (/dev/uinput write access)
sudo curl -fsSL https://raw.githubusercontent.com/chrippa/ds4drv/master/udev/50-ds4drv.rules \
    -o /etc/udev/rules.d/50-ds4drv.rules
sudo udevadm control --reload-rules
sudo udevadm trigger


# Go back to previous directory
cd ${DIR}

echo "╚══╣ Setup: SOBITS TELEOP (FINISHED) ╠══╝"
