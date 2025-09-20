#!/bin/zsh

# Built Script - For Building the ROS application

# This Script runs using ZSH!
# It can be changed to bash by modifying the top line

# PKGS_COMMON=("joystick_controller" "interfaces")
PKGS_COMMON=("interfaces" "controllers")
PKGS_JETSON=()

HOSTNAME_JETSON="jetson"
HOSTNAME=$(hostname)

build() {
    if [[ $# -eq 0 ]]; then
        echo "Build must be called with a list of packages"
        return 1
    fi

    if [[ "$HOSTNAME" == "$HOSTNAME_RASBPI" ]]; then
        export CC=clang-15
        export CXX=clang++-15
    else
        export CC=clang
        export CXX=clang++
    fi


    # Build the packages
    colcon build --packages-select "$@" --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Debug

    # Source Install
    if [[ -f "./install/local_setup.zsh" ]]; then
        source ./install/local_setup.zsh
    fi

    # Symlink the compile commands
    if [[ -f "./build/compile_commands.json" ]]; then
        ln -sf ./build/compile_commands.json ./
    fi
}


if [[ "$HOSTNAME" == "$HOSTNAME_JETSON" ]]; then
    echo "Building ROS Packages for Jetson"
    build "${PKGS_COMMON[@]}" "${PKGS_JETSON[@]}"

elif [[ "$HOSTNAME" == "$HOSTNAME_RASBPI" ]]; then
    echo "Building ROS Packages for Raspberry Pi"
    build "${PKGS_COMMON[@]}" "${PKGS_RASBPI[@]}"

else
    echo "Building ROS Workspace Common Packages"
    build "${PKGS_COMMON[@]}"
fi

