#!/bin/sh
# A collection of functions to make development of this repository easier.
# Written by Tiger Sachse.

SUPERPACKAGE_DIR="packages"
WORKSPACE_DIR="$HOME/.workspace"
SOURCE_DIR="$WORKSPACE_DIR/src"

# Set up the development environment.
setup_environment() {
    rm -rf "$WORKSPACE_DIR"
    mkdir -p "$SOURCE_DIR"
    cd "$SOURCE_DIR"
    catkin_init_workspace
}

# Kill all running ROS nodes.
kill_ros() {
    rosnode kill --all
    killall -SIGTERM roscore
    printf "\nROS has been shut down.\n"
}

# Build all packages.
build_packages() {
    cd "$WORKSPACE_DIR"
    catkin_make
}

# Install all packages that have been built.
install_packages() {
    cd "$WORKSPACE_DIR"
    catkin_make install
}

# Create a new ROS package in source control.
new_package() {
    mkdir -p "$SUPERPACKAGE_DIR/$1"
    cd "$SUPERPACKAGE_DIR/$1"

    # Create a new catkin package with all the arguments
    # passed to this function (after the first argument).
    catkin_create_pkg "${@:2}"
}

# Purge packages in the ROS workspace.
purge_packages() {
    cd "$SOURCE_DIR"
    echo "Purging all packages in /src..."
    find . ! -name "CMakeLists.txt" -type l -exec rm -f {} +
}

# Link packages into your workspace.
link_packages() {
    cd "$SUPERPACKAGE_DIR"

    for SUPERPACKAGE_DIR in *; do
        cd "$SUPERPACKAGE_DIR"
        for PACKAGE_DIR in *; do
            case "$1" in
                -o|--only)
                    if argument_in_list "$PACKAGE_DIR" "${@:2}"; then
                        link_package "$PACKAGE_DIR"
                    fi
                    ;;
                -e|--except)
                    if ! argument_in_list "$PACKAGE_DIR" "${@:2}"; then
                        link_package "$PACKAGE_DIR"
                    fi
                    ;;
                *)
                    link_package "$PACKAGE_DIR"
                    ;;
            esac
        done
        cd ..
    done
}

# Helper function that links a single package.
link_package() {
    if [ -L "$SOURCE_DIR/$1" ]; then
        rm -f "$SOURCE_DIR/$1"
        printf "Relinking %s...\n" "$PWD/$1"
    else
        printf "Linking %s...\n" "$PWD/$1"
    fi
    ln -s "$PWD/$1" "$SOURCE_DIR/$1"
}

# Helper function that determines if an argument is in a list.
argument_in_list() {
    for ARGUMENT in "${@:2}"; do
        if [ "$1" = "$ARGUMENT" ]; then
            return 0
        fi
    done
   
    return 1
}

# Main entry point of the script.
. /opt/ros/kinetic/setup.sh
case $1 in
    setup)
        setup_environment
        ;;
    new)
        new_package "${@:2}"
        ;;
    link)
        link_packages "${@:2}"
        ;;
    build)
        build_packages
        ;;
    install)
        install_packages
        ;;
    kill)
        kill_ros
        ;;
    purge)
        purge_packages
        ;;
    relink)
        purge_packages
        link_packages "${@:2}"
        ;;
esac
