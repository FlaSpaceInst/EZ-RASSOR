#!/bin/sh
# A collection of functions to make development of this repository easier.
# Written by Tiger Sachse.

WORKSPACE_DIR="$HOME/.workspace"
SOURCE_DIR="$WORKSPACE_DIR/src"
PACKAGES_DIR="packages"
EXTERNAL_DIR="external"

# Set up the development environment.
setup_environment() {
    rm -rf "$WORKSPACE_DIR"
    mkdir -p "$SOURCE_DIR"
    cd "$SOURCE_DIR"
    catkin_init_workspace
    cd - > /dev/null 2>&1
}

# Create a new ROS package in source control.
new_package() {
    mkdir -p "$PACKAGES_DIR/$1"
    cd "$PACKAGES_DIR/$1"

    # Create a new catkin package with all the arguments
    # passed to this function (after the first argument).
    shift
    catkin_create_pkg "$@"

    cd - > /dev/null 2>&1
}

# Link packages into your workspace.
link_packages() {
    LINK_ONLY_IN_LIST=false
    LINK_EXCEPT_IN_LIST=false
    case "$1" in
        -o|--only)
            LINK_ONLY_IN_LIST=true
            shift
            ;;
        -e|--except)
            LINK_EXCEPT_IN_LIST=true
            shift
            ;;
    esac

    for TARGET_DIR in "$PACKAGES_DIR" "$EXTERNAL_DIR"; do
        for SUPERPACKAGE_DIR in "$PWD/$TARGET_DIR"/*; do
            for PACKAGE_DIR in "$SUPERPACKAGE_DIR"/*; do
                if [ ! -d "$PACKAGE_DIR" ]; then
                    :
                elif [ "$LINK_ONLY_IN_LIST" = "true" ]; then
                    if argument_in_list "$(basename "$PACKAGE_DIR")" "$@"; then
                        link_package "$PACKAGE_DIR"
                    fi
                elif [ "$LINK_EXCEPT_IN_LIST" = "true" ]; then
                    if ! argument_in_list "$(basename "$PACKAGE_DIR")" "$@"; then
                        link_package "$PACKAGE_DIR"
                    fi
                else
                    link_package "$PACKAGE_DIR"
                fi
            done
        done
    done
}

# Helper function that links a single package.
link_package() {
    PACKAGE_PATH="$1"
    PACKAGE_NAME="$(basename "$PACKAGE_PATH")"
    if [ -L "$SOURCE_DIR/$PACKAGE_NAME" ]; then
        rm -f "$SOURCE_DIR/$PACKAGE_NAME"
        printf "Relinking %s...\n" "$PACKAGE_PATH"
    else
        printf "Linking %s...\n" "$PACKAGE_PATH"
    fi
    ln -s "$PACKAGE_PATH" "$SOURCE_DIR/$PACKAGE_NAME"
}

# Helper function that determines if an argument is in a list.
argument_in_list() {
    TARGET="$1"
    shift
    for ARGUMENT in "$@"; do
        if [ "$TARGET" = "$ARGUMENT" ]; then
            return 0
        fi
    done
   
    return 1
}

# Purge packages in the ROS workspace.
purge_packages() {
    cd "$SOURCE_DIR"
    echo "Purging all packages in /src..."
    find . ! -name "CMakeLists.txt" -type l -exec rm -f {} +
    cd - > /dev/null 2>&1
}

# Build all packages.
build_packages() {
    cd "$WORKSPACE_DIR"
    catkin_make
    cd - > /dev/null 2>&1
}

# Install all packages that have been built.
install_packages() {
    cd "$WORKSPACE_DIR"
    catkin_make install
    cd - > /dev/null 2>&1
}

# Kill all running ROS nodes.
kill_ros() {
    rosnode kill --all
    killall -SIGTERM roscore
    printf "\nROS has been shut down.\n"
}

# Main entry point of the script.
case $1 in
    setup)
        setup_environment
        ;;
    new)
        shift
        new_package "$@"
        ;;
    link)
        shift
        link_packages "$@"
        ;;
    purge)
        purge_packages
        ;;
    relink)
        shift
        purge_packages
        link_packages "$@"
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
esac
