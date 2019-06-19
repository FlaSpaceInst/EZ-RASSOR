#!/bin/sh
# A collection of functions that aid development of this repository.
# Written by Tiger Sachse.

PACKAGES_DIR="packages"
EXTERNAL_DIR="external"
WORKSPACE_DIR="$HOME/.workspace"
WORKSPACE_SOURCE_DIR="$WORKSPACE_DIR/src"
CONTRIBUTING_FILE="docs/CONTRIBUTING.rst"
USAGE_STRING="Usage: sh develop.sh <mode> [arguments...]\n"

# Set up the development environment.
setup_environment() {
    rm -r -f "$WORKSPACE_DIR"
    mkdir -p "$WORKSPACE_SOURCE_DIR"
    cd "$WORKSPACE_SOURCE_DIR"
    catkin_init_workspace
    cd - > /dev/null 2>&1
}

# Create a new ROS package in source control.
new_package() {
    package="$1"
    mkdir -p "$PACKAGES_DIR/$package"
    cd "$PACKAGES_DIR/$package"

    # Create a new catkin package with all the arguments
    # passed to this function (after the first argument).
    shift
    catkin_create_pkg "$@"

    cd - > /dev/null 2>&1
}

# Link packages into your workspace.
link_packages() {
    link_only_in_list=false
    link_except_in_list=false
    if [ $# -ne 0 ]; then
        case "$1" in
            "-o"|"--only")
                link_only_in_list=true
                shift
                ;;
            "-e"|"--except")
                link_except_in_list=true
                shift
                ;;
            *)
                echo "$USAGE_STRING"
                exit 1
                ;;
        esac
    fi

    for collection_dir in "$PWD/$PACKAGES_DIR" "$PWD/$EXTERNAL_DIR"; do
        for superpackage_dir in "$collection_dir"/*; do
            for package_dir in "$superpackage_dir"/*; do
                if [ ! -d "$package_dir" ]; then
                    :
                elif [ "$link_only_in_list" = "true" ]; then
                    if argument_in_list "$(basename "$package_dir")" "$@"; then
                        link_package "$package_dir"
                    fi
                elif [ "$link_except_in_list" = "true" ]; then
                    if ! argument_in_list "$(basename "$package_dir")" "$@"; then
                        link_package "$package_dir"
                    fi
                else
                    link_package "$package_dir"
                fi
            done
        done
    done
}

# Helper function that links a single package.
link_package() {
    package_path="$1"
    package_name="$(basename "$package_path")"
    if [ -L "$WORKSPACE_SOURCE_DIR/$package_name" ]; then
        rm -f "$WORKSPACE_SOURCE_DIR/$package_name"
        printf "Relinking %s...\n" "$package_path"
    else
        printf "Linking %s...\n" "$package_path"
    fi
    ln -s "$package_path" "$WORKSPACE_SOURCE_DIR/$package_name"
}

# Check if the first argument exists in the remaining list of arguments.
argument_in_list() {
    argument="$1"
    shift
    for item in "$@"; do
        if [ "$argument" = "$item" ]; then
            return 0
        fi
    done
   
    return 1
}

# Purge packages in the ROS workspace.
purge_packages() {
    cd "$WORKSPACE_SOURCE_DIR"
    printf "Purging all packages in /src...\n"
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

# Test all packages in the workspace.
test_packages() {
    cd "$WORKSPACE_DIR"
    catkin_make run_tests
    cd - > /dev/null 2>&1
}

# Show a help menu. This menu is parsed from pre-existing documentation.
throw_help() {
    printf "$USAGE_STRING"
    cat "$CONTRIBUTING_FILE" \
        | grep '^``' -A 1 \
        | sed 's/  //g' \
        | sed 's/^``/#/g' \
        | fold -s -w 75 \
        | sed '/^#/! s/^/   /g' \
        | sed '/``/ s/``/"/g' \
        | sed '/^#/ s/"//g' \
        | tr '#' '\n'
}

# Main entry point of the script.
case "$1" in
    "setup")
        setup_environment
        ;;
    "new")
        shift
        new_package "$@"
        ;;
    "link")
        shift
        link_packages "$@"
        ;;
    "purge")
        purge_packages
        ;;
    "relink")
        shift
        purge_packages
        link_packages "$@"
        ;;
    "build")
        build_packages
        ;;
    "install")
        install_packages
        ;;
    "kill")
        kill_ros
        ;;
    "test")
        test_packages
        ;;
    "help")
        throw_help
        ;;
    *)
        printf "$USAGE_STRING"
        ;;
esac
