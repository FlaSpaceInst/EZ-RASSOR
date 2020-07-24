#!/bin/sh
# A collection of functions that aid development of this repository.
# Written by Tiger Sachse.

USER_SHELLS="bash zsh"
PACKAGES_DIR="packages"
WORKSPACE_DIR="$HOME/ezrassor_ws"
WORKSPACE_SOURCE_DIR="$WORKSPACE_DIR/src"
WORKSPACE_DEVEL_DIR="$WORKSPACE_DIR/devel"
PACKAGE_XML_FILE="package.xml"
CONTRIBUTING_FILE="docs/CONTRIBUTING.rst"
USAGE_STRING="Usage: sh develop.sh <mode> [arguments...]\n"

# Source setup files within a given directory in the user's RC files.
source_setups_in_directory() {
    must_restart=false
    partial_source_file="$1/setup"
    for user_shell in $USER_SHELLS; do
        shellrc_file="$HOME/.${user_shell}rc"
        if [ -f "$shellrc_file" ]; then
            source_file="$partial_source_file.$user_shell"
            source_line=". \"$source_file\""

            printf "Attempting to source setup script for %s: " "$user_shell"
            if cat "$shellrc_file" | grep -Fq "$source_line"; then
                printf "Previously sourced.\n"
            else
                printf "%s\n" \
                       "" \
                       "# Source a ROS setup file, if it exists." \
                       "if [ -f \"$source_file\" ]; then" \
                       "    $source_line" \
                       "fi" >> "$shellrc_file"
                printf "Successfully sourced.\n"
                must_restart=true
            fi
        fi
    done

    if [ "$must_restart" = "true" ]; then
        printf "\n\n******** %s ********\n" \
               "Restart your terminal for changes to take effect."
    fi
}

# Set up the development environment.
setup_environment() {

    # Setup will run fresh every time.
    rm -r -f "$WORKSPACE_DIR"
    mkdir -p "$WORKSPACE_SOURCE_DIR"

    cd "$WORKSPACE_SOURCE_DIR"
    catkin_init_workspace
    cd - > /dev/null 2>&1

    source_setups_in_directory "$WORKSPACE_DEVEL_DIR"
    mkdir -p ~/.gazebo/models
    cp -r extra_worlds/* ~/.gazebo/models/
    cp -r extra_models/* ~/.gazebo/models/
}

# Create a new ROS package in source control.
new_package() {
    superpackage="$1"
    mkdir -p "$PACKAGES_DIR/$superpackage"
    cd "$PACKAGES_DIR/$superpackage"

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

    for collection_dir in "$PWD/$PACKAGES_DIR"; do
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
        case "$argument" in
            $item)
                return 0
                ;;
        esac
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

# Install dependencies for all linked packages.
resolve_packages() {
    rosdep install --from-paths "$WORKSPACE_SOURCE_DIR" --ignore-src -y
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

    # This command will run the tests but return a status of 0.
    catkin_make run_tests

    # This command will return a status code of 0 or 1 depending on if the previous tests succeeded.
    (catkin_test_results)
    local result=$?

    # After we return to the main directory, return the status code from the test results.
    cd - > /dev/null 2>&1
    [ $result -eq 0 ]
}

# Change the version number of all the packages.
reversion_packages() {
    sed_command="s|<version>.*</version>|<version>$1</version>|g"
    for superpackage_dir in "$PWD/$PACKAGES_DIR"/*; do
        for package_dir in "$superpackage_dir"/*; do
            for item in "$package_dir"/*; do
                if [ "$(basename "$item")" = "$PACKAGE_XML_FILE" ]; then
                    sed -i "$sed_command" "$item"
                fi
            done
        done
    done
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
    "resolve")
        resolve_packages
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
    "reversion")
        reversion_packages "$2"
        ;;
    "help")
        throw_help
        ;;
    *)
        printf "$USAGE_STRING"
        ;;
esac
