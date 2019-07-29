#!/bin/sh
# This script can install ROS, ROS build tools, and EZ-RASSOR components on systems
# running Ubuntu Xenial and Ubuntu Bionic.
# Written by Tiger Sachse.

USER_SHELLS="bash zsh"
SH_SETUP_FILE="setup.sh"
README_FILE="docs/README.rst"
CHANGES_FILE="docs/CHANGES.txt"
LICENSE_FILE="docs/LICENSE.txt"
EXTERNALS_DIR="external"
SUPERPACKAGES_DIR="packages"
ROS_INSTALL_PARTIAL_DIR="/opt/ros"
WORKSPACE_SOURCE_RELATIVE_DIR="src"
EZRASSOR_INSTALL_DIR="$HOME/.ezrassor"
INSTALLED_DOCS_DIR="$EZRASSOR_INSTALL_DIR/docs"
WORKSPACE_PARTIAL_DIR="/tmp/ezrassor_workspace"
RECV_KEY="C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654"
USAGE_STRING="Usage: sh install.sh <software> [arguments...]\n"
CATKIN_MAKE_ISOLATED_BIN="$WORKSPACE_SOURCE_RELATIVE_DIR/catkin/bin/catkin_make_isolated"

# Throw a help message at the user.
throw_help() {
    printf "$USAGE_STRING"
    cat "$README_FILE" \
        | grep '^``' -A 1 \
        | sed 's/  //g' \
        | sed 's/^``/#/g' \
        | fold -s -w 75 \
        | sed '/^#/! s/^/   /g' \
        | sed '/``/ s/``/"/g' \
        | sed '/^#/ s/"//g' \
        | sed 's/**//g' \
        | tr '#' '\n'
}

# Print an error message and exit this script.
throw_error() {
    printf "%s\n" "$@"
    trap ":" 0
    exit 1
}

# If required commands don't exist, throw an error.
require() {
    missing_requirement=false
    for requirement in "$@"; do
        set +e
        command -v "$requirement" > /dev/null 2>&1
        if [ $? -ne 0 ]; then
            printf "Required but not installed: $requirement\n"
            missing_requirement=true
        fi
        set -e
    done
    if [ "$missing_requirement" = "true" ]; then
        throw_error "Please install all missing components before proceeding. Aborting..."
    fi
}

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

# Check if the first argument exists in the remaining list of arguments.
argument_in_list() {
    argument="$1"
    shift
    for item in "$@"; do
        case "$argument" in
            $item)
                return 0
        esac
    done

    return 1
}

# Install ROS automatically with APT.
install_ros() {
    require "sudo" "apt" "apt-key"
    os_version="$1"
    ros_version="$2"
    key_server="$3"

    # Add the correct repository key to APT for ROS.
    echo_command="echo \"deb http://packages.ros.org/ros/ubuntu $os_version main\""
    ros_latest_dir="/etc/apt/sources.list.d/ros-latest.list"
    sudo sh -c "$echo_command > $ros_latest_dir"
    sudo apt-key adv --keyserver "$key_server" --recv-key "$RECV_KEY"
    sudo apt update

    # Install ROS and initialize rosdep.
    sudo apt install -y "ros-${ros_version}-ros-base" python-rosdep
    set +e
    sudo rosdep init
    set -e
    rosdep update

    # Source the ROS installation.
    source_setups_in_directory "$ROS_INSTALL_PARTIAL_DIR/$ros_version"
}

# Install build tools required to build EZ-RASSOR packages.
install_ros_tools() {
    require "sudo" "apt"
    sudo apt install -y python-rosdep python-pip build-essential
    set +e
    sudo rosdep init
    set -e
    rosdep update
}

# Install build tools required to build the EZ-RASSOR controller.
install_app_tools() {
    require "sudo" "apt"
    sudo apt install -y curl software-properties-common
    curl -sL https://deb.nodesource.com/setup_12.x | sudo -E bash -
    sudo apt install nodejs
    sudo npm install expo-cli --global
}

# Install EZ-RASSOR packages from source.
install_packages() {
    require "catkin_make" "rosdep" "pip"

    # Create a temporary workspace.
    workspace_dir="${WORKSPACE_PARTIAL_DIR}_$(date +%s)"
    workspace_source_dir="$workspace_dir/$WORKSPACE_SOURCE_RELATIVE_DIR"
    mkdir -p "$workspace_source_dir"

    # Determine if the user wants to exclude or include only certain packages.
    link_only_in_list=false
    link_except_in_list=false
    if [ $# -gt 1 ]; then
        case "$1" in
            "-o"|"--only")
                link_only_in_list=true
                shift
                ;;
            "-e"|"--except")
                link_except_in_list=true
                shift
                ;;
        esac
    fi

    # Link all of the packages that the user desires.
    for collection_dir in "$PWD/$EXTERNALS_DIR" "$PWD/$SUPERPACKAGES_DIR"; do
        for superpackage_dir in "$collection_dir"/*; do
            for package_dir in "$superpackage_dir"/*; do
                if [ ! -d "$package_dir" ]; then
                    :
                elif [ "$link_only_in_list" = "true" ]; then
                    if argument_in_list "$(basename "$package_dir")" "$@"; then
                        ln -s -f "$package_dir" "$workspace_source_dir"
                    fi
                elif [ "$link_except_in_list" = "true" ]; then
                    if ! argument_in_list "$(basename "$package_dir")" "$@"; then
                        ln -s -f "$package_dir" "$workspace_source_dir"
                    fi
                else
                    ln -s -f "$package_dir" "$workspace_source_dir"
                fi
            done
        done
    done

    # Install all of the dependencies of the linked packages in the temporary
    # workspace.
    cd "$workspace_dir"
    rosdep install --from-paths "$WORKSPACE_SOURCE_RELATIVE_DIR" \
                   --ignore-src \
                   -y

    # Build and install the linked packages into the EZRASSOR_INSTALL_DIR.
    catkin_make
    mkdir -p "$EZRASSOR_INSTALL_DIR"
    catkin_make -DCMAKE_INSTALL_PREFIX="$EZRASSOR_INSTALL_DIR" install

    cd - > /dev/null 2>&1

    # Copy the most important documentation into the EZRASSOR_INSTALL_DIR.
    mkdir -p "$INSTALLED_DOCS_DIR"
    cp "$README_FILE" "$INSTALLED_DOCS_DIR"
    cp "$CHANGES_FILE" "$INSTALLED_DOCS_DIR"
    cp "$LICENSE_FILE" "$INSTALLED_DOCS_DIR"

    # Source the installed EZ-RASSOR packages.
    source_setups_in_directory "$EZRASSOR_INSTALL_DIR"
}

# THE SCRIPT BEGINS HERE.
# Throw a message if the script quits early, and tell the script to quit after
# any non-zero error message.
trap 'throw_error "Something went horribly wrong."' 0
set -e

# Set the OS version, ROS version, and necessary key server (assuming the user
# is running a supported operating system).
require "lsb_release"
os_version="$(lsb_release -sc)"
if [ "$os_version" = "xenial" ]; then
    ros_version="kinetic"
    key_server="hkp://ha.pool.sks-keyservers.net:80"
elif [ "$os_version" = "bionic" ]; then
    ros_version="melodic"
    key_server="hkp://keyserver.ubuntu.com:80"
else
    throw_error "This script can only automatically install ROS for Ubuntu Xenial" \
                "and Ubuntu Bionic. Your operating system is not supported. :(" \
                "You may be able to find ROS installation instructions for your" \
                "operating system on the ROS wiki."
fi

# Install the appropriate software based on the user's first argument to this script.
case "$1" in
    "ros")
        install_ros "$os_version" "$ros_version" "$key_server"
        ;;
    "tools"|"ros-tools") # Remove the first tag once #247 is ready to go
        install_ros_tools
        ;;
    "app-tools")
        install_app_tools
        ;;
    "packages")
        shift
        install_packages "$@"
        ;;
    "help")
        throw_help
        ;;
    *)
        printf "$USAGE_STRING"
        ;;
esac

# If the script makes it this far, exit normally without an error messsage.
trap ":" 0
