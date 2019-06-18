#!/bin/sh
#
#

# A collection of necessary constants for this script.
USER_SHELLS="bash zsh"
SH_SETUP_FILE="setup.sh"
KEY_SERVER="hkp://ha.pool.sks-keyservers.net:80"
RECV_KEY="C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654"
EXTERNALS_DIR="external"
SUPERPACKAGES_DIR="packages"
ROS_PARTIAL_INSTALL_DIR="/opt/ros"
MOCK_INSTALL_RELATIVE_DIR="install"
WORKSPACE_SOURCE_RELATIVE_DIR="src"
EZRASSOR_INSTALL_DIR="$HOME/.ezrassor"
WORKSPACE_PARTIAL_DIR="/tmp/ezrassor_workspace"
CATKIN_MAKE_ISOLATED_BIN="$WORKSPACE_SOURCE_RELATIVE_DIR/catkin/bin/catkin_make_isolated"

# Throw a help message at the user.
throw_help() {
    printf "Usage: sh install.sh <mode> [--flags]\n"
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
                printf "Previously sourced!\n"
            else
                printf "%s\n" \
                       "" \
                       "# Source a ROS setup file, if it exists." \
                       "if [ -f \"$source_file\" ]; then" \
                       "    $source_line" \
                       "fi" >> "$shellrc_file"
                printf "Successfully sourced!\n"
                must_restart=true
            fi
        fi
    done

    if [ "$must_restart" = "true" ]; then
        printf "\n\n******** %s ********\n" \
               "Restart your terminal for changes to take effect."
    fi
}

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

# Install ROS automatically with APT.
install_ros() {
    require "sudo" "apt" "apt-key"
    os_version="$1"
    ros_version="$2"

    # Add the correct repository key to APT for ROS.
    echo_command="echo \"deb http://packages.ros.org/ros/ubuntu $os_version main\""
    ros_latest_dir="/etc/apt/sources.list.d/ros-latest.list"
    sudo sh -c "$echo_command > $ros_latest_dir"
    sudo apt-key adv --keyserver "$KEY_SERVER" --recv-key "$RECV_KEY"
    sudo apt update

    # Install ROS and initialize rosdep.
    sudo apt install -y "ros-${ros_version}-ros-base" python-rosdep
    set +e
    sudo rosdep init
    set -e
    rosdep update

    # Source the ROS installation.
    source_setups_in_directory "$ROS_PARTIAL_INSTALL_DIR/$ros_version"
}

# Install some build tools required to build the EZ-RASSOR packages.
install_build_tools() {
    require "sudo" "apt"
    sudo apt install -y python-rosdep python-pip build-essential
    set +e
    sudo rosdep init
    set -e
    rosdep update
}

# Install EZ-RASSOR packages from source.
install_ezrassor_packages() {
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

    # Source the installed EZ-RASSOR packages.
    source_setups_in_directory "$EZRASSOR_INSTALL_DIR" 
}

# The main entry point of this script.
# Throw a message if the script quits early, and tell the script to quit after
# any non-zero error message.
trap 'throw_error "Something went horribly wrong!"' 0
set -e

require "lsb_release"
os_version="$(lsb_release -sc)"
if [ "$os_version" = "xenial" ]; then
    ros_version="kinetic"
elif [ "$os_version" = "bionic" ]; then
    ros_version="melodic"
else
    throw_error "This script can only automatically install ROS for" \
                "Ubuntu Xenial and Ubuntu Bionic. Your operating system" \
                "is not supported. :( The ROS wiki may have instructions" \
                "that will help you install ROS on your system."
fi

case "$1" in
    "ros")
        install_ros "$os_version" "$ros_version"
        ;;
    "build-tools")
        install_build_tools
        ;;
    "packages")
        shift
        install_ezrassor_packages "$@"
        ;;
    *)
        throw_help
        ;;
esac

# Allow the script to exit normally, without an error messsage.
trap ":" 0
