#!/bin/sh
#
#

throw_help() {
    printf "Usage: sh install.sh <mode> [--flags]"
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
        set -e
        if [ $? -ne 0 ]; then
            printf "Required but not installed: $requirement\n"
            missing_requirement=true
        fi
    done
    if [ "$missing_requirement" = "true" ]; then
        throw_error "Please install all missing components before proceeding. Aborting..."
    fi
}

# Add the ROS repositories to APT.
add_ros_repository() {
    os_version="$1"
    require "sudo" "apt" "apt-key"
    echo_command="echo \"deb http://packages.ros.org/ros/ubuntu $os_version main\""
    ros_latest_dir="/etc/apt/sources.list.d/ros-latest.list"
    sudo sh -c "$echo_command > $ros_latest_dir"
    sudo apt-key adv --keyserver "$KEY_SERVER" --recv-key "$RECV_KEY"
    sudo apt update
}

# Install buildtools for ROS.
install_buildtools() {
    require "sudo" "apt"
    sudo apt install -y python-pip \
                        python-rosdep \
                        python-rosinstall-generator \
                        python-wstool \
                        python-rosinstall \
                        build-essential
    set +e
    sudo rosdep init
    set -e
    rosdep update
}

# Source setup files within a given directory in the user's RC files.
source_setups_in_directory() {
    must_restart=false
    partial_source_file="$1/setup"
    for user_shell in $USER_SHELLS; do
        shellrc_file="$HOME/.${user_shell}rc"
        if [ -f "$shellrc_file" ]; then
            source_file="$partial_source_file.$user_shell"
            source_line=". $source_file"

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

    if [ "$must_restart" = true ]; then
        printf "\n\n******** %s ********\n" \
               "Restart your terminal for changes to take effect."
    fi
}

# Install ROS automatically with APT.
install_ros_automatically() {
    ros_version="$1"
    automatic_ros_install_dir="$2"
    require "sudo" "apt"
    add_ros_repository
    sudo apt install -y "ros-${ros_version}-ros-base"
    set +e
    sudo rosdep init
    set -e
    rosdep update
    source_setups_in_directory "$automatic_ros_install_dir" 
}

# Install ROS manually.
install_ros_manually() {
    ros_version="$1"
    require "wstool" "rosdep" "rosinstall" "rosinstall_generator" "cmake"
    
    # Create a temporary workspace.
    workspace_dir="${WORKSPACE_PARTIAL_DIR}_$(date +%s)"
    workspace_source_dir="$workspace_dir/$WORKSPACE_SOURCE_RELATIVE_DIR"
    mkdir -p "$workspace_source_dir"
    cd "$workspace_dir"

    # Define the rosinstall_generator flags. Kinetic needs the --wet-only flag
    # for some reason (per the wiki).
    rosinstall_generator_flags="--rosdistro $ros_version --deps --tar"
    if [ "$ros_version" = "kinetic" ]; then
        rosinstall_generator_flags="$rosinstall_generator_flags --wet-only"
    fi

    # Download, build, and install all of the ROS communication core packages.
    rosinstall_generator ros_comm $rosinstall_generator_flags > "$ROSINSTALL_FILE"
    wstool init -j8 "$WORKSPACE_SOURCE_RELATIVE_DIR" "$ROSINSTALL_FILE"
    rosdep install --from-paths "$WORKSPACE_SOURCE_RELATIVE_DIR" \
                   --ignore-src \
                   --yes
    ./"$CATKIN_MAKE_ISOLATED_BIN" --install \
                                   -DCMAKE_BUILD_TYPE=Release
                                   --install-space="$MANUAL_ROS_INSTALL_DIR"
    source_setups_in_directory "$MANUAL_ROS_INSTALL_DIR"
}

# Install only EZ-RASSOR packages.
install_ezrassor_packages() {
    require "pip" "rosdep" "catkin_make"

        link_only_in_list=false
        link_except_in_list=false
        if [ $# -gt 1 ]; then
            case "$2" in
                -o|--only)
                    link_only_in_list=true
                    shift
                    ;;
                -e|--except)
                    link_except_in_list=true
                    shift
                    ;;
            esac
        for ARGUMENT in "$@"; do


        done
    # Create a temporary workspace.
    workspace_dir="${WORKSPACE_PARTIAL_DIR}_$(date +%s)"
    workspace_source_dir="$workspace_dir/$WORKSPACE_SOURCE_RELATIVE_DIR"
    mkdir -p "$workspace_source_dir"

    # Link packages into the temporary workspace based on the INSTALL flags. These
    # flags are set at the beginning of the script.
    if [ "$INSTALL_AUTONOMY" = "true" ]; then
        SUPERPACKAGE="$PWD/$EXTERNALS_DIR/viso2"
        ln -s -f "$SUPERPACKAGE/viso2" "$workspace_source_dir" 
        ln -s -f "$SUPERPACKAGE/libviso2" "$workspace_source_dir"
        ln -s -f "$SUPERPACKAGE/viso2_ros" "$workspace_source_dir"
        SUPERPACKAGE="$PWD/$SUPERPACKAGES_DIR/autonomy"
        ln -s -f "$SUPERPACKAGE/ezrassor_autonomous_control" "$workspace_source_dir"
    fi
    if [ "$INSTALL_EXTERNALS" = "true" ]; then
        SUPERPACKAGE="$PWD/$EXTERNALS_DIR/viso2"
        ln -s -f "$SUPERPACKAGE/viso2" "$workspace_source_dir" 
        ln -s -f "$SUPERPACKAGE/libviso2" "$workspace_source_dir"
        ln -s -f "$SUPERPACKAGE/viso2_ros" "$workspace_source_dir"
    fi
    if [ "$INSTALL_DASHBOARD" = "true" ]; then
        :
    fi
    if [ "$INSTALL_SIMULATION" = "true" ]; then
        SUPERPACKAGE="$PWD/$SUPERPACKAGES_DIR/simulation"
        ln -s -f "$SUPERPACKAGE/ezrassor_sim_gazebo" "$workspace_source_dir"
        ln -s -f "$SUPERPACKAGE/ezrassor_sim_control" "$workspace_source_dir"
        ln -s -f "$SUPERPACKAGE/ezrassor_sim_description" "$workspace_source_dir"
    fi
    if [ "$INSTALL_COMMUNICATION" = "true" ]; then
        SUPERPACKAGE="$PWD/$SUPERPACKAGES_DIR/communication"
        ln -s -f "$SUPERPACKAGE/ezrassor_joy_translator" "$workspace_source_dir"
        ln -s -f "$SUPERPACKAGE/ezrassor_topic_switch" "$workspace_source_dir"
        ln -s -f "$SUPERPACKAGE/ezrassor_controller_server" "$workspace_source_dir"
    fi
    SUPERPACKAGE="$PWD/$SUPERPACKAGES_DIR/extras"
    ln -s -f "$SUPERPACKAGE/ezrassor_launcher" "$workspace_source_dir"

    # Install all of the dependencies of the linked packages in the temporary
    # workspace.
    cd "$workspace_dir"
    rosdep install --from-paths "$WORKSPACE_SOURCE_RELATIVE_DIR" \
                   --ignore-src \
                   --yes

    # Build and install the linked packages into the MANUAL_EZRASSOR_INSTALL_DIR.
    catkin_make
    mkdir -p "$MANUAL_EZRASSOR_INSTALL_DIR"
    catkin_make -DCMAKE_INSTALL_PREFIX="$MANUAL_EZRASSOR_INSTALL_DIR" install

    cd - > /dev/null 2>&1

    source_setups_in_directory "$MANUAL_EZRASSOR_INSTALL_DIR" 
}

# The main entry point to the installation script.
USER_SHELLS="bash zsh"
SH_SETUP_FILE="setup.sh"
ROSINSTALL_FILE="ros-comm.rosinstall"
KEY_SERVER="hkp://ha.pool.sks-keyservers.net:80"
RECV_KEY="C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654"
CATKIN_MAKE_ISOLATED_BIN="$WORKSPACE_SOURCE_RELATIVE_DIR/catkin/bin/catkin_make_isolated"
EXTERNALS_DIR="external"
SUPERPACKAGES_DIR="packages"
MOCK_INSTALL_RELATIVE_DIR="install"
WORKSPACE_SOURCE_RELATIVE_DIR="src"
MANUAL_ROS_INSTALL_DIR="$HOME/.ezrassor/ros"
WORKSPACE_PARTIAL_DIR="/tmp/ezrassor_workspace"
MANUAL_EZRASSOR_INSTALL_DIR="$HOME/.ezrassor/core"

# Throw a message if the script quits early, and tell the script to quit after
# any non-zero error message.
trap 'throw_error "Something went horribly wrong!"' 0
set -e

case "$1" in
    "ros")
        if [ "$2" = "--from-source=kinetic" ]; then
            install_ros_manually "kinetic"
        elif [ "$2" = "--from-source=melodic" ]; then
            install_ros_manually "melodic"
        else
            require "lsb_release"
            os_version="$(lsb_release -sc)"
            if [ "$os_version" = "xenial" ]; then
                ros_version="kinetic"
            elif [ "$os_version" = "bionic" ]; then
                ros_version="melodic"
            else
                throw_error "This script can only automatically install ROS for"
                            "Ubuntu Xenial and Ubuntu Bionic. Your operating system" \
                            "is not supported. :( You can still try to install ROS" \
                            "manually using the \"--from-source=\" parameter."
            fi
            automatic_ros_install_dir="/opt/ros/$ros_version"
            install_ros_automatically "$ros_version" "$automatic_ros_install_dir"
        fi
        ;;
    "buildtools")
        install_buildtools
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
