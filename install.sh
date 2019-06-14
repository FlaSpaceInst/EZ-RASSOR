#!/bin/sh
#
#

# Print an error message and exit this script.
throw_error() {
    printf "%s\n" "$@"
    trap ":" 0
    exit 1
}

# Add the ROS repositories to APT.
add_ros_repository() {
    ECHO_COMMAND="echo \"deb http://packages.ros.org/ros/ubuntu $OS_VERSION main\""
    ROS_LATEST_DIR="/etc/apt/sources.list.d/ros-latest.list"
    sudo sh -c "$ECHO_COMMAND > $ROS_LATEST_DIR"
    sudo apt-key adv --keyserver "$KEY_SERVER" --recv-key "$RECV_KEY"
    sudo apt update
}

# Install buildtools for ROS.
install_ros_buildtools() {
    sudo apt install -y python-rosdep \
                        python-rosinstall-generator \
                        python-wstool \
                        python-rosinstall \
                        build-essential
}

# Source setup files within a given directory in the user's RC files.
source_setups_in_directory() {
    MUST_RESTART=false
    PARTIAL_SOURCE_TARGET="$1/setup"
    for USER_SHELL in $USER_SHELLS; do
        SHELLRC="$HOME/.${USER_SHELL}rc"
        if [ -f "$SHELLRC" ]; then
            SOURCE_TARGET="$PARTIAL_SOURCE_TARGET.$USER_SHELL"
            SOURCE_LINE="source $SOURCE_TARGET"

            printf "Attempting to source setup script for %s: " "$USER_SHELL"
            if cat "$SHELLRC" | grep -Fq "$SOURCE_LINE"; then
                printf "Previously sourced!\n"
            else
                printf "%s\n" \
                       "" \
                       "# Source a ROS setup file, if it exists." \
                       "if [ -f \"$SOURCE_TARGET\" ]; then" \
                       "    $SOURCE_LINE" \
                       "fi" >> "$SHELLRC"
                printf "Successfully sourced!\n"
                MUST_RESTART=true
            fi
        fi
    done

    if [ "$MUST_RESTART" = true ]; then
        printf "\n\n******** %s ********\n" \
               "RESTART YOUR TERMINAL FOR CHANGES TO TAKE EFFECT"
    fi
}

# Install ROS automatically with APT.
install_ros_automatically() {
    add_ros_repository
    sudo apt install -y "ros-${ROS_VERSION}-ros-base"
    sudo rosdep init || true
    rosdep update
    source_setups_in_directory "$AUTOMATIC_INSTALL_DIR" 
}

install_ros_manually() {
    install_ros_buildtools
}

# Install only EZ-RASSOR packages.
install_ezrassor_packages() { # check if rosdep installed
                              # source seperate install dir after
                              # check if ros is installed

    # Create a temporary workspace.
    WORKSPACE_DIR="${WORKSPACE_PARTIAL_DIR}_$(date +%s)"
    WORKSPACE_SOURCE_DIR="$WORKSPACE_DIR/$WORKSPACE_SOURCE_RELATIVE_DIR"
    mkdir -p "$WORKSPACE_SOURCE_DIR"

    # Link packages into the temporary workspace based on the INSTALL flags. These
    # flags are set at the beginning of the script.
    if [ "$INSTALL_AUTONOMY" = "true" ]; then
        SUPERPACKAGE="$PWD/$EXTERNALS_DIR/viso2"
        ln -s -f "$SUPERPACKAGE/viso2" "$WORKSPACE_SOURCE_DIR" 
        ln -s -f "$SUPERPACKAGE/libviso2" "$WORKSPACE_SOURCE_DIR"
        ln -s -f "$SUPERPACKAGE/viso2_ros" "$WORKSPACE_SOURCE_DIR"
        SUPERPACKAGE="$PWD/$SUPERPACKAGES_DIR/autonomy"
        ln -s -f "$SUPERPACKAGE/ezrassor_autonomous_control" "$WORKSPACE_SOURCE_DIR"
    fi
    if [ "$INSTALL_EXTERNALS" = "true" ]; then
        SUPERPACKAGE="$PWD/$EXTERNALS_DIR/viso2"
        ln -s -f "$SUPERPACKAGE/viso2" "$WORKSPACE_SOURCE_DIR" 
        ln -s -f "$SUPERPACKAGE/libviso2" "$WORKSPACE_SOURCE_DIR"
        ln -s -f "$SUPERPACKAGE/viso2_ros" "$WORKSPACE_SOURCE_DIR"
    fi
    if [ "$INSTALL_DASHBOARD" = "true" ]; then
        :
    fi
    if [ "$INSTALL_SIMULATION" = "true" ]; then
        SUPERPACKAGE="$PWD/$SUPERPACKAGES_DIR/simulation"
        ln -s -f "$SUPERPACKAGE/ezrassor_sim_gazebo" "$WORKSPACE_SOURCE_DIR"
        ln -s -f "$SUPERPACKAGE/ezrassor_sim_control" "$WORKSPACE_SOURCE_DIR"
        ln -s -f "$SUPERPACKAGE/ezrassor_sim_description" "$WORKSPACE_SOURCE_DIR"
    fi
    if [ "$INSTALL_COMMUNICATION" = "true" ]; then
        SUPERPACKAGE="$PWD/$SUPERPACKAGES_DIR/communication"
        ln -s -f "$SUPERPACKAGE/ezrassor_joy_translator" "$WORKSPACE_SOURCE_DIR"
        ln -s -f "$SUPERPACKAGE/ezrassor_topic_switch" "$WORKSPACE_SOURCE_DIR"
        ln -s -f "$SUPERPACKAGE/ezrassor_controller_server" "$WORKSPACE_SOURCE_DIR"
    fi
    SUPERPACKAGE="$PWD/$SUPERPACKAGES_DIR/extras"
    ln -s -f "$SUPERPACKAGE/ezrassor_launcher" "$WORKSPACE_SOURCE_DIR"

    # Install all of the dependencies of the linked packages in the temporary
    # workspace.
    cd "$WORKSPACE_DIR"
    rosdep install -y \
                   --from-paths "$WORKSPACE_SOURCE_RELATIVE_DIR" \
                   --ignore-src \
                   --rosdistro "$ROS_VERSION"

    # Build and install the linked packages into the EZRASSOR_INSTALL_DIR.
    catkin_make
    catkin_make install
    sudo mkdir -p "$EZRASSOR_INSTALL_DIR"
    sudo cp -R "$MOCK_INSTALL_RELATIVE_DIR"/* "$EZRASSOR_INSTALL_DIR"

    cd - > /dev/null 2>&1
}

# The main entry point to the installation script.
USER_SHELLS="bash zsh"
INSTALL_AUTONOMY=true
INSTALL_EXTERNALS=true
INSTALL_DASHBOARD=true
INSTALL_SIMULATION=true
INSTALL_COMMUNICATION=true
INSTALLATION_METHOD="automatic"
EXTERNALS_DIR="external"
SUPERPACKAGES_DIR="packages"
MOCK_INSTALL_RELATIVE_DIR="install"
WORKSPACE_SOURCE_RELATIVE_DIR="src"
EZRASSOR_INSTALL_DIR="/opt/ezrassor"
WORKSPACE_PARTIAL_DIR="/tmp/ezrassor_workspace"
KEY_SERVER="hkp://ha.pool.sks-keyservers.net:80"
RECV_KEY="C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654"

# Throw a message if something breaks.
trap 'throw_error "Something went horribly wrong!"' 0
set -e

# Determine the user's OS version and an appropriate ROS version.
OS_VERSION="$(lsb_release -sc)"
if [ "$OS_VERSION" = "xenial" ]; then
    ROS_VERSION="kinetic"
elif [ "$OS_VERSION" = "bionic" ]; then
    ROS_VERSION="melodic"
else
    throw_error "This script is only tested on Ubuntu Xenial and Ubuntu Bionic with ROS" \
                "Kinetic and ROS Melodic. Your operating system is not supported. :(" \
                "You may still attempt to install the packages that you want manually" \
                "using Catkin. Refer to the ROS wiki for instructions."
fi
AUTOMATIC_INSTALL_DIR="/opt/ros/$ROS_VERSION"

# Determine the user's command line arguments.
SET_COMPONENTS=false
SET_INSTALLATION_METHOD=false
MISSING_COMPONENTS_LIST=false
for ARGUMENT in "$@"; do

    # If the user has included the "--method" flag, set the INSTALLATION_METHOD
    # appropriately.
    if [ "$SET_INSTALLATION_METHOD" = "true" ]; then
        if [ "$ARGUMENT" = "automatic" ]; then
            INSTALLATION_METHOD="automatic"
        elif [ "$ARGUMENT" = "manual" ]; then
            INSTALLATION_METHOD="manual"
        elif [ "$ARGUMENT" = "packages-only" ]; then
            INSTALLATION_METHOD="packages-only"
        else
            throw_error "Invalid installation method: $ARGUMENT"
        fi
        SET_INSTALLATION_METHOD=false
        continue

    # If the user has included the "--components" flag, set the
    # INSTALL_COMPONENT flags appropriately.
    elif [ "$SET_COMPONENTS" = "true" ]; then
        if [ "$ARGUMENT" = "autonomy" ]; then
            MISSING_COMPONENTS_LIST=false
            INSTALL_AUTONOMY=true
            continue
        elif [ "$ARGUMENT" = "externals" ]; then
            MISSING_COMPONENTS_LIST=false
            INSTALL_EXTERNALS=true
            continue
        elif [ "$ARGUMENT" = "dashboard" ]; then
            MISSING_COMPONENTS_LIST=false
            INSTALL_DASHBOARD=true
            continue
        elif [ "$ARGUMENT" = "simulation" ]; then
            MISSING_COMPONENTS_LIST=false
            INSTALL_SIMULATION=true
            continue
        elif [ "$ARGUMENT" = "communication" ]; then
            MISSING_COMPONENTS_LIST=false
            INSTALL_COMMUNICATION=true
            continue
        else
            SET_COMPONENTS=false
        fi
    fi

    # Check the user's arguments for supported flags.
    case "$ARGUMENT" in
        --method)
            SET_INSTALLATION_METHOD=true
            ;;
        --components)
            SET_COMPONENTS=true
            INSTALL_AUTONOMY=false
            INSTALL_EXTERNALS=false
            INSTALL_DASHBOARD=false
            INSTALL_SIMULATION=false
            INSTALL_COMMUNICATION=false
            MISSING_COMPONENTS_LIST=true
            ;;
    esac
done

# Throw an error if the user used a flag incorrectly.
if [ "$SET_INSTALLATION_METHOD" = "true" ]; then
    throw_error "Missing installation method."
elif [ "$MISSING_COMPONENTS_LIST" = "true" ]; then
    throw_error "Missing components list."
fi

# Install ROS and/or EZ-RASSOR packages based on the specified installation method.

# NOTE this process will have to change bc ros cant be installed and then use
# catkin in the same script on same run, need to restart terminal in between
if [ "$INSTALLATION_METHOD" = "automatic" ]; then
    install_ros_automatically
    install_ezrassor_packages
elif [ "$INSTALLATION_METHOD" = "manual" ]; then
    install_ros_manually
    install_ezrassor_packages
elif [ "$INSTALLATION_METHOD" = "packages-only" ]; then
    install_ezrassor_packages
else
    throw_error "Invalid installation method."
fi
trap ":" 0
