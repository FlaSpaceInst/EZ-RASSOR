#!/bin/sh
WORKSPACE_PARTIAL_DIR="/tmp/ezrassor_workspace"
WORKSPACE_RELATIVE_SOURCE_DIR="src"
ROS_INSTALL_DIR="/opt/ezrassor"

#
DISTRIBUTION_CODENAME="$(lsb_release -sc)"
INSTALL_DIR="/opt/ros/melodic" #
SUPERPACKAGE_DIR="packages"
MOCK_INSTALL_DIR="install"
EXTERNAL_DIR="external"
SETUP_FILE="setup.bash" #
USER_SHELLS="bash zsh"
#

#
source_setup() {
    MUST_RESTART=false
    for USER_SHELL in "$USER_SHELLS"; do
        SHELLRC="$HOME/.${USER_SHELL}rc"
        if [ -f "$SHELLRC" ]; then
            SOURCE_TARGET="$INSTALL_DIR/setup.$USER_SHELL"
            SOURCE_LINE="source $SOURCE_TARGET"

            printf "Attempting to source setup script for %s: " "$USER_SHELL"
            if cat "$SHELLRC" | grep -Fq "$SOURCE_LINE"; then
                printf "Previously sourced!\n"
            else
                printf "%s\n" \
                       "" \
                       "# Source the ROS installation setup file, if it exists." \
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
               "RESTART YOUR TERMINAL FOR CHANGES TO TAKE EFFECT "
    fi
}

# Main entry point of the script.
#sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > \
#           /etc/apt/sources.list.d/ros-latest.list'
#sudo apt-key adv \
#             --keyserver hkp://ha.pool.sks-keyservers.net:80 \
#             --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
#sudo apt update

#if [ "$#" = "0" ]; then
#    link_and_install "communication" "hardware" "autonomy"
#else
#    link_and_install "$@"
#fi

    #sudo apt install -y ros-kinetic-ros-base \
    #                    python-rosdep \
    #                    python-rosinstall-generator \
    #                    python-wstool \
    #                    python-rosinstall \
    #                    build-essential

    #source "$INSTALL_DIR/$SETUP_FILE"
    #sudo rosdep init
    #rosdep update




# Print an error message and exit this script.
throw_error() {
    printf "%s\n" "$@"
    exit 1
}

install_ezrassor_packages() {
    WORKSPACE_DIR="${WORKSPACE_PARTIAL_DIR}_$(date +%s)"
    WORKSPACE_SOURCE_DIR="$WORKSPACE_DIR/$WORKSPACE_RELATIVE_SOURCE_DIR"
    mkdir -p "$WORKSPACE_SOURCE_DIR"

    if [ "$INSTALL_AUTONOMY" = "true" ]; then
        ln -s "$PWD/external/viso2/viso2" "$WORKSPACE_SOURCE_DIR" 
        ln -s "$PWD/external/viso2/libviso2" "$WORKSPACE_SOURCE_DIR"
        ln -s "$PWD/external/viso2/viso2_ros" "$WORKSPACE_SOURCE_DIR"
        ln -s "$PWD/packages/autonomy/ezrassor_autonomous_control" "$WORKSPACE_SOURCE_DIR"
    fi
    if [ "$INSTALL_EXTERNALS" = "true" ]; then
        ln -s "$PWD/external/viso2/viso2" "$WORKSPACE_SOURCE_DIR" 
        ln -s "$PWD/external/viso2/libviso2" "$WORKSPACE_SOURCE_DIR"
        ln -s "$PWD/external/viso2/viso2_ros" "$WORKSPACE_SOURCE_DIR"
    fi
    if [ "$INSTALL_DASHBOARD" = "true" ]; then
        :
    fi
    if [ "$INSTALL_SIMULATION" = "true" ]; then
        ln -s "$PWD/packages/simulation/ezrassor_sim_gazebo" "$WORKSPACE_SOURCE_DIR"
        ln -s "$PWD/packages/simulation/ezrassor_sim_control" "$WORKSPACE_SOURCE_DIR"
        ln -s "$PWD/packages/simulation/ezrassor_sim_description" "$WORKSPACE_SOURCE_DIR"
    fi
    if [ "$INSTALL_COMMUNICATION" = "true" ]; then
        ln -s "$PWD/packages/communication/ezrassor_joy_translator" "$WORKSPACE_SOURCE_DIR"
        ln -s "$PWD/packages/communication/ezrassor_topic_switch" "$WORKSPACE_SOURCE_DIR"
        ln -s "$PWD/packages/communication/ezrassor_controller_server" "$WORKSPACE_SOURCE_DIR"
    fi
    ln -s "$PWD/packages/extras/ezrassor_launcher" "$WORKSPACE_SOURCE_DIR"

    cd "$WORKSPACE_DIR"
    rosdep install -y \
                   --from-paths "$WORKSPACE_RELATIVE_SOURCE_DIR" \
                   --ignore-src \
                   --rosdistro "$ROS_VERSION"
    catkin_make
    catkin_make install
    #sudo cp -R "$MOCK_INSTALL_DIR"/* "$INSTALL_DIR"
    cd - &> /dev/null
}


# The main entry point to the installation script.
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

# Set a few installation flags with defaults.
INSTALL_AUTONOMY=true
INSTALL_EXTERNALS=true
INSTALL_DASHBOARD=true
INSTALL_SIMULATION=true
INSTALL_COMMUNICATION=true
INSTALLATION_METHOD="automatic"

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

install_ezrassor_packages
