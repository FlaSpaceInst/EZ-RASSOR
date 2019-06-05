#!/bin/sh
WORKSPACE_DIR="/tmp/ezrassor_temporary_catkin_workspace"
DISTRIBUTION_CODENAME="$(lsb_release -sc)"
SOURCE_DIR="$WORKSPACE_DIR/src"
INSTALL_DIR="/opt/ros/melodic" #
SUPERPACKAGE_DIR="packages"
MOCK_INSTALL_DIR="install"
EXTERNAL_DIR="external"
SETUP_FILE="setup.bash" #
USER_SHELLS="bash zsh"

# Link a package into the workspace.
link_package() {
    if [ -L "$SOURCE_DIR/$2" ]; then
        rm -f "$SOURCE_DIR/$2"
        printf "Relinking '%s'...\n" "$2"
    else
        printf "Linking '%s'...\n" "$2"
    fi
    ln -s "$PWD/$1/$2" "$SOURCE_DIR/$2"
}

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

# Link and install a collection of packages from this repository.
link_and_install() {
    rm -rf "$WORKSPACE_DIR"
    mkdir -p "$SOURCE_DIR"

    # For each specified superpackage, link all necessary packages.
    for SUPERPACKAGE in "$@"; do
        case "$SUPERPACKAGE" in
            autonomy)
                link_package "external/viso2" "viso2"
                link_package "external/viso2" "libviso2"
                link_package "external/viso2" "viso2_ros"
                link_package "packages/autonomy" "ezrassor_autonomous_control"
                ;;
            simulation)
                link_package "packages/simulation" "ezrassor_sim_gazebo"
                link_package "packages/simulation" "ezrassor_sim_control"
                link_package "packages/simulation" "ezrassor_sim_description"
                ;;
            communication)
                link_package "packages/communication" "ezrassor_joy_translator"
                link_package "packages/communication" "ezrassor_topic_switch"
                link_package "packages/communication" "ezrassor_controller_server"
                ;;
            dashboard)
                ;;
        esac
    done
    link_package "packages/extras" "ezrassor_launcher"
    #sudo apt install -y ros-kinetic-ros-base \
    #                    python-rosdep \
    #                    python-rosinstall-generator \
    #                    python-wstool \
    #                    python-rosinstall \
    #                    build-essential

    source "$INSTALL_DIR/$SETUP_FILE"
    sudo rosdep init
    rosdep update

    # Install packages in the workspace from source.
    cd "$WORKSPACE_DIR"
    rosdep install -y --from-paths src --ignore-src --rosdistro melodic
    catkin_make
    catkin_make install
    sudo cp -R "$MOCK_INSTALL_DIR"/* "$INSTALL_DIR"
    cd - &> /dev/null

    #source_setup bash zsh
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





# Print an error message and exit this script.
throw_error() {
    printf "%s\n" "$@"
    exit 1
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

echo "$INSTALL_AUTONOMY"
echo "$INSTALL_EXTERNALS"
echo "$INSTALL_DASHBOARD"
echo "$INSTALL_SIMULATION"
echo "$INSTALL_COMMUNICATION"
echo "$INSTALLATION_METHOD"
echo "$OS_VERSION"
echo "$ROS_VERSION"
