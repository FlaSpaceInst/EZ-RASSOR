# This script will automatically install and configure
# ROS Kinetic on Ubuntu Xenial (16.04).
# Written by Tiger Sachse.

WORKSPACE_DIR="/tmp/EZRASSOR_TEMPORARY_CATKIN_WORKSPACE"
SOURCE_DIR="$WORKSPACE_DIR/src"

# Configure APT to install from the ROS repository.
add_ros_repository() {
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > \
        /etc/apt/sources.list.d/ros-latest.list'
    sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 \
        --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
    sudo apt update
}

link_package() {
    if [ -L "$SOURCE_DIR/$2" ]; then
        rm -f "$SOURCE_DIR/$2"
        printf "Relinking '%s'...\n" "$2"
    else
        printf "Linking '%s'...\n" "$2"
    fi
    ln -s "$PWD/$1/$2" "$SOURCE_DIR/$2"
}

# Source the setup script for each user shell passed to this function, if it is
# not already sourced in the appropriate RC file and if that user shell's RC
# file exists. Print a message if the user must restart her terminal.
source_setup() {
    MUST_RESTART=false
    for USER_SHELL in "$@"; do
        SHELLRC="$HOME/.${USER_SHELL}rc" 
        if [ -f "$SHELLRC" ]; then
            SOURCE_TARGET="/opt/ros/kinetic/setup.$USER_SHELL"
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
##add_ros_repository

mkdir -p $SOURCE_DIR
NEED_ROS_DESKTOP_FULL=false
NEED_ROS_DESKTOP=false
NEED_ROS_BASE=false

# For each specified superpackage, link all necessary packages.
for SUPERPACKAGE in "$@"; do
    case "$SUPERPACKAGE" in
        autonomy)
            ;;
        simulation)
            ;;
        communication)
            NEED_ROS_BASE=true
            link_package "communication" "ez_rassor_comms"
            link_package "ezrc" "ezrc_control"
            ;;
        hardware)
            ;;
        dashboard)
            ;;
    esac
done

if [ "$NEED_ROS_DESKTOP_FULL" = true ]; then
    sudo apt install -y ros-kinetic-desktop-full
else if [ "$NEED_ROS_DESKTOP" = true ]; then
    sudo apt install -y ros-kinetic-desktop
else if [ "$NEED_ROS_BASE" = true ]; then
    sudo apt install -y ros-kinetic-base
fi
##sudo rosdep init
##rosdep update

# build all packages

##source_setup bash zsh

##sudo apt install -y python-rosdep \
##                    python-rosinstall-generator \
##                    python-wstool \
##                    python-rosinstall \
##                    build-essential
##
## Initialize rosdep.
##rosinstall_generator ros_comm \
##                     --rosdistro kinetic \
##                     --deps --wet-only --tar > \
##                     kinetic-ros_comm-wet.rosinstall
##wstool init -j8 src kinetic-ros_comm-wet.rosinstall
##rosdep install -y --from-paths src --ignore-src --rosdistro kinetic
