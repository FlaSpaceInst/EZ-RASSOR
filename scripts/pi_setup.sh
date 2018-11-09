# A collection of functions that automatically install and prepare ROS.
# For use on a fresh Raspberry Pi. These functions are tuned for
# Raspbian Stretch and ROS Kinetic and so if you wish to use
# distributions other than these you may need to modify this script.

# This script can be run using this command:
#       $ bash pi_setup.sh [--flag]

# Flags allow you to execute different parts of this script:
#   --prerequisites
#       Install and configure the prerequisites for ROS on the Raspberry Pi.
#   --download
#       Download ROS packages into a new Catkin workspace.
#   --build
#       Build ROS packages in a Catkin workspace.
# These flags can be chained together, and they will execute in order.
# If you provide no flags, all of the functions will be run in the correct
# order (recommended).

# Written by Tiger Sachse for the EZ-RASSOR project.

SHELL="bash"
SHELL_RC="$HOME/.bashrc"
WORKSPACE="$HOME/workspace"
INSTALL_SPACE="/opt/ros/kinetic"

# Configure the prerequisites for ROS.
function setup_prerequisites {

    # Add source for necessary packages.
    sudo bash -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > \
        /etc/apt/sources.list.d/ros-latest.list'
    sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 \
        --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

    # Update the system.
    sudo apt update && sudo apt upgrade -y

    # Install bootstrap dependencies.
    sudo apt install -y python-rosdep python-rosinstall-generator python-wstool \
        python-rosinstall build-essential cmake

    # Initialize rosdep.
    sudo rosdep init
    rosdep update
}

# Download some ROS packages to get us started.
function download_packages {

    # Create a Catkin workspace.
    rm -rf $WORKSPACE
    mkdir -p $WORKSPACE
    cd $WORKSPACE

    # Install the core ROS packages and communication libraries. This section
    # skips downloading Assimp because it has issues during compilation and
    # is unneeded anyway.
    rosinstall_generator ros_comm --rosdistro kinetic --deps --wet-only \
        --exclude collada_parser collada_urdf --tar > kinetic-ros_comm-wet.rosinstall
    wstool init src kinetic-ros_comm-wet.rosinstall

    # Resolve dependencies.
    rosdep install -y --from-paths src --ignore-src --rosdistro kinetic \
        -r --os=debian:stretch

    cd -
}

# Build the packages located in the Catkin workspace.
function build_packages {
    cd $WORKSPACE

    sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release \
        --install-space $INSTALL_SPACE -j2 --quiet

    # Source the setup script for the appropriate shell.
    source $INSTALL_SPACE/setup.$SHELL
    echo "source $INSTALL_SPACE/setup.$SHELL" >> $SHELL_RC

    cd -
}

# Run every part of this script.
function run_full_installation {
    setup_prerequisites
    download_packages
    build_packages
}

# Main entry point to the script. If no arguments are provided, run everything.
# Otherwise, run the sections of this script that are specified by the given flags.
if (( $# == 0 )); then
    run_full_installation
else
    for ARGUMENT in "$@"
    do
        case $ARGUMENT in
            --prerequisites)
                setup_prerequisites
                ;;
            --download)
                download_packages
                ;;
            --build)
                build_packages
                ;;
        esac
    done
fi
