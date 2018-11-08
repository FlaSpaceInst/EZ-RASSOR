# WIP -TS

WORKSPACE="$HOME/workspace"

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

    # Create a workspace.
    rm -rf $WORKSPACE
    mkdir -p $WORKSPACE
    cd $WORKSPACE

    # Install the core ROS packages and communication libraries.
    rosinstall_generator ros_comm --rosdistro kinetic --deps --wet-only --tar > \
        kinetic-ros_comm-wet.rosinstall
    wstool init src kinetic-ros_comm-wet.rosinstall
}

# Run every part of this script.
function run_full_installation {
    #setup_prerequisites
    #download_packages
    echo "running all"
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
            --packages)
                download_packages
                ;;
        esac
    done
fi
