# A collection of functions that make working with this repository easier.

# Written by Tiger Sachse.
# Part of the EZ-RASSOR suite of software.

SCRIPTS_DIR="scripts"
PACKAGES_DIR="packages"
WORKSPACE_DIR="$HOME/.workspace"
SOURCE_DIR="$WORKSPACE_DIR/src"

# Install collections of software.
install_software() {
    for COLLECTION in "$@"; do
        case $COLLECTION in
            ros)
                bash "$SCRIPTS_DIR/install_ros.sh"
                ;;
            devtools)
                sudo apt install -y python-rosinstall build-essential \
                    python-rosinstall-generator python-wstool
                ;;
            ezrc)
                sudo apt install -y python-pip
                yes | sudo pip install RPi.GPIO adafruit-pca9685
                ;;
            ai|swarm)
                sudo apt install -y libsuitesparse-dev libqglviewer-dev-qt4 \
                    ros-kinetic-libg2o ros-kinetic-opencv3
                sudo ln -s /usr/lib/x86_64-linux-gnu/libQGLViewer-qt4.so \
                    /usr/lib/x86_64-linux-gnu/libQGLViewer.so
                sudo apt install -y ros-kinetic-ros-control ros-kinetic-ros-controllers
                ;;
            joy)
                sudo apt install ros-kinetic-joy
                sudo jstest /dev/input/js1
                sudo chmod a+rw /dev/input/js1
                ;;
            *)
                return
                ;;
        esac
    done

    printf "\n\nNOTICE!\n"
    echo "Please restart your terminal for changes to take effect."
}

# Set up the catkin workspace.
setup_catkin() {
    mkdir -p $SOURCE_DIR
    cd $SOURCE_DIR
    catkin_init_workspace
    cd - &> /dev/null
    build_packages
}

# Start a ROS graph.
start_ros() {
    case $1 in
        ezrc)
            source "$WORKSPACE/devel/setup.bash"
            roscore &
            rosrun ezrc_moving_parts arms_node.py &
            rosrun ezrc_moving_parts wheels_node.py &
            rosrun ezrc_moving_parts drums_node.py &
            ;;
        control)
            roslaunch ez_rassor_control ez_rassor_control.launch
            ;;
        gazebo)
            sudo killall rosmaster
            sudo killall gzserver
            sudo killall gzclient
            roslaunch ez_rassor_gazebo ez_rassor_world.launch
            ;;
        rviz)
            roslaunch ez_rassor_description ez_rassor_rviz.launch
            ;;
        slam-core)
            source "$WORKSPACE/devel/setup.bash"
            roscore &
            rosrun lsd_slam_core live_slam \
                /image:=ez_rassor/camera_<front/back>/image_raw \
                /camera_info:=/ez_rassor/camera_<front/back>/camera_info &
            ;;
        slam-viewer)
            source "$WORKSPACE/devel/setup.bash"
            roscore &
            rosrun lsd_slam_viewer viewer &
            ;;
    esac
}

# Kill all running ROS nodes.
kill_ros() {
    rosnode kill --all
    killall -SIGTERM roscore
    printf "\nROS has been shut down.\n"
}

# Build all packages.
build_packages() {
    cd $WORKSPACE_DIR
    catkin_make
    cd - &> /dev/null
}

# Create a new ROS package in source control.
# Arguments are:
#   superpackage package [dependencies]
new_package() {
    mkdir -p "$PACKAGES_DIR/$1"
    cd "$PACKAGES_DIR/$1"

    # Create a new catkin package with all the arguments
    # passed to this function.
    catkin_create_pkg "${@:2}"

    # Create a symlink in the main workspace so that catkin
    # can build this new package.
    ln -s "$PWD/$2" "$SOURCE_DIR/$2"

    cd - &> /dev/null
}

# Link all packages to the ROS workspace.
link_packages() {
    cd $PACKAGES_DIR

    for SUPERPACKAGE_DIR in *; do
        cd $SUPERPACKAGE_DIR
        for PACKAGE_DIR in *; do
            case $1 in
                -o|--only)
                    if argument_in_list "$PACKAGE_DIR" "${@:2}"; then
                        link_package "$PACKAGE_DIR"
                    fi
                    ;;
                -i|--ignore)
                    if ! argument_in_list "$PACKAGE_DIR" "${@:2}"; then
                        link_package "$PACKAGE_DIR"
                    fi
                    ;;
                *)
                    link_package "$PACKAGE_DIR"
                    ;;
            esac
        done
        cd ..
    done

    cd ..
}

# Helper function that links a single package.
link_package() {
    if [ -L "$SOURCE_DIR/$1" ]; then
        rm -f "$SOURCE_DIR/$1"
        printf "Relinking %s...\n" $PWD/$1
    else
        printf "Linking %s...\n" $PWD/$1
    fi
    ln -s "$PWD/$1" "$SOURCE_DIR/$1"
}

# Purge packages in the ROS workspace.
purge_packages() {
    cd $SOURCE_DIR
    echo "Purging all packages in /src..."
    find . ! -name 'CMakeLists.txt' -type l -exec rm -f {} +
    cd - &> /dev/null
}

# Helper function that determines if an argument is in a list.
argument_in_list() {
    for ARGUMENT in ${@:2}; do
        if [ "$1" = "$ARGUMENT" ]; then
            return 0
        fi
    done
   
    return 1
}

# Main entry point of the script.
case $1 in
    -i|--install)
        install_software "${@:2}"
        ;;
    -c|--catkin)
        setup_catkin
        ;;
    -n|--new)
        new_package "${@:2}"
        ;;
    -l|--link)
        link_packages "${@:2}"
        ;;
    -b|--build)
        build_packages
        ;;
    -s|--start)
        start_ros "${@:2}"
        ;;
    -k|--kill)
        kill_ros
        ;;
    -p|--purge)
        purge_packages
        ;;
    -r|--relink)
        purge_packages
        link_packages "${@:2}"
        ;;
esac
