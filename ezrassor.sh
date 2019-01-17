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
                yes | sudo pip install RPi.GPIO #adafruit-pca9685 \
                    #mpu6050-raspberrypi cffi smbus-cffi
                ;;
            ai|swarm)
                sudo apt install -y libsuitesparse-dev libqglviewer-dev-qt4 \
                    ros-kinetic-libg2o ros-kinetic-opencv3 ros-kinetic-ros-control \
                    ros-kinetic-ros-controllers
                sudo ln -s /usr/lib/x86_64-linux-gnu/libQGLViewer-qt4.so \
                    /usr/lib/x86_64-linux-gnu/libQGLViewer.so
                ;;
        esac
    done
}

# Set up the catkin workspace.
setup_catkin() {
    mkdir -p $SOURCE_DIR

    cd $SOURCE_DIR
    catkin_init_workspace
    cd ..

    build_packages
}

# Start a ROS graph.
start_ros_graph() {
    case $1 in
        ezrc)
            bash "$SCRIPTS_DIR/start_ezrc_graph.sh" $WORKSPACE_DIR
            ;;
    esac
}

# Kill all running ROS nodes.
kill_ros() {
    rosnode kill --all
    killall -SIGTERM roscore
}

# Build all packages.
build_packages() {
    cd $WORKSPACE_DIR

    catkin_make
    source "$WORKSPACE_DIR/devel/setup.bash"
}

# Create a new ROS package in source control.
# Arguments are:
#   superpackage package [dependencies]
new_package() {

    # Make the superpackage, if necessary.
    mkdir -p "$PACKAGES_DIR/$1"

    cd "$PACKAGES_DIR/$1"

    # Create a new catkin package with all the arguments
    # passed to this function.
    catkin_create_pkg "${@:2}"

    # Create a symlink in the main workspace so that catkin
    # can build this new package.
    ln -s "$PWD/$2" "$SOURCE_DIR/$2"
}

# Relink all packages to the ROS workspace.
relink_packages() {
    cd $PACKAGES_DIR
    for SUPERPACKAGE_DIR in *; do
        cd $SUPERPACKAGE_DIR
        for PACKAGE_DIR in *; do
            if [ -L "$SOURCE_DIR/$PACKAGE_DIR" ]; then
                rm "$SOURCE_DIR/$PACKAGE_DIR"
            fi
            ln -s "$PWD/$PACKAGE_DIR" "$SOURCE_DIR/$PACKAGE_DIR"
            printf "Linked %s.\n" "$PWD/$PACKAGE_DIR"
        done
        cd - &> /dev/null
    done
    cd - &> /dev/null
}

# Main entry point of the script.
case $1 in
    --install)
        install_software "${@:2}"
        ;;
    --catkin)
        setup_catkin
        ;;
    --new)
        new_package "${@:2}"
        ;;
    --relink)
        relink_packages
        ;;
    --build)
        build_packages
        ;;
    --start)
        start_ros_graph "${@:2}"
        ;;
    --kill)
        kill_ros
        ;;
esac
