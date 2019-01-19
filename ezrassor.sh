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

    printf "\n\nNOTICE!\n"
    echo "Please restart your terminal for changes to take effect."
}

# Set up the catkin workspace.
setup_catkin() {
    mkdir -p $SOURCE_DIR
    cd $SOURCE_DIR
    catkin_init_workspace
    cd ..
    build_packages
}

# Call catkin_make in the workspace.
catkin_make_workspace() {
    cd $WORKSPACE_DIR
    catkin_make
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
    mkdir -p "$PACKAGES_DIR/$1"
    cd "$PACKAGES_DIR/$1"

    # Create a new catkin package with all the arguments
    # passed to this function.
    catkin_create_pkg "${@:2}"

    # Create a symlink in the main workspace so that catkin
    # can build this new package.
    ln -s "$PWD/$2" "$SOURCE_DIR/$2"
}

# Link all packages to the ROS workspace.
link_packages() {
    cd $PACKAGES_DIR
    for SUPERPACKAGE_DIR in *; do
        cd $SUPERPACKAGE_DIR
        for PACKAGE_DIR in *; do
            if [ -L "$SOURCE_DIR/$PACKAGE_DIR" ]; then
                rm -f "$SOURCE_DIR/$PACKAGE_DIR"
                printf "Relinking %s...\n" $PWD/$PACKAGE_DIR
            else
                printf "Linking %s...\n" $PWD/$PACKAGE_DIR
            fi
            ln -s "$PWD/$PACKAGE_DIR" "$SOURCE_DIR/$PACKAGE_DIR"
        done
        cd ..
    done
}

# Purge packages in the ROS workspace.
purge_packages() {
    cd $SOURCE_DIR
    echo "Purging all packages in /src..."
    find . ! -name 'CMakeLists.txt' -type l -exec rm -f {} +
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
        link_packages
        ;;
    -b|--build)
        build_packages
        ;;
    -s|--start)
        start_ros_graph "${@:2}"
        ;;
    -k|--kill)
        kill_ros
        ;;
    -p|--purge)
        purge_packages
        ;;
    -m|--make)
        catkin_make_workspace
        ;;
esac
