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
            ai|swarm)
                sudo apt install -y libsuitesparse-dev libqglviewer-dev-qt4 \
                    ros-kinetic-libg2o ros-kinetic-opencv3 ros-kinetic-ros-control \
                    ros-kinetic-ros-controllers
                sudo ln -s /usr/lib/x86_64-linux-gnu/libQGLViewer-qt4.so \
                    /usr/lib/x86_64-linux-gnu/libQGLViewer.so
        esac
    done
}

# Set up the catkin workspace.
setup_catkin() {
    mkdir -p $SOURCE_DIR

    cd $WORKSPACE_DIR
    catkin_make
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
function relink_packages {
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
esac
