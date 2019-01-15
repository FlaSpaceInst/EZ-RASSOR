SCRIPTS_DIR="scripts"
PACKAGES_DIR="packages"
WORKSPACE_DIR="$HOME/.workspace"
SOURCE_DIR="$WORKSPACE_DIR/src"

install_software() {
    case $1 in
        ros)
            bash $SCRIPTS_DIR/install_ros.sh
            ;;
        devtools)
            sudo apt install -y python-rosinstall python-rosinstall-generator \
                python-wstool build-essential
            ;;
    esac
}

setup_catkin() {
    mkdir -p $SOURCE_DIR

    cd $WORKSPACE_DIR
    catkin_make
}

# Create a new ROS package in source control.
# Arguments are:
#   superpackage package [dependencies]
create_package() {

    # Make the superpackage, if necessary.
    mkdir -p $PACKAGES_DIR/$1

    cd $PACKAGES_DIR/$1

    # Create a new catkin package with all the arguments
    # passed to this function.
    catkin_create_pkg "${@:2}"

    # Add a scripts directory to the new package structure.
    mkdir "$2/scripts"

    # Create a symlink in the main workspace so that catkin
    # can build this new package.
    ln -s "$PWD/$2" "$SOURCE_DIR/$2"
}

# Relink all packages to the ROS workspace.
function relink_packages {
    if [ ! -d $SOURCE_DIR ]; then
        make_workspace
    fi
}

case $1 in
    --install)
        install_software $2
        ;;
    --setup-catkin)
        setup_catkin
        ;;
    --create-package)
        create_package "${@:2}"
        ;;
esac
