SCRIPTS_DIR="scripts"
PACKAGES_DIR="packages"
WORK_SPACE="$HOME/.workspace"
SOURCE_SPACE="$WORK_SPACE/src"

# Make the main workspace.
function make_workspace {
    mkdir -p $SOURCE_SPACE

    cd $WORK_SPACE
    catkin_make
    cd -
}

function build_packages {
    echo ""
}

function run_ros_graph {
    echo ""
}

# Create a new ROS package.
function create_new_package {
    if [ ! -d $SOURCE_SPACE ]; then
        make_workspace
    fi

    cd $PACKAGES_DIR

    # Create a new catkin package with all the arguments
    # passed to this function.
    catkin_create_pkg "$@"

    # Make a few modifications to the new package structure.
    mkdir "$1/scripts"
    rm -rf "$1/include" "$1/src"

    # Create a symlink in the main workspace so that catkin
    # can build this new package.
    ln -s "$PWD/$1" "$SOURCE_SPACE/$1"

    cd -
}

# Relink all packages to the ROS workspace.
function relink_packages {
    if [ ! -d $SOURCE_SPACE ]; then
        make_workspace
    fi
}

for ARGUMENT in "$@"; do
    case $ARGUMENT in
        --install)
            bash $SCRIPTS_DIR/install_ros.sh
            make_workspace
            ;;
        --build)
            build_packages
            ;;
        --run)
            run_ros_graph
            ;;
        --new)
            shift
            create_new_package "$@"
            ;;
        --relink)
            relink_packages
            ;;
    esac
done
