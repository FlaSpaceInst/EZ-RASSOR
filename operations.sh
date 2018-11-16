SCRIPTS_DIR="scripts"
PACKAGES_DIR="packages"
WORK_SPACE="$HOME/.workspace"
SOURCE_SPACE="$WORK_SPACE/src"

function make_workspace {
    mkdir -p $SOURCE_SPACE
    cd $WORK_SPACE
    catkin_make
}

function clean {
    rm -rf $WORK_SPACE
}

function copy_packages {
    if [ ! -d $SOURCE_SPACE ]; then
        make_workspace
    fi
    
    for PACKAGE in $PACKAGES_DIR/*; do
        cp -r $PACKAGE $SOURCE_SPACE/
    done
}

for ARGUMENT in "$@"; do
    case $ARGUMENT in
        --install)
            bash $SCRIPTS_DIR/install_ros.sh
            ;;
        --copy)
            copy_packages
            ;;
        --build)
            ;;
        --run)
            ;;
        --clean)
            ;;
    esac
done
