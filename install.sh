WORKSPACE_DIR="/tmp/ezrassor_temporary_catkin_workspace"
SOURCE_DIR="$WORKSPACE_DIR/src"
INSTALL_DIR="/opt/ros/melodic"
SUPERPACKAGE_DIR="packages"
MOCK_INSTALL_DIR="install"
EXTERNAL_DIR="external"
SETUP_FILE="setup.bash"

# Link a package into the workspace.
link_package() {
    if [ -L "$SOURCE_DIR/$2" ]; then
        rm -f "$SOURCE_DIR/$2"
        printf "Relinking '%s'...\n" "$2"
    else
        printf "Linking '%s'...\n" "$2"
    fi
    ln -s "$PWD/$1/$2" "$SOURCE_DIR/$2"
}

# Source the setup script for each user shell passed to this function if it is
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

# Link and install a collection of packages from this repository.
link_and_install() {
    rm -rf "$WORKSPACE_DIR"
    mkdir -p "$SOURCE_DIR"

    # For each specified superpackage, link all necessary packages.
    for SUPERPACKAGE in "$@"; do
        case "$SUPERPACKAGE" in
            autonomy)
                link_package "external/viso2" "viso2"
                link_package "external/viso2" "libviso2"
                link_package "external/viso2" "viso2_ros"
                link_package "packages/autonomy" "ezrassor_autonomous_control"
                ;;
            simulation)
                link_package "packages/simulation" "ezrassor_sim_gazebo"
                link_package "packages/simulation" "ezrassor_sim_control"
                link_package "packages/simulation" "ezrassor_sim_description"
                ;;
            communication)
                link_package "packages/communication" "ezrassor_joy_translator"
                link_package "packages/communication" "ezrassor_topic_switch"
                link_package "packages/communication" "ezrassor_controller_server"
                ;;
            hardware)
                ;;
            dashboard)
                ;;
        esac
    done
    link_package "packages/extras" "ezrassor_launcher"
    #sudo apt install -y ros-kinetic-ros-base \
    #                    python-rosdep \
    #                    python-rosinstall-generator \
    #                    python-wstool \
    #                    python-rosinstall \
    #                    build-essential

    source "$INSTALL_DIR/$SETUP_FILE"
    sudo rosdep init
    rosdep update

    # Install packages in the workspace from source.
    cd "$WORKSPACE_DIR"
    rosdep install -y --from-paths src --ignore-src --rosdistro melodic
    catkin_make
    catkin_make install
    sudo cp -R "$MOCK_INSTALL_DIR"/* "$INSTALL_DIR"
    cd - &> /dev/null

    #source_setup bash zsh
}

# Main entry point of the script.
#sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > \
#           /etc/apt/sources.list.d/ros-latest.list'
#sudo apt-key adv \
#             --keyserver hkp://ha.pool.sks-keyservers.net:80 \
#             --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
#sudo apt update

#if [ "$#" = "0" ]; then
#    link_and_install "communication" "hardware" "autonomy"
#else
#    link_and_install "$@"
#fi
