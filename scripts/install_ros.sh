# This script will automatically install and configure
# ROS Kinetic on Ubuntu Xenial (16.04).
# Written by Tiger Sachse.

# Add source for necessary packages.
#sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > \
#    /etc/apt/sources.list.d/ros-latest.list'
#sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 \
#    --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
#sudo apt update
#
#sudo apt install -y ros-kinetic-ros-base
#
##sudo apt install -y python-rosdep \
##                    python-rosinstall-generator \
##                    python-wstool \
##                    python-rosinstall \
##                    build-essential
##
## Initialize rosdep.
#sudo rosdep init
#rosdep update
#
##rosinstall_generator ros_comm \
##                     --rosdistro kinetic \
##                     --deps --wet-only --tar > \
##                     kinetic-ros_comm-wet.rosinstall
##wstool init -j8 src kinetic-ros_comm-wet.rosinstall
##rosdep install -y --from-paths src --ignore-src --rosdistro kinetic
#
#
## Source the installation's environment variables at each new session.
##echo "source /opt/ros/kinetic/setup.bash" >> "$HOME/.bashrc"
#source_kinetic_setup() {
#
#
#}

for USER_SHELL in bash zsh; do
    SHELLRC_FILE="$HOME/.${USER_SHELL}rc" 
    if [ -f "$SHELLRC_FILE" ]; then
        SOURCE_TARGET="/opt/ros/kinetic/setup.$USER_SHELL"
        SOURCE_LINE="source $SOURCE_TARGET"
        printf "Attempting to source setup script for %s: " "$USER_SHELL"
        if cat "$SHELLRC_FILE" | grep -Fq "$SOURCE_LINE"; then
            printf "Previously sourced!\n"
        else
            printf "%s\n" \
                   "" \
                   "# Source the ROS installation setup file, if it exists." \
                   "if [ -f \"$SOURCE_TARGET\" ]; then" \
                   "    $SOURCE_LINE" \
                   "fi" >> "$SHELLRC_FILE"
            printf "Successfully sourced!\n"
        fi
    fi
done
