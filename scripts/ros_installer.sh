# This script will automatically install and configure
# ROS Melodic on APT systems (Debian, Ubuntu, Raspbian).

# Written by Tiger Sachse for the EZ-RASSOR project.

# UPDATE: We aren't using Melodic.

SHELL="bash"
SHELL_RC="$HOME/.bashrc"

# Add source for necessary packages.
sudo bash -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > \
    /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 \
    --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

# Update the system.
sudo apt update && sudo apt upgrade -y

# Install build tools.
sudo apt-get install python-rosinstall python-rosinstall-generator \
    python-wstool build-essential

# Install the basic version of ROS Melodic.
sudo apt install ros-melodic-ros-base

# Initialize rosdep.
sudo rosdep init
rosdep update

# Source the installation's environment variables.
echo "source /opt/ros/melodic/setup.$SHELL" >> $SHELL_RC
source $SHELL_RC
