# This script will automatically install and configure
# ROS Kinetic on APT systems (Debian, Ubuntu, Raspbian).

# Written by Tiger Sachse.
# Part of the EZ-RASSOR suite of software.

# Add source for necessary packages.
sudo bash -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > \
    /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 \
    --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt update

# Install the full desktop version of ROS Kinetic. This installs the basic
# version first due to a weird bug that involves missing dependencies when
# installing only ros-kinetic-desktop-full.
sudo apt install -y ros-kinetic-desktop
sudo apt install -y ros-kinetic-desktop-full

# Initialize rosdep.
sudo rosdep init
rosdep update

# Source the installation's environment variables.
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
