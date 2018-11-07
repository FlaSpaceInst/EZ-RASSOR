# WIP -TS

# Add source for necessary packages.
sudo bash -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > \
    /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 \
    --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

# Update the system.
sudo apt update && sudo apt upgrade -y

# Install bootstrap dependencies.
sudo apt install -y python-rosdep python-rosinstall-generator python-wstool \
    python-rosinstall build-essential cmake

# Initialize rosdep.
sudo rosdep init
rosdep update
