Steps to begin working on ai/swarm branch:

mkdir ~/PATH_TO_WORKSPACE/EZ_RASSOR/

cd ~/PATH_TO_WORKSPACE/EZ_RASSOR/

git init

git remote add -f -m master -t ai/swarm origin https://github.com/FlaSpaceInst/NASA-E-RASSOR-Team

git checkout ai/swarm

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

sudo apt-get update

sudo apt-get install ros-kinetic-desktop-full

sudo rosdep init

rosdep update

echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc

source ~/.bashrc

sudo apt install libsuitesparse-dev libqglviewer-dev-qt4 ros-kinetic-libg2o  ros-kinetic-opencv3

sudo ln -s /usr/lib/x86_64-linux-gnu/libQGLViewer-qt4.so /usr/lib/x86_64-linux-gnu/libQGLViewer.so

sudo apt-get install ros-kinetic-ros-control ros-kinetic-ros-controllers

catkin_make






