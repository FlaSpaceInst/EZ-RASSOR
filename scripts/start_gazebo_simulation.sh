# A script that starts the Gazebo world simulation.

# Written by Tiger Sachse.
# Part of the EZ-RASSOR suite of software.

# The first argument to this script should be a catkin workspace. The
# setup.bash script must be sourced before every run of the graph.
cd $1
source "devel/setup.bash"

# ROS node initializations.
roscore &
roslaunch ez_rassor_gazebo ez_rassor_world.launch &
