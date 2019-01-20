# A script that starts the ROS graph for the EZRC.

# Written by Tiger Sachse.
# Part of the EZ-RASSOR suite of software.

# The first argument to this script should be a catkin workspace. The
# setup.bash script must be sourced before every run of the graph.
cd $1
source "devel/setup.bash"

# ROS node initializations.
roscore &
rosrun ezrc_core arms.py &
