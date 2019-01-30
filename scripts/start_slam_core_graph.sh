# A script that starts SLAM core.

# Written by Tiger Sachse.
# Part of the EZ-RASSOR suite of software.

# The first argument to this script should be a catkin workspace. The
# setup.bash script must be sourced before every run of the graph.
cd $1
source "devel/setup.bash"

# ROS node initializations.
rosrun lsd_slam_core live_slam \
    /image:=/ez_rassor/camera1/image_raw1 \
    /camera_info:=/ez_rassor/camera1/camera_info &
