cd $1
source "devel/setup.bash"

# ROS master and Gazebo world launch.
xterm -e roslaunch ez_rassor_gazebo ai_demo.launch &

sleep 6


# Spawn control nodes for ez_rassor hardware
rosrun ezrc_core wheels.py &
rosrun ezrc_core arms.py &
rosrun ezrc_core drums.py &

sleep 2

# Spawn SLAM Viewer
rosrun lsd_slam_viewer viewer &

sleep 2

# Spawn SLAM Core
rosrun lsd_slam_core live_slam \
    /image:=/ez_rassor/front_camera/left/image_raw \
    /camera_info:=/ez_rassor/front_camera/left/camera_info &

sleep 5

echo "Starting AI script in 5..."
sleep 1
echo "Starting AI script in 4..."
sleep 1
echo "Starting AI script in 3..."
sleep 1
echo "Starting AI script in 2..."
sleep 1
echo "Starting AI script in 1..."
sleep 1

rosrun ai_control ai_control.py
