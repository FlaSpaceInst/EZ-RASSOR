cd $1
source "devel/setup.bash"

# Spawn SLAM Viewer
rosrun lsd_slam_viewer viewer &

sleep 3

# Spawn SLAM Core
rosrun lsd_slam_core live_slam \
    /image:=/ez_rassor/front_camera/left/image_raw \
    /camera_info:=/ez_rassor/front_camera/left/camera_info &

sleep 10