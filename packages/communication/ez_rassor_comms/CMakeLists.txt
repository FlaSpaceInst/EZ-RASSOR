# CMake installation file for the ez_rassor_comms ROS package.
# Written by Tiger Sachse.
cmake_minimum_required(VERSION 2.8.3)
project(ez_rassor_comms)
find_package(
    catkin REQUIRED COMPONENTS
        rospy
        std_msgs
)

catkin_package()
catkin_python_setup()
catkin_install_python(
    PROGRAMS
        nodes/http_server
        nodes/request_switcher
    DESTINATION
        ${CATKIN_PACKAGE_BIN_DESTINATION}
)
