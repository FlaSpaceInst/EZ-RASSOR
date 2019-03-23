# CMake installation file for the ezrassor_request_switch ROS package.
# Written by Tiger Sachse.
cmake_minimum_required(VERSION 2.8.3)
project(ezrassor_request_switch)
find_package(catkin REQUIRED)
catkin_package()
catkin_python_setup()
catkin_install_python(
    PROGRAMS
        nodes/request_switch
    DESTINATION
        ${CATKIN_PACKAGE_BIN_DESTINATION}
)
