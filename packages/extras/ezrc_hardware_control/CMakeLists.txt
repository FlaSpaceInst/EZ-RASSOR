# CMake installation file for the ezrc__hardware_control ROS package.
# Written by Tiger Sachse.
cmake_minimum_required(VERSION 2.8.3)
project(ezrc_hardware_control)
find_package(catkin REQUIRED)
catkin_package()
catkin_python_setup()
catkin_install_python(
    PROGRAMS
        nodes/arms_driver
        nodes/drums_driver
        nodes/wheels_driver
    DESTINATION
        ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(
    DIRECTORY
        launchers/
    DESTINATION
        ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
