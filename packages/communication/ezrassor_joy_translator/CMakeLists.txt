# CMake installation file for the ezrassor_joy_translator ROS package.
# Written by Tiger Sachse.
cmake_minimum_required(VERSION 2.8.3)
project(ezrassor_joy_translator)
find_package(catkin REQUIRED)
catkin_package()
catkin_python_setup()
catkin_install_python(
    PROGRAMS
        nodes/joy_translator
    DESTINATION
        ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(
    DIRECTORY
        launchers/
    DESTINATION
        ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
