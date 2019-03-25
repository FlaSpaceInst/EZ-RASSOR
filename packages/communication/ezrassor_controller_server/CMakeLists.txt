# CMake installation file for the ezrassor_controller_server ROS package.
# Written by Tiger Sachse.
cmake_minimum_required(VERSION 2.8.3)
project(ezrassor_controller_server)
find_package(catkin REQUIRED)
catkin_package()
catkin_python_setup()
catkin_install_python(
    PROGRAMS
        nodes/controller_server
    DESTINATION
        ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(
    DIRECTORY
        launch/
    DESTINATION
        ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
