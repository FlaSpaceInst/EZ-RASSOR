# CMake installation file for the ezrassor_swarm_control ROS package.

cmake_minimum_required(VERSION 2.8.3)
project(ezrassor_swarm_control)
find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    rospy
)
catkin_python_setup()
catkin_package()
catkin_install_python(
    PROGRAMS
        nodes/swarm_control
    DESTINATION
        ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(
    DIRECTORY
        launch/
    DESTINATION
        ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
