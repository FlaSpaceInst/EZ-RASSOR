# CMake installation file for the ezrassor_sim_gazebo ROS package.
# Written by Ron Marrero.
cmake_minimum_required(VERSION 2.8.3)
project(ezrassor_sim_gazebo)
find_package(catkin REQUIRED)
catkin_package()
install(
    DIRECTORY
        launch/
    DESTINATION
        ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install(
    DIRECTORY
        worlds/
    DESTINATION
        ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds
)
