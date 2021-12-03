# CMake installation file for the ezrassor_sim_description ROS package.
# Written by Ron Marrero.
cmake_minimum_required(VERSION 2.8.3)
project(ezrassor_sim_description)
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
        urdf/
    DESTINATION
        ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
) 
install(
    DIRECTORY
        meshes/
    DESTINATION
        ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes
) 
