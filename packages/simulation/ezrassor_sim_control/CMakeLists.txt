# CMake installation file for the ezrassor_sim_control ROS package.
# Written by Ron Marrero.
cmake_minimum_required(VERSION 2.8.3)
project(ezrassor_sim_control)
find_package(catkin REQUIRED)
catkin_package()
catkin_python_setup()
catkin_install_python(
    PROGRAMS
        nodes/sim_arms_driver
        nodes/sim_drums_driver
        nodes/sim_wheels_driver
	    nodes/sim_paver_arm_driver
    DESTINATION
        ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(
    DIRECTORY
        launch/
    DESTINATION
        ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install(
    DIRECTORY
        config/
    DESTINATION
        ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
