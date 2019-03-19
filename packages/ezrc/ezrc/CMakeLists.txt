#
cmake_minimum_required(VERSION 2.8.3)
project(ezrc)
find_package(
    catkin REQUIRED COMPONENTS
        rospy
        std_msgs
)

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
