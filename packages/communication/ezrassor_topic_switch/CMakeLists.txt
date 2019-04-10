# CMake installation file for the ezrassor_topic_switch ROS package.
# Written by Tiger Sachse.
cmake_minimum_required(VERSION 2.8.3)
project(ezrassor_topic_switch)
find_package(catkin REQUIRED)
catkin_package()
catkin_python_setup()
catkin_install_python(
    PROGRAMS
        nodes/topic_switch
    DESTINATION
        ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(
    DIRECTORY
        launch/
    DESTINATION
        ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
if(CATKIN_ENABLE_TESTING)
    find_package(rostest)
    add_rostest(test/test_single_topic_switch.test)
endif()
