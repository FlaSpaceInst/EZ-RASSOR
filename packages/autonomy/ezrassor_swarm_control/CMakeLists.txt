# CMake installation file for the ezrassor_swarm_control ROS package.

cmake_minimum_required(VERSION 2.8.3)
project(ezrassor_swarm_control)
find_package(catkin REQUIRED COMPONENTS
    actionlib_msgs
    geometry_msgs
    rospy
    genmsg
)
catkin_python_setup()

add_action_files(
  DIRECTORY action
  FILES waypoint.action
)

add_message_files(
    FILES Path.msg
)

add_service_files(
  DIRECTORY srv
  FILES GetRoverStatus.srv
)

generate_messages(DEPENDENCIES
  actionlib_msgs
  geometry_msgs
)

catkin_package(
    CATKIN_DEPENDS geometry_msgs actionlib_msgs message_runtime
)

catkin_install_python(
    PROGRAMS
        nodes/swarm_control
        nodes/waypoint_client
    DESTINATION
        ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    DIRECTORY
        launch/
    DESTINATION
        ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
