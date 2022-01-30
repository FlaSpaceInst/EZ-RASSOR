# CMake installation file for the ezrassor_arm_autonomous_control ROS find_package
# Written by Robert Forristall
# Inspired by Cameron Taylor (EZ-RASSOR Autonomoy package)
cmake_minimum_required(VERSION 3.0.2)
project(ezrassor_arm_autonomous_control)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)
catkin_python_setup()
add_message_files(
  FILES
  ArmCommand.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
)
catkin_package(
  CATKIN_DEPENDS message_runtime
)
catkin_install_python(
  PROGRAMS
    nodes/move_group_interface
    nodes/object_detection
    nodes/autonomous_driving
    nodes/arm_action_server
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
    msg/
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}/msg
)

