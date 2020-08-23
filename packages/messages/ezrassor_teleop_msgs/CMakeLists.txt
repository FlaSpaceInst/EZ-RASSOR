# CMake installation file for the ezrassor_teleop_msgs ROS package.
# Written by Ron Marrero.
cmake_minimum_required(VERSION 3.0.2)
project(ezrassor_teleop_msgs) 
find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  rospy
  std_msgs
)
add_action_files(
  FILES
  Teleop.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
)
catkin_package()