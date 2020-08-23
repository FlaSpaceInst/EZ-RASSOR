# CMake installation file for the ezrassor_teleop_actions ROS package.
# Written by Ron Marrero.
cmake_minimum_required(VERSION 3.0.2)
project(ezrassor_teleop_actions) 
find_package(catkin REQUIRED COMPONENTS
  ezrassor_teleop_msgs
  rospy
  std_msgs
)
catkin_package()
catkin_python_setup()
catkin_install_python(
    PROGRAMS
        nodes/teleop_action_server
    DESTINATION
        ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(
    DIRECTORY
        launch/
    DESTINATION
        ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
