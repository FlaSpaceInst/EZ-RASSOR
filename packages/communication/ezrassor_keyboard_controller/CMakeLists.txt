cmake_minimum_required(VERSION 2.8.3)
project(ezrassor_keyboard_controller)
find_package(catkin REQUIRED)
catkin_package()
catkin_python_setup()
catkin_install_python(
  PROGRAMS
    nodes/keyboard_controller
  DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(
  DIRECTORY
    launch/
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
