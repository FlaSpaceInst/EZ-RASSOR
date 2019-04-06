cmake_minimum_required(VERSION 2.8.3)
project(ez_gui)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

##############################################################################
# Catkin
##############################################################################
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(CMAKE_INCLUDE_CURRENT_DIR ON)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rviz
  cv_bridge
)

include_directories(/opt/ros/kinetic/include/opencv-3.3.1-dev)
##############################################################################
# Qt Environment
##############################################################################

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Qt5 REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)
file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/ros_gui/*.hpp *.h)
QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT5_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES test
#  CATKIN_DEPENDS roscpp rospy rviz
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

add_executable(ez_gui ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(ez_gui ${QT_LIBRARIES} ${catkin_LIBRARIES})
install(TARGETS ez_gui RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

