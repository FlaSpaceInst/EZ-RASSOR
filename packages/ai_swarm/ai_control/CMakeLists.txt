cmake_minimum_required(VERSION 2.8.3)
project(ai_control)

find_package(catkin REQUIRED COMPONENTS 
    message_generation
    std_msgs
    rospy
    roscpp
)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   ObstacleDetection.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   ai_control
)

catkin_package(

   CATKIN_DEPENDS message_runtime
)

include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)
