cmake_minimum_required(VERSION 2.8.3)
project(ezrassor_autonomous_control)

find_package(catkin REQUIRED COMPONENTS 
    message_generation
    std_msgs
    rospy
    roscpp
)

catkin_python_setup()

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   ObstacleDetection.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   ezrassor_autonomous_control
)

catkin_package(

   CATKIN_DEPENDS message_runtime
)

catkin_install_python(
    PROGRAMS
        nodes/distance_map
        nodes/autonomous_control
        nodes/tf_camera_broadcaster
    DESTINATION
        ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(
    DIRECTORY
        launch/
    DESTINATION
        ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
