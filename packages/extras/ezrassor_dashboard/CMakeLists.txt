# CMake installation file for the ezrassor_dashboard ROS package.
# Written by Tiger Sachse.
cmake_minimum_required(VERSION 3.1.0)

# Set some important variables so CMake knows where all the required files are.
set(EXECUTABLE_NAME dashboard)
set(PACKAGE_NAME ezrassor_dashboard)
set(DASHBOARD_RESOURCES_DIR resources)
set(DASHBOARD_INTERFACE_DIR ${DASHBOARD_RESOURCES_DIR}/interfaces)
set(DASHBOARD_SOURCE_DIR source/${PACKAGE_NAME})
set(DASHBOARD_INCLUDE_DIR include/${PACKAGE_NAME})
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC_SEARCH_PATHS ${DASHBOARD_INTERFACE_DIR})

# Initialize this project, plus hunt down Catkin and Qt5.
project(${PACKAGE_NAME})
find_package(catkin REQUIRED COMPONENTS roscpp cv_bridge sensor_msgs std_msgs)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)

catkin_package()

# Tell Qt5 where the resources are for this project, then create a new C++
# executable for the dashboard. Include necessary headers and finally install
# the executable into the appropriate bin.
qt5_add_resources(DASHBOARD_RESOURCES ${DASHBOARD_RESOURCES_DIR}/images.qrc)
include_directories(${catkin_INCLUDE_DIRS})
add_executable(
    ${EXECUTABLE_NAME}
    ${DASHBOARD_INTERFACE_DIR}/main_window.ui
    ${DASHBOARD_INCLUDE_DIR}/main_window.h
    ${DASHBOARD_INCLUDE_DIR}/help_window.h
    ${DASHBOARD_INCLUDE_DIR}/topic_translator.h
    ${DASHBOARD_SOURCE_DIR}/main_window.cpp
    ${DASHBOARD_SOURCE_DIR}/help_window.cpp
    ${DASHBOARD_SOURCE_DIR}/ezrassor_dashboard.cpp
    ${DASHBOARD_SOURCE_DIR}/topic_translator.cpp
    ${DASHBOARD_RESOURCES}
)
target_include_directories(${EXECUTABLE_NAME} PRIVATE ${DASHBOARD_INCLUDE_DIR})
target_link_libraries(${EXECUTABLE_NAME} Qt5::Widgets ${catkin_LIBRARIES})
install(
    TARGETS
        ${EXECUTABLE_NAME}
    RUNTIME DESTINATION
        ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install the status_monitor Python node.
catkin_python_setup()
catkin_install_python(
    PROGRAMS
        nodes/status_monitor
    DESTINATION
        ${CATKIN_PACKAGE_BIN_DESTINATION}
)
