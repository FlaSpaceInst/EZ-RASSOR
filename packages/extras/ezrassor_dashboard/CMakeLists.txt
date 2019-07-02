# CMake installation file for the ezrassor_dashboard ROS package.
# Written by Tiger Sachse.
cmake_minimum_required(VERSION 3.1.0)

# Set some important variables so CMake knows where all the required files are.
set(DASHBOARD_NAME ezrassor_dashboard)
set(DASHBOARD_RESOURCES_DIR resources)
set(DASHBOARD_INTERFACE_DIR ${DASHBOARD_RESOURCES_DIR}/interfaces)
set(DASHBOARD_SOURCE_DIR source/ezrassor_dashboard)
set(DASHBOARD_INCLUDE_DIR include/ezrassor_dashboard)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC_SEARCH_PATHS ${DASHBOARD_INTERFACE_DIR})

# Initialize this project, plus hunt down Catkin and Qt5.
project(${DASHBOARD_NAME})
catkin_package()
find_package(catkin REQUIRED)
find_package(Qt5 COMPONENTS Core Widgets REQUIRED)

# Tell Qt5 where the resources are for this project, then create a new C++
# executable for the dashboard. Include necessary headers and finally install
# the executable into the appropriate bin.
qt5_add_resources(DASHBOARD_RESOURCES ${DASHBOARD_RESOURCES_DIR}/images.qrc)
add_executable(
    ${DASHBOARD_NAME}
    ${DASHBOARD_INTERFACE_DIR}/main_window.ui
    ${DASHBOARD_INTERFACE_DIR}/about_window.ui
    ${DASHBOARD_INCLUDE_DIR}/main_window.h
    ${DASHBOARD_INCLUDE_DIR}/about_window.h
    ${DASHBOARD_SOURCE_DIR}/main_window.cpp
    ${DASHBOARD_SOURCE_DIR}/about_window.cpp
    ${DASHBOARD_SOURCE_DIR}/ezrassor_dashboard.cpp
    ${DASHBOARD_RESOURCES}
)
target_include_directories(${DASHBOARD_NAME} PRIVATE ${DASHBOARD_INCLUDE_DIR})
target_link_libraries(${DASHBOARD_NAME} Qt5::Widgets)
install(
    TARGETS
        ${DASHBOARD_NAME}
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
