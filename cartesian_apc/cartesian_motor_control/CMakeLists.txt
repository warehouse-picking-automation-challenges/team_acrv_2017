cmake_minimum_required(VERSION 2.8.3)
project(cartesian_motor_control)

set(CMAKE_CXX_FLAGS "-std=c++11 -Wall ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  hardware_interface
  controller_manager
  roscpp
  control_msgs
  trajectory_msgs
  actionlib
  urdf
  joint_limits_interface
  transmission_interface
  control_toolbox
  std_msgs
  sensor_msgs
  rosparam_shortcuts
  message_generation
)

add_service_files(
  FILES
  ChangeGrip.srv
)

generate_messages(
  DEPENDENCIES
  #std_msgs
)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)
# find_package(Gflags REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    hardware_interface
    controller_manager
    roscpp
    control_msgs
    trajectory_msgs
    urdf
    joint_limits_interface
    transmission_interface
    control_toolbox
    std_msgs
    dynamixel_msgs
    sensor_msgs
    rosparam_shortcuts
  LIBRARIES
    cartesian_motor_hw_interface
)

include_directories(
  include/
  ${catkin_INCLUDE_DIRS}
  ${Gflags_INCLUDE_DIRS}
)


# HarveyLift Hardware Interface
add_library(cartesian_motor_hw_interface
  src/cartesian_motor_hw_interface.cpp
)
target_link_libraries(cartesian_motor_hw_interface
  ${catkin_LIBRARIES}
)

# Main control executable
add_executable(cartesian_motor_hw_main src/cartesian_motor_hw_main.cpp)
target_link_libraries(cartesian_motor_hw_main
  cartesian_motor_hw_interface
  ${catkin_LIBRARIES}
)

# Test trajectory generator node
add_executable(${PROJECT_NAME}_test_trajectory src/test_trajectory.cpp)
set_target_properties(${PROJECT_NAME}_test_trajectory PROPERTIES OUTPUT_NAME test_trajectory PREFIX "")
target_link_libraries(${PROJECT_NAME}_test_trajectory
  ${catkin_LIBRARIES}
)

## Install ------------------------------------------------------------

# Install libraries
install(TARGETS
    cartesian_motor_hw_interface
  LIBRARY DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Install executables
install(TARGETS
    cartesian_motor_hw_main
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install header files
install(DIRECTORY include/cartesian_motor_control/   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
