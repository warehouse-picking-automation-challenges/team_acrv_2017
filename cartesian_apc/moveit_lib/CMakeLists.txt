cmake_minimum_required(VERSION 2.8.3)
project(moveit_lib)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -DBT_USE_DOUBLE_PRECISION -Wall")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
  message_filters
  pcl_ros
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  tf_conversions
  eigen_conversions
  message_generation
)

#find_package(superquadric_fitter REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenMP)

#if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/cmake/CustomPCLConfig.cmake")
#  set(PCL_CONFIGURED true)
#  message(STATUS "pcl: cmake/CustomPCLConfig.cmake found, project will be built.")
#else()
#  set(PCL_CONFIGURED false)
#  message(WARNING "pcl: cmake/CustomPCLConfig.cmake not found, project won't be built.")
#endif()

#if (${PCL_CONFIGURED})
#  include(cmake/CustomPCLConfig.cmake)
#  message(STATUS "pcl: Looking for PCL configuration file at " ${PCL_DIR})
#  find_package(PCL CONFIG REQUIRED)
#  include_directories(${PCL_INCLUDE_DIRS})
#  link_directories(${PCL_LIBRARY_DIRS})
#  add_definitions(${PCL_DEFINITIONS})
#endif()
#find_package(PCL REQUIRED)

if(OPENMP_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################

## Generate added messages and services with any dependencies listed here
 add_service_files(
   FILES
   move_robot_tf.srv
   move_robot_pose.srv
   move_robot_pose_constrained.srv
   move_robot_pose_array.srv
   move_robot_named.srv
 )

generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
  geometry_msgs
  moveit_lib
)



###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES fruit_picking_test
#  DEPENDS system_lib
)

###########
## Build ##
###########
execute_process(COMMAND find ${PROJECT_SOURCE_DIR} -type f OUTPUT_VARIABLE FILES_STRING)
string(REPLACE "\n" ";" FILES_LIST ${FILES_STRING})
add_custom_target(additional_files_${PROJECT_NAME}
  SOURCES
  ${FILES_LIST}
)
## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(include
  #${capsicum_detection_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIR}
  #${PCL_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(moveit_lib
#   src/${PROJECT_NAME}/moveit_robot.cpp
# )

## Declare a cpp executable
 add_executable(moveit_robot src/moveit_robot_node.cpp src/moveit_robot.cpp )
 add_dependencies(moveit_robot apc_msgs_generate_messages_cpp)
 add_dependencies(moveit_robot moveit_lib_generate_messages_cpp)

## Add cmake target dependenciesolour_space.h"
#include <c of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(fruit_picking_test_node fruit_picking_test_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(fruit_picking
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBS}
#   ${PCL_LIBRARIES}
#   capsicum_filtering_tools
# )

# target_link_libraries(camera_move
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBS}
#   ${PCL_LIBRARIES}
#   capsicum_filtering_tools
# )

# target_link_libraries(grasp_pose_experiment
#   ${roscpp_LIBRARIES}
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBS}
#   ${PCL_LIBRARIES}
#   capsicum_filtering_tools
# )

#target_link_libraries(field_trial
#   ${roscpp_LIBRARIES}
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBS}
#   ${PCL_LIBRARIES}
#   capsicum_filtering_tools
# )

target_link_libraries(moveit_robot
   ${roscpp_LIBRARIES}
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
 )


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS fruit_picking_test fruit_picking_test_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_fruit_picking_test.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
