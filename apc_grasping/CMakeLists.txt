cmake_minimum_required(VERSION 2.8.3)
project(apc_grasping)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -DBT_USE_DOUBLE_PRECISION -Wall")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  image_transport
  sensor_msgs
  message_filters
  tf
  tf_conversions
  eigen_conversions
  apc_msgs
  std_srvs
  pcl_conversions
  pcl_ros
  cv_bridge
  apc_msgs
)

find_package(OpenMP)


if(OPENMP_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/cmake/CustomPCLConfig.cmake")
    set(PCL_CONFIGURED true)
    message(STATUS "pcl: cmake/CustomPCLConfig.cmake found, project will be built.")
else()
    set(PCL_CONFIGURED false)
    message(WARNING "pcl: cmake/CustomPCLConfig.cmake not found, project won't be built.")
endif()

if (${PCL_CONFIGURED})
    include(cmake/CustomPCLConfig.cmake)
    message(STATUS "pcl: Looking for PCL configuration file at " ${PCL_DIR})
    find_package(PCL CONFIG REQUIRED)
    include_directories(${PCL_INCLUDE_DIRS})
    link_directories(${PCL_LIBRARY_DIRS})
    add_definitions(${PCL_DEFINITIONS})
endif()

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

## Generate added messages and services with any dependencies listed here
#  add_service_files(
#    FILES
#    grasp_pose.srv
#  )
#
# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   geometry_msgs
#   apc_grasping
#   # message_generation
# )

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
  CATKIN_DEPENDS roscpp image_transport message_filters
    moveit_core moveit_ros_planning moveit_ros_planning_interface
  INCLUDE_DIRS include
  #  LIBRARIES fruit_picking_test
  # DEPENDS system_lib
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
    ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(detect_grasp_candidates src/detect_grasp_candidates.cpp src/pcl_filters.cpp)
add_executable(cartesian_grasp_candidates src/cartesian_grasp_candidates.cpp src/pcl_filters.cpp)
add_executable(extract_pca src/extract_pca.cpp)
add_dependencies(detect_grasp_candidates apc_msgs_generate_messages_cpp)
add_dependencies(cartesian_grasp_candidates apc_msgs_generate_messages_cpp)
add_dependencies(extract_pca apc_msgs_generate_messages_cpp)

target_link_libraries(detect_grasp_candidates
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

target_link_libraries(cartesian_grasp_candidates
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

target_link_libraries(extract_pca
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)
