cmake_minimum_required(VERSION 2.8.3)
project(apc_3d_vision)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -DBT_USE_DOUBLE_PRECISION -Wall")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    image_transport
    sensor_msgs
    shape_msgs
    cv_bridge
    message_filters
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    tf
    tf_conversions
    eigen_conversions
    message_generation
    image_geometry
)

find_package(octomap REQUIRED)

find_package(OpenCV REQUIRED)
find_package(OpenMP)
find_package(Eigen3 REQUIRED) # Eigen is not a ROS (Catkin) package but PCL is built as one
# find_package(PCL REQUIRED)

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
find_package(PCL 1.8 REQUIRED)
# find_package(PCL 1.2 REQUIRED)
# find_package(PCL 1.8 REQUIRED)

if(OPENMP_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

add_service_files(
    FILES
    split_labelled_point_cloud.srv
)

generate_messages(
    DEPENDENCIES
    sensor_msgs
    shape_msgs
    std_msgs
    geometry_msgs
    apc_3d_vision
)

catkin_package(
    CATKIN_DEPENDS
    cv_bridge
    roscpp
    image_transport
    message_filters
    tf
    tf_conversions
    eigen_conversions
    INCLUDE_DIRS include
    DEPENDS EIGEN3
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
    ${OpenCV_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS}
)

add_executable(find_free_space src/find_free_space.cpp)
add_executable(test_find_free_space src/test_find_free_space.cpp)
add_executable(cropTote src/cropTote.cpp src/apc_3d_vision.cpp)
add_executable(segment_pointcloud_node src/segment_pointcloud_node.cpp)
add_executable(image_to_world_node src/image_to_world_node.cpp)
add_executable(octomap_node src/octomap_node.cpp)
add_executable(test_octomap_node src/test_octomap_node.cpp)


## Specify libraries to link a library or executable target against

target_link_libraries(cropTote
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    ${Eigen_Libraries}
)
target_link_libraries(find_free_space
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)
target_link_libraries(test_find_free_space
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)
target_link_libraries(segment_pointcloud_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
)
target_link_libraries(image_to_world_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
)
target_link_libraries(octomap_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    ${OCTOMAP_LIBRARIES}
)
target_link_libraries(test_octomap_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    ${OCTOMAP_LIBRARIES}
)

add_dependencies(segment_pointcloud_node apc_msgs_generate_messages_cpp)
add_dependencies(image_to_world_node apc_msgs_generate_messages_cpp)
add_dependencies(find_free_space apc_3d_vision_gencpp)
add_dependencies(octomap_node apc_msgs_generate_messages_cpp)
add_dependencies(test_octomap_node apc_msgs_generate_messages_cpp)

add_dependencies(find_free_space apc_msgs_generate_messages_cpp)
add_dependencies(test_find_free_space apc_msgs_generate_messages_cpp)

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
