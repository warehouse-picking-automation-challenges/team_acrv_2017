cmake_minimum_required(VERSION 2.8.3)
project(acrv_realsense_ros)

set(librealsense_DIR "/usr/local/share/librealsense/cmake")
find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  image_transport
  roscpp
  roslib
  std_msgs
  tf
  camera_info_manager
  cv_bridge
  message_generation
  librealsense
  acrv_apc_2017_perception
)

find_package(PCL 1.7 REQUIRED)

add_service_files(
  FILES
  get_camera_cloud.srv
  get_camera_image.srv
  get_camera_info.srv
  get_all_images.srv
  get_scalars.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  acrv_realsense_ros
)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
find_package(VTK REQUIRED)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(OpenCV REQUIRED)
IF (OPENCV_FOUND)
    MESSAGE("-- Found OpenCV version ${OPENCV_VERSION}: ${OPENCV_INCLUDE_DIRS}")
ENDIF (OPENCV_FOUND)

catkin_package(

)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${librealsense_INCLUDE_DIRS}
)

link_directories(
    ${OpenCV_LIBRARY_DIRS}
    ${PCL_LIBRARY_DIRS}
)

add_definitions(
    ${OpenCV_DEFINITIONS}
    ${PCL_DEFINITIONS}
)

add_executable(acrv_realsense_ros_node
    src/acrv_realsense_ros.cpp
)

add_executable(acrv_realsense_capture_service
    src/realsense_capture_service.cpp
)

add_executable(acrv_realsense_capture_client
    src/realsense_capture_client.cpp
)

# add_executable(acrv_inpaint_pub
#     src/inpaint_pub.cpp
# )

add_executable(register_depth
    src/register_depth.cpp
)

add_executable(data_regeneration_server_node
    src/data_regeneration_server.cpp
)

add_executable(acrv_r200_ros
    src/acrv_r200_ros.cpp
)

add_executable(sr300_r200_combined_service
    src/sr300_r200_combined_service.cpp
)

add_executable(test_sr300_r200_combined_service
    src/test_sr300_r200_combined_service.cpp
)

add_executable(get_serial
    src/get_serial.cpp
)

add_executable(capture_checkerboard_images
  src/capture_checkerboard_images.cpp)

# These lines are needed to ensure we generate messages
add_dependencies(acrv_realsense_capture_service acrv_realsense_ros_generate_messages_cpp)
add_dependencies(acrv_realsense_capture_client acrv_realsense_ros_generate_messages_cpp)
# add_dependencies(acrv_inpaint_pub acrv_realsense_ros_generate_messages_cpp)
add_dependencies(sr300_r200_combined_service apc_msgs_generate_messages_cpp)
add_dependencies(test_sr300_r200_combined_service apc_msgs_generate_messages_cpp)
add_dependencies(get_serial stereo_realsense_ros_generate_messages_cpp)


target_link_libraries(acrv_realsense_ros_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    -ludev
    ${librealsense_LIBRARIES}
    yaml-cpp
)

target_link_libraries(acrv_realsense_capture_service
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    -ludev
    ${librealsense_LIBRARIES}
)

target_link_libraries(acrv_realsense_capture_client
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    -ludev
    ${librealsense_LIBRARIES}
    ${PCL_COMMON_LIBRARIES}
    ${PCL_IO_LIBRARIES}
)

# target_link_libraries(acrv_inpaint_pub
#     ${catkin_LIBRARIES}
#     ${OpenCV_LIBS}
#     -ludev
#     ${librealsense_LIBRARIES}
# )

target_link_libraries(register_depth
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    -ludev
    ${librealsense_LIBRARIES}
)

target_link_libraries(data_regeneration_server_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    -ludev
    ${librealsense_LIBRARIES}
    ${PCL_COMMON_LIBRARIES}
    ${PCL_IO_LIBRARIES}
)

target_link_libraries(acrv_r200_ros
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    -ludev
    ${librealsense_LIBRARIES}
    yaml-cpp
    ${PCL_COMMON_LIBRARIES}
    ${PCL_IO_LIBRARIES}
)

target_link_libraries(sr300_r200_combined_service
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    -ludev
    ${librealsense_LIBRARIES}
    yaml-cpp
    ${PCL_COMMON_LIBRARIES}
    ${PCL_IO_LIBRARIES}
    ${PCL_FILTERS_LIBRARIES}
)

target_link_libraries(test_sr300_r200_combined_service
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    -ludev
    ${librealsense_LIBRARIES}
    yaml-cpp
    ${PCL_COMMON_LIBRARIES}
    ${PCL_IO_LIBRARIES}
    ${PCL_FILTERS_LIBRARIES}
)

target_link_libraries(get_serial
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    -ludev
    ${librealsense_LIBRARIES}
)

target_link_libraries(capture_checkerboard_images
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    -ludev
    ${librealsense_LIBRARIES}
)
