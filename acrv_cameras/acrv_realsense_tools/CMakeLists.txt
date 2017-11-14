cmake_minimum_required(VERSION 2.1)
project(acrv_realsense_tools)

set(CMAKE_CXX_STANDARD 11)

find_package(librealsense)
find_package(PCL 1.7 REQUIRED)

add_definitions(-std=c++11)

include_directories(${PCL_INCLUDE_DIRS} ${librealsense_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_library(LIBREALSENSE_LIBRARY NAMES librealsense)
include_directories(include/realsense_tools)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})

add_executable(capture_data_structure_one src/capture_data_structure_one.cpp)
add_executable(capture_data_structure_two src/capture_data_structure_two.cpp)
add_executable(align_depth_to_rgb src/align_depth_to_rgb.cpp)
add_executable(get_camera_params src/get_camera_params.cpp)
add_executable(capture_checkerboard_images src/capture_checkerboard_images.cpp)
# add_executable(test src/test.cpp)

target_link_libraries(capture_data_structure_one ${catkin_LIBRARIES} ${OpenCV_LIBS} -ludev ${librealsense_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})
target_link_libraries(capture_data_structure_two ${catkin_LIBRARIES} ${OpenCV_LIBS} -ludev ${librealsense_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})
target_link_libraries(align_depth_to_rgb ${catkin_LIBRARIES} ${OpenCV_LIBS} -ludev ${librealsense_LIBRARIES})
target_link_libraries(get_camera_params ${catkin_LIBRARIES} ${OpenCV_LIBS} -ludev ${librealsense_LIBRARIES})
target_link_libraries(capture_checkerboard_images ${catkin_LIBRARIES} ${OpenCV_LIBS} -ludev ${librealsense_LIBRARIES})
# target_link_libraries(test ${catkin_LIBRARIES} ${OpenCV_LIBS} -ludev ${librealsense_LIBRARIES})
# target_link_libraries(~Downloads/yaml-cpp-master)
