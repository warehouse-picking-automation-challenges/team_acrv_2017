#include <string>
// Include service types
#include <acrv_realsense_ros/get_camera_info.h>
#include <acrv_realsense_ros/get_camera_cloud.h>
#include <acrv_realsense_ros/get_camera_image.h>
#include <acrv_realsense_ros/get_all_images.h>
#include <vector>
#include <ros/ros.h>
#include <ros/callback_queue.h>
// Include message types
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <librealsense/rs.hpp>
#include <math.h>
#include <sstream>
#include <string>
#include <iostream>
#include <iomanip>
#include <thread>
#include <cstdio>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cstdlib>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>


#include <sstream>
/*
Capture script for the RealSense Camera.
*/

int main(int argc, char** argv)
{
    ros::init(argc, argv, "realsense_capture_client");
    ros::NodeHandle n;

    cv::Size board_size;
    board_size.width = 8;
    board_size.height = 6;

    // Establish client
    ros::ServiceClient client_realsense_all_images = n.serviceClient<acrv_realsense_ros::get_all_images>("/acrv_realsense_ros/get_realsense_all_images");

    // Service response and request containers
    acrv_realsense_ros::get_all_images srv_realsense_all_images;

    //File naming variables
    uint16_t capture_count = 0;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    while (true) {
        // Wait here for user to press enter to capture, or q + ENTER to exit program
        std::cout << "Press enter to capture checkerboard images..." << std::endl;
        int c;
        c = std::getchar();

        // Call all images service
        if (client_realsense_all_images.call(srv_realsense_all_images)) {

            // Obtain ir_image_raw
            cv_bridge::CvImagePtr cv_ptr_image_ir_image_raw;
            try
            {
                cv_ptr_image_ir_image_raw = cv_bridge::toCvCopy(srv_realsense_all_images.response.image_ir_image_raw,
                                             sensor_msgs::image_encodings::TYPE_16UC1);
            }
            catch(cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                break;
            }

            // Obtain rgb_image_raw
            cv_bridge::CvImagePtr cv_ptr_image_rgb_image_raw;
            try
            {
                cv_ptr_image_rgb_image_raw = cv_bridge::toCvCopy(srv_realsense_all_images.response.image_rgb_image_raw,
                                             sensor_msgs::image_encodings::BGR8);
            }
            catch(cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                break;
            }

            bool found_corners_color = false;
            bool found_corners_ir = false;
            std::vector<cv::Point2f> color_point_buf;
            std::vector<cv::Point2f> ir_point_buf;
            std::vector<std::vector<cv::Point2f>> color_image_points;
            std::vector<std::vector<cv::Point2f>> ir_image_points;

            found_corners_color = cv::findChessboardCorners(cv_ptr_image_rgb_image_raw->image, board_size, color_point_buf,
                CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);

            found_corners_ir = cv::findChessboardCorners(cv_ptr_image_ir_image_raw->image, board_size, ir_point_buf,
                CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);

            // Save if checkerboard is found
            if (found_corners_color && found_corners_ir) {
              ROS_INFO_STREAM("Checkerboard found.  Saving images...");
              std::stringstream ir_fn;
              ir_fn << "../data/ir/" << capture_count << "_ir.png";
              cv::imwrite(ir_fn.str(), cv_ptr_image_ir_image_raw->image, compression_params);

              std::stringstream colour_fn;
              colour_fn << "../data/color/" << capture_count << "_color.png";
              cv::imwrite(colour_fn.str(), cv_ptr_image_rgb_image_raw->image, compression_params);

              ROS_INFO_STREAM("Images saved.");
              capture_count++;
            }
            else {
              ROS_INFO_STREAM("No checkerboard detected.  Try again");
            }
        }
        else {
            ROS_INFO_STREAM("Failed to call service.  Check that the realsense camera is properly connected.");
        }

        capture_count++;
    }

    return 0;
}
