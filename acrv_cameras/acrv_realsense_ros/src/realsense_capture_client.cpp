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
Saves each separated capture into individual folders named img_<index no. (5 digits)>.
Each folder contains
- color.png
- ir.png
- depth.png
- cloud.pcl

*/

int main(int argc, char** argv)
{
    ros::init(argc, argv, "realsense_capture_client");
    ros::NodeHandle n;

    // Establish clients
    // ros::ServiceClient client_realsense_depth_camera_info = n.serviceClient<acrv_realsense_ros::get_camera_info>("/acrv_realsense_ros/get_realsense_depth_camera_info");
    // ros::ServiceClient client_realsense_depth_image_raw = n.serviceClient<acrv_realsense_ros::get_camera_image>("/acrv_realsense_ros/get_realsense_depth_image_raw");
    // ros::ServiceClient client_realsense_depth_image_raw_m = n.serviceClient<acrv_realsense_ros::get_camera_image>("/acrv_realsense_ros/get_realsense_depth_image_raw_m");
    // ros::ServiceClient client_realsense_depth_image_rect = n.serviceClient<acrv_realsense_ros::get_camera_image>("/acrv_realsense_ros/get_realsense_depth_image_rect");
    // ros::ServiceClient client_realsense_depth_registered_camera_info = n.serviceClient<acrv_realsense_ros::get_camera_info>("/acrv_realsense_ros/get_realsense_depth_registered_camera_info");
    // ros::ServiceClient client_realsense_depth_registered_image_rect = n.serviceClient<acrv_realsense_ros::get_camera_image>("/acrv_realsense_ros/get_realsense_depth_registered_image_rect");
    // ros::ServiceClient client_realsense_depth_registered_points = n.serviceClient<acrv_realsense_ros::get_camera_cloud>("/acrv_realsense_ros/get_realsense_depth_registered_points");
    // ros::ServiceClient client_realsense_ir_camera_info = n.serviceClient<acrv_realsense_ros::get_camera_info>("/acrv_realsense_ros/get_realsense_ir_camera_info");
    // ros::ServiceClient client_realsense_ir_image_raw = n.serviceClient<acrv_realsense_ros::get_camera_image>("/acrv_realsense_ros/get_realsense_ir_image_raw");
    // ros::ServiceClient client_realsense_ir_image_rect = n.serviceClient<acrv_realsense_ros::get_camera_image>("/acrv_realsense_ros/get_realsense_ir_image_rect");
    // ros::ServiceClient client_realsense_rgb_camera_info = n.serviceClient<acrv_realsense_ros::get_camera_info>("/acrv_realsense_ros/get_realsense_rgb_camera_info");
    // ros::ServiceClient client_realsense_rgb_image_raw = n.serviceClient<acrv_realsense_ros::get_camera_image>("/acrv_realsense_ros/get_realsense_rgb_image_raw");
    // ros::ServiceClient client_realsense_rgb_image_rect = n.serviceClient<acrv_realsense_ros::get_camera_image>("/acrv_realsense_ros/get_realsense_rgb_image_rect");
    ros::ServiceClient client_realsense_all_images = n.serviceClient<acrv_realsense_ros::get_all_images>("/acrv_realsense_ros/get_realsense_all_images");

    // Service response and request containers
    // acrv_realsense_ros::get_camera_info srv_realsense_depth_camera_info;
    // acrv_realsense_ros::get_camera_image srv_realsense_depth_image_raw;
    // acrv_realsense_ros::get_camera_image srv_realsense_depth_image_raw_m;
    // acrv_realsense_ros::get_camera_image srv_realsense_depth_image_rect;
    // acrv_realsense_ros::get_camera_info srv_realsense_depth_registered_camera_info;
    // acrv_realsense_ros::get_camera_image srv_realsense_depth_registered_image_rect;
    // acrv_realsense_ros::get_camera_cloud srv_realsense_depth_registered_points;
    // acrv_realsense_ros::get_camera_info srv_realsense_ir_camera_info;
    // acrv_realsense_ros::get_camera_image srv_realsense_ir_image_raw;
    // acrv_realsense_ros::get_camera_image srv_realsense_ir_image_rect;
    // acrv_realsense_ros::get_camera_info srv_realsense_rgb_camera_info;
    // acrv_realsense_ros::get_camera_image srv_realsense_rgb_image_raw;
    // acrv_realsense_ros::get_camera_image srv_realsense_rgb_image_rect;
    acrv_realsense_ros::get_all_images srv_realsense_all_images;

    //File naming variables
    uint16_t capture_count = 0;
    ROS_INFO_STREAM("Enter the index you would like to start at:");
    bool loop = true;
    while (loop) {
        std::string s;
        std::getline(std::cin, s);
        std::stringstream stream(s);
        if(stream >> capture_count) {
            loop = false;
            continue;
        }
        ROS_INFO_STREAM("Please enter an unsigned integer!");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    while (true) {
        // Wait here for user to press enter to capture, or q + ENTER to exit program
        std::cout << "Press enter to capture images in folder img_" << std::setfill('0') << std::setw(5) << capture_count << ".  Otherwise press 'q' to exit..." << std::endl;
        int c;
        c = std::getchar();
        if (c == 113) {  // 113 = q
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(),'\n');   // ignore until newline
            std::cout << "Exiting now..." << std::endl;
            break;
        }
        if (c != 10) {  // 10 = enter
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(),'\n');   // ignore until newline
        }

        // Create folder with directory
        std::stringstream folder_name;
        folder_name << "../data/img_" <<  std::setfill('0') << std::setw(5) << capture_count;
        const int dir_err = system(("mkdir -p " + folder_name.str()).c_str());
        if (-1 == dir_err)
        {
            printf("Error creating directory!\n");
            exit(1);
        }

        // Call all images service
        if (client_realsense_all_images.call(srv_realsense_all_images)) {

            // Save depth_image_raw.png
            cv_bridge::CvImagePtr cv_ptr_image_depth_image_raw;
            try
            {
                cv_ptr_image_depth_image_raw = cv_bridge::toCvCopy(srv_realsense_all_images.response.image_depth_image_raw,
                                             sensor_msgs::image_encodings::TYPE_16UC1);
            }
            catch(cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                break;
            }
            std::stringstream img_fn_image_depth_image_raw;
            img_fn_image_depth_image_raw << folder_name.str() << "/depth_image_raw.png";
            std::cout << "Saving image " << img_fn_image_depth_image_raw.str() << std::endl;
            std::cout << "Success: " << cv::imwrite(img_fn_image_depth_image_raw.str(), cv_ptr_image_depth_image_raw->image) << std::endl;

            // Save depth_image_raw_m.png
            cv_bridge::CvImagePtr cv_ptr_image_depth_image_raw_m;
            try
            {
                cv_ptr_image_depth_image_raw_m = cv_bridge::toCvCopy(srv_realsense_all_images.response.image_depth_image_raw_m,
                                             sensor_msgs::image_encodings::TYPE_16UC1);
            }
            catch(cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                break;
            }
            std::stringstream img_fn_image_depth_image_raw_m;
            img_fn_image_depth_image_raw_m << folder_name.str() << "/depth_image_raw_m.png";
            std::cout << "Saving image " << img_fn_image_depth_image_raw_m.str() << std::endl;
            std::cout << "Success: " << cv::imwrite(img_fn_image_depth_image_raw_m.str(), cv_ptr_image_depth_image_raw_m->image) << std::endl;

            // Save depth_image_rect.png
            cv_bridge::CvImagePtr cv_ptr_image_depth_image_rect;
            try
            {
                cv_ptr_image_depth_image_rect = cv_bridge::toCvCopy(srv_realsense_all_images.response.image_depth_image_rect,
                                             sensor_msgs::image_encodings::TYPE_16UC1);
            }
            catch(cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                break;
            }
            std::stringstream img_fn_image_depth_image_rect;
            img_fn_image_depth_image_rect << folder_name.str() << "/depth_image_rect.png";
            std::cout << "Saving image " << img_fn_image_depth_image_rect.str() << std::endl;
            std::cout << "Success: " << cv::imwrite(img_fn_image_depth_image_rect.str(), cv_ptr_image_depth_image_rect->image) << std::endl;

            // Save depth_registered_image_rect.png
            cv_bridge::CvImagePtr cv_ptr_image_depth_registered_image_rect;
            try
            {
                cv_ptr_image_depth_registered_image_rect = cv_bridge::toCvCopy(srv_realsense_all_images.response.image_depth_registered_image_rect,
                                             sensor_msgs::image_encodings::TYPE_16UC1);
            }
            catch(cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                break;
            }
            std::stringstream img_fn_image_depth_registered_image_rect;
            img_fn_image_depth_registered_image_rect << folder_name.str() << "/depth_registered_image_rect.png";
            std::cout << "Saving image " << img_fn_image_depth_registered_image_rect.str() << std::endl;
            std::cout << "Success: " << cv::imwrite(img_fn_image_depth_registered_image_rect.str(), cv_ptr_image_depth_registered_image_rect->image) << std::endl;

            // Save depth_registered_points.pcd
            pcl::PCLPointCloud2 pcl_pc2;
            pcl_conversions::toPCL(srv_realsense_all_images.response.cloud_depth_registered_points,pcl_pc2);
            pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

            std::stringstream img_fn;
            img_fn << folder_name.str() << "/depth_registered_points.pcd";
            std::cout << "Saving cloud " << img_fn.str() << std::endl;
            std::cout << "Success: " << pcl::io::savePCDFileASCII(img_fn.str(), *temp_cloud) << std::endl;

            // Save ir_image_raw.png
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
            std::stringstream img_fn_image_ir_image_raw;
            img_fn_image_ir_image_raw << folder_name.str() << "/ir_image_raw.png";
            std::cout << "Saving image " << img_fn_image_ir_image_raw.str() << std::endl;
            std::cout << "Success: " << cv::imwrite(img_fn_image_ir_image_raw.str(), cv_ptr_image_ir_image_raw->image) << std::endl;

            // Save ir_image_rect.png
            cv_bridge::CvImagePtr cv_ptr_image_ir_image_rect;
            try
            {
                cv_ptr_image_ir_image_rect = cv_bridge::toCvCopy(srv_realsense_all_images.response.image_ir_image_rect,
                                             sensor_msgs::image_encodings::TYPE_16UC1);
            }
            catch(cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                break;
            }
            std::stringstream img_fn_image_ir_image_rect;
            img_fn_image_ir_image_rect << folder_name.str() << "/ir_image_rect.png";
            std::cout << "Saving image " << img_fn_image_ir_image_rect.str() << std::endl;
            std::cout << "Success: " << cv::imwrite(img_fn_image_ir_image_rect.str(), cv_ptr_image_ir_image_rect->image) << std::endl;

            // Save rgb_image_raw.png
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
            std::stringstream img_fn_image_rgb_image_raw;
            img_fn_image_rgb_image_raw << folder_name.str() << "/rgb_image_raw.png";
            std::cout << "Saving image " << img_fn_image_rgb_image_raw.str() << std::endl;
            std::cout << "Success: " << cv::imwrite(img_fn_image_rgb_image_raw.str(), cv_ptr_image_rgb_image_raw->image) << std::endl;

            // Save rgb_image_rect.png
            cv_bridge::CvImagePtr cv_ptr_image_rgb_image_rect;
            try
            {
                cv_ptr_image_rgb_image_rect = cv_bridge::toCvCopy(srv_realsense_all_images.response.image_rgb_image_rect,
                                             sensor_msgs::image_encodings::BGR8);
            }
            catch(cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                break;
            }
            std::stringstream img_fn_image_rgb_image_rect;
            img_fn_image_rgb_image_rect << folder_name.str() << "/rgb_image_rect.png";
            std::cout << "Saving image " << img_fn_image_rgb_image_rect.str() << std::endl;
            std::cout << "Success: " << cv::imwrite(img_fn_image_rgb_image_rect.str(), cv_ptr_image_rgb_image_rect->image) << std::endl;
        }
        else {
            ROS_INFO_STREAM("Failed to call service.  Check that the realsense camera is properly connected.");
        }

        // // Call services
        // if (client_realsense_depth_camera_info.call(srv_realsense_depth_camera_info)) {
        //     // Save camera info into .txt
        // }
        // else {
        //     ROS_INFO_STREAM("Failed to call service: srv_realsense_depth_camera_info");
        // }
        //
        // if (client_realsense_depth_image_raw.call(srv_realsense_depth_image_raw)) {
        //     cv_bridge::CvImagePtr cv_ptr;
        //     try
        //     {
        //         cv_ptr = cv_bridge::toCvCopy(srv_realsense_depth_image_raw.response.camera_image,
        //                                      sensor_msgs::image_encodings::TYPE_16UC1);
        //     }
        //     catch(cv_bridge::Exception& e)
        //     {
        //         ROS_ERROR("cv_bridge exception: %s", e.what());
        //         break;
        //     }
        //     std::stringstream img_fn;
        //     img_fn << folder_name.str() << "/depth_image_raw.png";
        //     std::cout << "Saving image " << img_fn.str() << std::endl;
        //     std::cout << "Success: " << cv::imwrite(img_fn.str(), cv_ptr->image) << std::endl;
        // }
        // else {
        //     ROS_INFO_STREAM("Failed to call service: srv_realsense_depth_image_raw");
        // }
        //
        // if (client_realsense_depth_image_raw_m.call(srv_realsense_depth_image_raw_m)) {
        //     cv_bridge::CvImagePtr cv_ptr;
        //     try
        //     {
        //         cv_ptr = cv_bridge::toCvCopy(srv_realsense_depth_image_raw_m.response.camera_image,
        //                                      sensor_msgs::image_encodings::TYPE_16UC1);
        //     }
        //     catch(cv_bridge::Exception& e)
        //     {
        //         ROS_ERROR("cv_bridge exception: %s", e.what());
        //         break;
        //     }
        //     std::stringstream img_fn;
        //     img_fn << folder_name.str() << "/depth_image_raw_m.png";
        //     std::cout << "Saving image " << img_fn.str() << std::endl;
        //     std::cout << "Success: " << cv::imwrite(img_fn.str(), cv_ptr->image) << std::endl;
        // }
        // else {
        //     ROS_INFO_STREAM("Failed to call service: srv_realsense_depth_image_raw_m");
        // }
        //
        // if (client_realsense_depth_image_rect.call(srv_realsense_depth_image_rect)) {
        //     cv_bridge::CvImagePtr cv_ptr;
        //     try
        //     {
        //         cv_ptr = cv_bridge::toCvCopy(srv_realsense_depth_image_rect.response.camera_image,
        //                                      sensor_msgs::image_encodings::TYPE_16UC1);
        //     }
        //     catch(cv_bridge::Exception& e)
        //     {
        //         ROS_ERROR("cv_bridge exception: %s", e.what());
        //         break;
        //     }
        //     std::stringstream img_fn;
        //     img_fn << folder_name.str() << "/depth_image_rect.png";
        //     std::cout << "Saving image " << img_fn.str() << std::endl;
        //     std::cout << "Success: " << cv::imwrite(img_fn.str(), cv_ptr->image) << std::endl;
        // }
        // else {
        //     ROS_INFO_STREAM("Failed to call service: srv_realsense_depth_image_rect");
        // }
        //
        // // if (client_realsense_depth_registered_camera_info.call(srv_realsense_depth_registered_camera_info)) {
        // //     // Save camera info into .txt
        // // }
        // // else {
        // //     ROS_INFO_STREAM("Failed to call service: srv_realsense_depth_registered_camera_info");
        // // }
        //
        // if (client_realsense_depth_registered_image_rect.call(srv_realsense_depth_registered_image_rect)) {
        //     cv_bridge::CvImagePtr cv_ptr;
        //     try
        //     {
        //         cv_ptr = cv_bridge::toCvCopy(srv_realsense_depth_registered_image_rect.response.camera_image,
        //                                      sensor_msgs::image_encodings::TYPE_16UC1);
        //     }
        //     catch(cv_bridge::Exception& e)
        //     {
        //         ROS_ERROR("cv_bridge exception: %s", e.what());
        //         break;
        //     }
        //     std::stringstream img_fn;
        //     img_fn << folder_name.str() << "/depth_registered_image_rect.png";
        //     std::cout << "Saving image " << img_fn.str() << std::endl;
        //     std::cout << "Success: " << cv::imwrite(img_fn.str(), cv_ptr->image) << std::endl;
        // }
        // else {
        //     ROS_INFO_STREAM("Failed to call service: srv_realsense_depth_registered_image_rect");
        // }
        //
        // if (client_realsense_depth_registered_points.call(srv_realsense_depth_registered_points)) {
        //     pcl::PCLPointCloud2 pcl_pc2;
        //     pcl_conversions::toPCL(srv_realsense_depth_registered_points.response.camera_cloud,pcl_pc2);
        //     pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        //     pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
        //
        //     std::stringstream img_fn;
        //     img_fn << folder_name.str() << "/depth_registered_points.pcd";
        //     std::cout << "Saving cloud " << img_fn.str() << std::endl;
        //     std::cout << "Success: " << pcl::io::savePCDFileASCII(img_fn.str(), *temp_cloud) << std::endl;
        // }
        // else {
        //     ROS_INFO_STREAM("Failed to call service: srv_realsense_depth_registered_points");
        // }
        //
        // // if (client_realsense_ir_camera_info.call(srv_realsense_ir_camera_info)) {
        // //     // Save camera info into .txt
        // // }
        // // else {
        // //     ROS_INFO_STREAM("Failed to call service: srv_realsense_ir_camera_info");
        // // }
        //
        // if (client_realsense_ir_image_raw.call(srv_realsense_ir_image_raw)) {
        //     cv_bridge::CvImagePtr cv_ptr;
        //     try
        //     {
        //         cv_ptr = cv_bridge::toCvCopy(srv_realsense_ir_image_raw.response.camera_image,
        //                                      sensor_msgs::image_encodings::TYPE_16UC1);
        //     }
        //     catch(cv_bridge::Exception& e)
        //     {
        //         ROS_ERROR("cv_bridge exception: %s", e.what());
        //         break;
        //     }
        //     std::stringstream img_fn;
        //     img_fn << folder_name.str() << "/ir_image_raw.png";
        //     std::cout << "Saving image " << img_fn.str() << std::endl;
        //     std::cout << "Success: " << cv::imwrite(img_fn.str(), cv_ptr->image) << std::endl;
        // }
        // else {
        //     ROS_INFO_STREAM("Failed to call service: srv_realsense_ir_image_raw");
        // }
        //
        //
        // if (client_realsense_ir_image_rect.call(srv_realsense_ir_image_rect)) {
        //     cv_bridge::CvImagePtr cv_ptr;
        //     try
        //     {
        //         cv_ptr = cv_bridge::toCvCopy(srv_realsense_ir_image_rect.response.camera_image,
        //                                      sensor_msgs::image_encodings::TYPE_16UC1);
        //     }
        //     catch(cv_bridge::Exception& e)
        //     {
        //         ROS_ERROR("cv_bridge exception: %s", e.what());
        //         break;
        //     }
        //     std::stringstream img_fn;
        //     img_fn << folder_name.str() << "/ir_image_rect.png";
        //     std::cout << "Saving image " << img_fn.str() << std::endl;
        //     std::cout << "Success: " << cv::imwrite(img_fn.str(), cv_ptr->image) << std::endl;
        // }
        // else {
        //     ROS_INFO_STREAM("Failed to call service: srv_realsense_ir_image_rect");
        // }
        //
        //
        // //
        // // if (client_realsense_rgb_camera_info.call(srv_realsense_rgb_camera_info)) {
        // //     // Save camera info into .txt
        // // }
        // // else {
        // //     ROS_INFO_STREAM("Failed to call service: srv_realsense_rgb_camera_info");
        // // }
        //
        // if (client_realsense_rgb_image_raw.call(srv_realsense_rgb_image_raw)) {
        //     cv_bridge::CvImagePtr cv_ptr;
        //     try
        //     {
        //         cv_ptr = cv_bridge::toCvCopy(srv_realsense_rgb_image_raw.response.camera_image,
        //                                      sensor_msgs::image_encodings::BGR8);
        //     }
        //     catch(cv_bridge::Exception& e)
        //     {
        //         ROS_ERROR("cv_bridge exception: %s", e.what());
        //         break;
        //     }
        //     std::stringstream img_fn;
        //     img_fn << folder_name.str() << "/rgb_image_raw.png";
        //     std::cout << "Saving image " << img_fn.str() << std::endl;
        //     std::cout << "Success: " << cv::imwrite(img_fn.str(), cv_ptr->image) << std::endl;
        // }
        // else {
        //     ROS_INFO_STREAM("Failed to call service: srv_realsense_rgb_image_raw");
        // }
        //
        // if (client_realsense_rgb_image_rect.call(srv_realsense_rgb_image_rect)) {
        //     cv_bridge::CvImagePtr cv_ptr;
        //     try
        //     {
        //         cv_ptr = cv_bridge::toCvCopy(srv_realsense_rgb_image_rect.response.camera_image,
        //                                      sensor_msgs::image_encodings::BGR8);
        //     }
        //     catch(cv_bridge::Exception& e)
        //     {
        //         ROS_ERROR("cv_bridge exception: %s", e.what());
        //         break;
        //     }
        //     std::stringstream img_fn;
        //     img_fn << folder_name.str() << "/rgb_image_rect.png";
        //     std::cout << "Saving image " << img_fn.str() << std::endl;
        //     std::cout << "Success: " << cv::imwrite(img_fn.str(), cv_ptr->image) << std::endl;
        // }
        // else {
        //     ROS_INFO_STREAM("Failed to call service: srv_realsense_rgb_image_rect");
        // }

        capture_count++;
    }

    return 0;
}
