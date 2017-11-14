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
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cstdlib>

/*

Capture script for the RealSense Camera.
Saves each separated capture into individual folders named img_<index no. (5 digits)>.
Each folder contains
- color.png
- ir.png
- depth.png
- cloud.pcl

*/

int main() try
{
    // Turn on logging. We can separately enable logging to console or to file, and use different severity filters for each.
    rs::log_to_console(rs::log_severity::warn);
    //rs::log_to_file(rs::log_severity::debug, "librealsense.log");

    // Create a context object. This object owns the handles to all connected realsense devices.
    rs::context ctx;
    printf("There are %d connected RealSense devices.\n", ctx.get_device_count());
    if(ctx.get_device_count() == 0) return EXIT_FAILURE;

    // This tutorial will access only a single device, but it is trivial to extend to multiple devices
    rs::device * dev = ctx.get_device(0);
    printf("\nUsing device 0, an %s\n", dev->get_name());
    printf("    Serial number: %s\n", dev->get_serial());
    printf("    Firmware version: %s\n", dev->get_firmware_version());

    // Configure depth and color to run with the device's preferred settings
    dev->enable_stream(rs::stream::depth, 640, 480, rs::format::z16, 30);
    // NOTE I was first thinking we should collect both colour in 640x480 and 1920x1080 but the PointCloud
    // seems to look fine when processed off the full hd image and we can always downscale and crop to get
    // the smaller image if we want it
    // dev->enable_stream(rs::stream::color, 640, 480, rs::format::rgb8, 30);
    dev->enable_stream(rs::stream::color, 1920, 1080, rs::format::rgb8, 30);
    dev->enable_stream(rs::stream::infrared, 640, 480, rs::format::y8, 30);
    dev->start();

    //File naming variables
    uint16_t capture_count = 0;
    // std::string temp_name = "image";

    // // Enter item prefix to be appended onto output files
    // std::cout << "Enter item name:" << std::endl;
    // std::cin >> temp_name;

    std::cout << "Enter the index you would like to start at:" << std::endl;

    bool loop = true;

    while (loop) {
        std::string s;
        std::getline(std::cin, s);

        std::stringstream stream(s);

        if(stream >> capture_count) {
            loop = false;
            continue;
        }

        std::cout << "Please enter an unsigned integer!" << std::endl;
    }

    // std::cin.ignore(std::numeric_limits<std::streamsize>::max(),'\n');   // ignore until newline

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    while (true) {
        // TODO Wait here for user to press enter to capture again
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

        dev->wait_for_frames();

        // Retrieve our images
        uint16_t * depth_image = (uint16_t *)dev->get_frame_data(rs::stream::depth);
        uint8_t * color_image = (uint8_t *)dev->get_frame_data(rs::stream::color);
        uint16_t * ir_image = (uint16_t *)dev->get_frame_data(rs::stream::infrared);

        // Retrieve camera parameters for mapping between depth and color
        rs::intrinsics depth_intrin = dev->get_stream_intrinsics(rs::stream::depth);
        rs::extrinsics depth_to_color = dev->get_extrinsics(rs::stream::depth, rs::stream::color);
        rs::intrinsics color_intrin = dev->get_stream_intrinsics(rs::stream::color);
        rs::intrinsics ir_intrin = dev->get_stream_intrinsics(rs::stream::infrared);
        float scale = dev->get_depth_scale();

        // Point Cloud
        // Declare pointer
        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> color_reg_depth_cloud;
        // Allocate memory
        color_reg_depth_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
        // Setup the point cloud
        color_reg_depth_cloud->points.resize(depth_intrin.width * depth_intrin.height);
        color_reg_depth_cloud->header.frame_id = "depth_frame";
        color_reg_depth_cloud->is_dense = false;
        color_reg_depth_cloud->height = depth_intrin.height;
        color_reg_depth_cloud->width = depth_intrin.width;


        for (int dy=0; dy<depth_intrin.height; ++dy){
            for (int dx=0; dx<depth_intrin.width; ++dx){
                //obtain the depth value and apply scale factor
                uint16_t depth_value = depth_image[dy * depth_intrin.width + dx];
                float depth_in_meters = depth_value * scale;

                // Skip over pixels with a depth value of zero, which is used to indicate no data
                if(depth_value == 0) {
                    continue;
                }

                // Map from pixel coordinates in the depth image to pixel coordinates in the color image
                rs::float2 depth_pixel = {(float)dx, (float)dy};
                rs::float3 depth_point = depth_intrin.deproject(depth_pixel, depth_in_meters);
                rs::float3 color_point = depth_to_color.transform(depth_point);
                rs::float2 color_pixel = color_intrin.project(color_point);

                // Use the color from the nearest color pixel, ignore this point falls outside the color image
                const int cx = (int)std::round(color_pixel.x), cy = (int)std::round(color_pixel.y);

                if (!(cx < 0 || cy < 0 || cx >= color_intrin.width || cy >= color_intrin.height)){
                    //obtain pointer to current colour pixel
                    const uint8_t * color_ptr = color_image + (cy * color_intrin.width + cx) * 3;

                    // create xyzrgb point
                    pcl::PointXYZRGB point(*(color_ptr), *(color_ptr+1), *(color_ptr+2));
                    point.x = depth_point.x;
                    point.y = depth_point.y;
                    point.z = depth_point.z;

                    //add point to color_reg_depth_cloud
                    color_reg_depth_cloud->points[dy * depth_intrin.width + dx] = point;
                }
            }
        }

        std::stringstream cloud_fn;
        std::stringstream folder_name;

        // Create folder first
        folder_name << "../data/img_" <<  std::setfill('0') << std::setw(5) << capture_count;

        // const int dir_err = mkdir(folder_name.str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
        // if (-1 == dir_err)
        // {
        //     printf("Error creating directory!n");
        //     exit(1);
        // }



        const int dir_err = system(("mkdir -p " + folder_name.str()).c_str());
        if (-1 == dir_err)
        {
            printf("Error creating directory!\n");
            exit(1);
        }

        cloud_fn << folder_name.str() << "/cloud.pcd";
        pcl::io::savePCDFileASCII(cloud_fn.str(), *color_reg_depth_cloud);
        // Point Cloud
        std::vector<int> compression_params;
        compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
        compression_params.push_back(0);

        // Colour Image
        cv::Mat color_image_cv(color_intrin.height, color_intrin.width, CV_8UC3, color_image, cv::Mat::AUTO_STEP);
        std::stringstream colour_fn;
        colour_fn << folder_name.str() << "/colour.png";
        cv::cvtColor(color_image_cv, color_image_cv, CV_RGB2BGR);
        cv::imwrite(colour_fn.str(), color_image_cv, compression_params);
        // Colour Image

        // IR Image
        cv::Mat ir_image_cv(ir_intrin.height, ir_intrin.width, CV_8UC1, ir_image, cv::Mat::AUTO_STEP);
        std::stringstream ir_fn;
        ir_fn << folder_name.str() << "/ir.png";
        cv::imwrite(ir_fn.str(), ir_image_cv, compression_params);
        // IR Image

        // Depth Image
        cv::Mat depth_image_cv(depth_intrin.height, depth_intrin.width, CV_16UC1, depth_image, cv::Mat::AUTO_STEP);
        cv::Mat depth_image_cv_out(depth_intrin.height, depth_intrin.width, CV_16UC1);
        cv::Mat depth_image_cv_scaled(depth_intrin.height, depth_intrin.width, CV_64FC1);

        depth_image_cv.convertTo(depth_image_cv_scaled, CV_64FC1);
        depth_image_cv_scaled *= scale*65536.0/2.0;
        depth_image_cv_scaled.convertTo(depth_image_cv_out, CV_16UC1);

        // Depth values are in: (metres * 65536/2)
        // CV_16UC1 range is 0-65536, camera depth range is 0.3 to 2.0 m

        std::stringstream depth_fn;
        depth_fn << folder_name.str() << "/depth.png";
        cv::imwrite(depth_fn.str(), depth_image_cv_out, compression_params);
        // Depth Image

        capture_count++;
    }

    dev->stop();

    return EXIT_SUCCESS;
}
catch(const rs::error & e)
{
    // Method calls against librealsense objects may throw exceptions of type rs::error
    printf("rs::error was thrown when calling %s(%s):\n", e.get_failed_function().c_str(), e.get_failed_args().c_str());
    printf("    %s\n", e.what());
    return EXIT_FAILURE;
}
