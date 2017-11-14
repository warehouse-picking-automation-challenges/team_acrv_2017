#include <librealsense/rs.hpp>
#include <math.h>
#include <sstream>
#include <string>
#include <iostream>
#include <iomanip>
#include <thread>
#include <cstdio>
#include <time.h>

#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

/*

Errors that currently occur:

rs.warn: usb:20:1: uvc_open2(...) returned Not supported

capture_data(43080,0x7fffb44cc3c0) malloc: *** error for object 0x7f9259500e28: incorrect checksum for freed object - object was probably modified after being freed.
*** set a breakpoint in malloc_error_break to debug
Abort trap: 6

Segmentation fault: 11

*/

/*
NOTE
Assumes 1x Intel RealSense SR300
*/

// Create a context object. This object owns the handles to all connected realsense devices.
rs::context ctx;

void my_handler(int s){
    printf("\nExiting safely...\n");
    rs::device * dev = ctx.get_device(0);
    dev->wait_for_frames();
    dev->stop();
    exit(1);
}

int main() try
{
    struct sigaction sigIntHandler;

    sigIntHandler.sa_handler = my_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, NULL);

    // Params for saving png
    std::vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(0);

    // Turn on logging. We can separately enable logging to console or to file, and use different severity filters for each.
    rs::log_to_console(rs::log_severity::warn);
    //rs::log_to_file(rs::log_severity::debug, "librealsense.log");

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

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    cv::Size board_size;
    board_size.width = 8;
    board_size.height = 6;

    int capture_count = 0;

    clock_t prevTimestamp = 0;
    int delay = 1000;

    while (true) {
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

        cv::Mat color_image_cv(color_intrin.height, color_intrin.width, CV_8UC3, color_image, cv::Mat::AUTO_STEP);
        cv::Mat ir_image_cv(ir_intrin.height, ir_intrin.width, CV_8UC1, ir_image, cv::Mat::AUTO_STEP);
        cv::Mat color_image_cv_for_display = color_image_cv.clone();
        cv::Mat ir_image_cv_for_display = ir_image_cv.clone();

        bool found_corners_color = false;
        bool found_corners_ir = false;
        std::vector<cv::Point2f> color_point_buf;
        std::vector<cv::Point2f> ir_point_buf;
        std::vector<std::vector<cv::Point2f>> color_image_points;
        std::vector<std::vector<cv::Point2f>> ir_image_points;

        found_corners_color = cv::findChessboardCorners(color_image_cv, board_size, color_point_buf,
            // CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
            CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);

        found_corners_ir = cv::findChessboardCorners(ir_image_cv, board_size, ir_point_buf,
            // CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
            CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);

        if (found_corners_color && found_corners_ir) {
            // Color
            cv::Mat color_image_cv_gray;
            cv::cvtColor(color_image_cv, color_image_cv_gray, cv::COLOR_RGB2GRAY);

            cv::cornerSubPix(color_image_cv_gray, color_point_buf, cv::Size(11,11),
                cv::Size(-1,-1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1 ));

            // color_image_points.push_back(color_point_buf);

            cv::drawChessboardCorners(color_image_cv_for_display, board_size, cv::Mat(color_point_buf), found_corners_color);

            // IR
            cv::cornerSubPix(ir_image_cv, ir_point_buf, cv::Size(11,11),
                cv::Size(-1,-1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1 ));

            // ir_image_points.push_back(ir_point_buf);

            cv::drawChessboardCorners(ir_image_cv_for_display, board_size, cv::Mat(ir_point_buf), found_corners_ir);

            // Save images
            if (clock() - prevTimestamp > delay*1e-3*CLOCKS_PER_SEC) {
                // Color Image
                std::stringstream colour_fn;
                colour_fn << "../data/color/" << capture_count << "_color.png";
                cv::cvtColor(color_image_cv, color_image_cv, CV_RGB2BGR);
                cv::imwrite(colour_fn.str(), color_image_cv, compression_params);
                // Color Image

                // IR Image
                std::stringstream ir_fn;
                ir_fn << "../data/ir/" << capture_count << "_ir.png";
                cv::imwrite(ir_fn.str(), ir_image_cv, compression_params);
                // IR Image

                prevTimestamp = clock();
                capture_count++;
                printf("Capture %d taken\n", capture_count);
            }
        }

        cv::Mat color_image_cv_for_display_bgr;
        cv::cvtColor(color_image_cv_for_display, color_image_cv_for_display_bgr, cv::COLOR_RGB2BGR);
        cv::Mat color_image_cv_for_display_downscaled;
        cv::resize(color_image_cv_for_display_bgr, color_image_cv_for_display_downscaled, cv::Size(640, 480));

        cv::namedWindow("color", cv::WINDOW_AUTOSIZE);
        cv::imshow("color", color_image_cv_for_display_downscaled);

        cv::namedWindow("ir", cv::WINDOW_AUTOSIZE);
        cv::imshow("ir", ir_image_cv_for_display);

        cv::waitKey(1);
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
