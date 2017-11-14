#include <librealsense/rs.hpp>
#include <math.h>
#include <sstream>
#include <string>
#include <iostream>
#include <iomanip>
#include <thread>
#include <cstdio>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


int main(int argc, char** argv) try
{
    // Turn on logging. We can separately enable logging to console or to file, and use different severity filters for each.
    rs::log_to_console(rs::log_severity::warn);

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

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // Retrieve camera parameters for mapping between depth and color
    rs::intrinsics depth_intrin = dev->get_stream_intrinsics(rs::stream::depth);
    rs::extrinsics depth_to_color = dev->get_extrinsics(rs::stream::depth, rs::stream::color);
    rs::intrinsics color_intrin = dev->get_stream_intrinsics(rs::stream::color);
    rs::intrinsics ir_intrin = dev->get_stream_intrinsics(rs::stream::infrared);
    float scale = dev->get_depth_scale();

    std::cout.precision(std::numeric_limits<double>::max_digits10);

    std::cout << std::endl;
    std::cout << "depth_intrin.model = " << std::fixed << depth_intrin.model() << std::endl;
    std::cout << "color_intrin.model = " << std::fixed << color_intrin.model() << std::endl;
    std::cout << std::endl;
    std::cout << "float scale = " << std::fixed << scale << ";" << std::endl;
    std::cout << "depth_intrin.height = " << std::fixed << depth_intrin.height << ";" << std::endl;
    std::cout << "depth_intrin.width = " << std::fixed << depth_intrin.width << ";" << std::endl;
    std::cout << "depth_intrin.fx = " << std::fixed << depth_intrin.fx << ";" << std::endl;
    std::cout << "depth_intrin.fy = " << std::fixed << depth_intrin.fy << ";" << std::endl;
    std::cout << "depth_intrin.ppx = " << std::fixed << depth_intrin.ppx << ";" << std::endl;
    std::cout << "depth_intrin.ppy = " << std::fixed << depth_intrin.ppy << ";" << std::endl;
    std::cout << "depth_intrin.coeffs[0] = " << std::fixed << depth_intrin.coeffs[0] << ";" << std::endl;
    std::cout << "depth_intrin.coeffs[1] = " << std::fixed << depth_intrin.coeffs[1] << ";" << std::endl;
    std::cout << "depth_intrin.coeffs[2] = " << std::fixed << depth_intrin.coeffs[2] << ";" << std::endl;
    std::cout << "depth_intrin.coeffs[3] = " << std::fixed << depth_intrin.coeffs[3] << ";" << std::endl;
    std::cout << "depth_intrin.coeffs[4] = " << std::fixed << depth_intrin.coeffs[4] << ";" << std::endl;
    std::cout << "color_intrin.height = " << std::fixed << color_intrin.height << ";" << std::endl;
    std::cout << "color_intrin.width = " << std::fixed << color_intrin.width << ";" << std::endl;
    std::cout << "color_intrin.fx = " << std::fixed << color_intrin.fx << ";" << std::endl;
    std::cout << "color_intrin.fy = " << std::fixed << color_intrin.fy << ";" << std::endl;
    std::cout << "color_intrin.ppx = " << std::fixed << color_intrin.ppx << ";" << std::endl;
    std::cout << "color_intrin.ppy = " << std::fixed << color_intrin.ppy << ";" << std::endl;
    std::cout << "color_intrin.coeffs[0] = " << std::fixed << color_intrin.coeffs[0] << ";" << std::endl;
    std::cout << "color_intrin.coeffs[1] = " << std::fixed << color_intrin.coeffs[1] << ";" << std::endl;
    std::cout << "color_intrin.coeffs[2] = " << std::fixed << color_intrin.coeffs[2] << ";" << std::endl;
    std::cout << "color_intrin.coeffs[3] = " << std::fixed << color_intrin.coeffs[3] << ";" << std::endl;
    std::cout << "color_intrin.coeffs[4] = " << std::fixed << color_intrin.coeffs[4] << ";" << std::endl;
    std::cout << "depth_to_color.rotation[0] = " << std::fixed << depth_to_color.rotation[0] << ";" << std::endl;
    std::cout << "depth_to_color.rotation[1] = " << std::fixed << depth_to_color.rotation[1] << ";" << std::endl;
    std::cout << "depth_to_color.rotation[2] = " << std::fixed << depth_to_color.rotation[2] << ";" << std::endl;
    std::cout << "depth_to_color.rotation[3] = " << std::fixed << depth_to_color.rotation[3] << ";" << std::endl;
    std::cout << "depth_to_color.rotation[4] = " << std::fixed << depth_to_color.rotation[4] << ";" << std::endl;
    std::cout << "depth_to_color.rotation[5] = " << std::fixed << depth_to_color.rotation[5] << ";" << std::endl;
    std::cout << "depth_to_color.rotation[6] = " << std::fixed << depth_to_color.rotation[6] << ";" << std::endl;
    std::cout << "depth_to_color.rotation[7] = " << std::fixed << depth_to_color.rotation[7] << ";" << std::endl;
    std::cout << "depth_to_color.rotation[8] = " << std::fixed << depth_to_color.rotation[8] << ";" << std::endl;
    std::cout << "depth_to_color.translation[0] = " << std::fixed << depth_to_color.translation[0] << ";" << std::endl;
    std::cout << "depth_to_color.translation[1] = " << std::fixed << depth_to_color.translation[1] << ";" << std::endl;
    std::cout << "depth_to_color.translation[2] = " << std::fixed << depth_to_color.translation[2] << ";" << std::endl;
    std::cout << std::endl;

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
