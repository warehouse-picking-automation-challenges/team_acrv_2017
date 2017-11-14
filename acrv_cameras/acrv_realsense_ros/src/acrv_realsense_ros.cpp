#include <ros/ros.h>
#include <ros/console.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <librealsense/rs.hpp>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <opencv2/core/core.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>
#include <math.h>
#include <std_srvs/SetBool.h>
#include <thread>
#include "yaml-cpp/yaml.h"

ros::Publisher color_hd_pub, color_pub, color_info_pub, ir_pub, depth_pub, color_hd_info_pub, ir_info_pub, depth_info_pub;
sensor_msgs::CameraInfo rgb_hd_camera_info, rgb_camera_info, ir_camera_info, depth_camera_info;
rs::device * dev;

// cv Mat to sensor_msgs::Image conversion
sensor_msgs::ImagePtr cvImagetoMsg(cv::Mat &image,std::string encoding, std::string frame) {
    static int header_sequence_id = 0;

    std_msgs::Header header;

    header.seq = header_sequence_id++;
    header.stamp = ros::Time::now();
    header.frame_id = frame;

    cv_bridge::CvImage::Ptr cv_msg_ptr = cv_bridge::CvImage::Ptr(new cv_bridge::CvImage(header,encoding,image));

    return cv_msg_ptr->toImageMsg();
}

void rotate_cv_map_180(cv::Mat &image) {
    cv::Mat flip_1, flip_2;
    cv::flip(image, flip_1, 0);
    cv::flip(flip_1, flip_2, 1);
    image = flip_2;
}

// Function importing camera info matrices into sensor_msgs container
void load_camera_info_from_file(sensor_msgs::CameraInfo& camera_info, std::string path_to_camera_info_yaml) {
    ROS_INFO_STREAM(path_to_camera_info_yaml);

    YAML::Node camera_info_yaml = YAML::LoadFile(path_to_camera_info_yaml);

    // Select sections of YAML
    YAML::Node yaml_matrix_k = camera_info_yaml["camera_matrix"];
    YAML::Node yaml_matrix_d = camera_info_yaml["distortion_coefficients"];
    YAML::Node yaml_matrix_r = camera_info_yaml["rectification_matrix"];
    YAML::Node yaml_matrix_p = camera_info_yaml["projection_matrix"];

    std::size_t i;
    for (i = 0; i < yaml_matrix_k["data"].size(); ++i) {
        camera_info.K[i] = yaml_matrix_k["data"][i].as<double>();
    }
    camera_info.D.resize(yaml_matrix_d["data"].size());
    for (i = 0; i < yaml_matrix_d["data"].size(); ++i) {
        camera_info.D[i] = yaml_matrix_d["data"][i].as<double>();
    }
    for (i = 0; i < yaml_matrix_r["data"].size(); ++i) {
        camera_info.R[i] = yaml_matrix_r["data"][i].as<double>();
    }
    for (i = 0; i < yaml_matrix_p["data"].size(); ++i) {
        camera_info.P[i] = yaml_matrix_p["data"][i].as<double>();
    }
    camera_info.distortion_model = camera_info_yaml["distortion_model"].as<std::string>();
    camera_info.width = camera_info_yaml["image_width"].as<uint16_t>();
    camera_info.height = camera_info_yaml["image_height"].as<uint16_t>();
}

int main(int argc, char * argv[]) try {
    rs::log_to_console(rs::log_severity::warn);
    ros::init(argc, argv, "acrv_realsense_ros_node");
    ros::NodeHandle n("~");

    bool rotate_image_180;
    n.param<bool>("rotate_image_180", rotate_image_180, false);
    ROS_INFO_STREAM("\"rotate_image_180\" = " << rotate_image_180);

    bool is_serial_number_provided = false;
    std::string serial_number;
    if (n.getParam("serial_number", serial_number)) {
        ROS_INFO_STREAM("Serial number received: " << serial_number);
        is_serial_number_provided = true;
    } else {
        ROS_WARN_STREAM("\"serial_number\" not provided. Will look for any connected RealSense...");
    }

    std::unique_ptr<rs::context> ctx;

    if (is_serial_number_provided)
    {
        ctx = std::unique_ptr<rs::context>(new rs::context(serial_number));
    }
    else
    {
        ctx = std::unique_ptr<rs::context>(new rs::context());
    }

    int number_of_connected_devices = 0;
    number_of_connected_devices = ctx->get_device_count();
    if(number_of_connected_devices == 0) throw std::runtime_error("No device detected. Is it plugged in?");
    ROS_INFO_STREAM("There are " << number_of_connected_devices << " connected RealSense devices.");

    if (is_serial_number_provided) {
        for(int i=0; i<number_of_connected_devices; ++i) {
            try {
                dev = ctx->get_device(i);
                if (dev->get_serial() == serial_number) {
                    ROS_INFO_STREAM("Connecting to your requested device with serial number: " << serial_number);
                    break;
                }
            } catch(...) {
                ROS_ERROR_STREAM("Failed to get device!");
            }
        }
    } else {
        dev = ctx->get_device(0);
    }

    dev->enable_stream(rs::stream::depth, 640, 480, rs::format::z16, 30);
    dev->enable_stream(rs::stream::color, 1920, 1080, rs::format::rgb8, 30);
    dev->enable_stream(rs::stream::infrared, 640, 480, rs::format::y16, 30);

    dev->start();

    if(dev->supports_option(rs::option::color_enable_auto_white_balance)) {
        int value = 0;
        dev->set_option(rs::option::color_enable_auto_white_balance,value);

        // ROS_INFO_STREAM("Colour exposure supported, current value:" << dev->get_option(rs::option::color_enable_auto_white_balance));
    }

    std::string camera_name;
    if (n.getParam("camera_name", camera_name)) {
        ROS_INFO_STREAM("\"camera_name\" = " << camera_name);
    } else {
        ROS_ERROR_STREAM("\"camera_name\" not provided.");
        return 0;
    }

    image_transport::ImageTransport image_transport(n);

    color_hd_pub = n.advertise < sensor_msgs::Image > ("/" + camera_name + "/rgb_hd/image_raw", 1);
    color_pub = n.advertise < sensor_msgs::Image > ("/" + camera_name + "/rgb/image_raw", 1);
    depth_pub = n.advertise < sensor_msgs::Image > ("/" + camera_name + "/depth/image_raw", 1);
    ir_pub = n.advertise < sensor_msgs::Image > ("/" + camera_name + "/ir/image_raw", 1);
    color_hd_info_pub = n.advertise < sensor_msgs::CameraInfo > ("/" + camera_name + "/rgb_hd/camera_info", 1);
    color_info_pub = n.advertise < sensor_msgs::CameraInfo > ("/" + camera_name + "/rgb/camera_info", 1);
    depth_info_pub = n.advertise < sensor_msgs::CameraInfo > ("/" + camera_name + "/depth/camera_info", 1);
    ir_info_pub = n.advertise < sensor_msgs::CameraInfo > ("/" + camera_name + "/ir/camera_info", 1);

    rs::intrinsics depth_intrin = dev->get_stream_intrinsics(rs::stream::depth);
    rs::intrinsics color_intrin = dev->get_stream_intrinsics(rs::stream::color);
    rs::intrinsics ir_intrin = dev->get_stream_intrinsics(rs::stream::infrared);

    float scale = dev->get_depth_scale();

    std::string rgb_hd_camera_info_path, rgb_camera_info_path, depth_camera_info_path;
    if (n.getParam("rgb_hd_camera_info_path", rgb_hd_camera_info_path) &&
        n.getParam("depth_camera_info_path", depth_camera_info_path)) {
        ROS_INFO_STREAM("Found camera info yamls");

        load_camera_info_from_file(rgb_hd_camera_info, rgb_hd_camera_info_path);
        load_camera_info_from_file(depth_camera_info, depth_camera_info_path);

        // Convert from rgb_hd camera info to rgb camera info
        rgb_camera_info = rgb_hd_camera_info;
        rgb_camera_info.K[0] = rgb_camera_info.K[0]/3.0;  // fx
        rgb_camera_info.K[4] = rgb_camera_info.K[4]/3.0;  // fy
        rgb_camera_info.K[2] = rgb_camera_info.K[2]/3.0;  // ppx
        rgb_camera_info.K[5] = rgb_camera_info.K[5]/3.0;  // ppy
        rgb_camera_info.P[0] = rgb_camera_info.P[0]/3.0;  // fx
        rgb_camera_info.P[5] = rgb_camera_info.P[5]/3.0;  // fy
        rgb_camera_info.P[2] = rgb_camera_info.P[2]/3.0;  // ppx
        rgb_camera_info.P[6] = rgb_camera_info.P[6]/3.0;  // ppy
        rgb_camera_info.width = rgb_camera_info.width/3.0;
        rgb_camera_info.height = rgb_camera_info.height/3.0;
        ir_camera_info = depth_camera_info;
    } else {
        ROS_ERROR_STREAM("Camera info yamls not found");
        // return 0;
    }

    ros::Rate r(30);
    int emitter_enabled = 1;
    int prev_emitter_enabled = emitter_enabled;
    int laser_power;

    while(ros::ok()) {
        if(dev->is_streaming()) {
            if (n.getParam("emitter_enabled", emitter_enabled)) {
                if (emitter_enabled != prev_emitter_enabled) {
                    if (emitter_enabled == 1) {
                        laser_power = 16;
                    } else {
                        laser_power = 0;
                    }
                    dev->set_option(rs::option::f200_laser_power, laser_power);
                    prev_emitter_enabled = emitter_enabled;
                    for (int i = 0; i < 10; i++) {
                        dev->wait_for_frames();
                    }
                }
            }
            dev->wait_for_frames();
        } else {
            ros::spinOnce();
            r.sleep();
            continue;
        }

        ros::Time timeNow = ros::Time::now();

        if (depth_pub.getNumSubscribers() > 0 || depth_info_pub.getNumSubscribers() > 0) {
            uint16_t *depth_raw = (uint16_t *)dev->get_frame_data(rs::stream::depth);
            cv::Mat depth_image_raw(depth_intrin.height,depth_intrin.width,CV_16UC1,depth_raw, cv::Mat::AUTO_STEP);
            cv::Mat depth_image(depth_intrin.height,depth_intrin.width,CV_16UC1);
            cv::Mat depth_image_scaled(depth_intrin.height,depth_intrin.width,CV_64FC1);

            depth_image_raw.copyTo(depth_image);
            depth_image_raw.convertTo(depth_image_scaled,CV_64FC1);
            depth_image_scaled *= 1000.0f*scale;
            depth_image_scaled.convertTo(depth_image,CV_16UC1);

            if (rotate_image_180) {
                rotate_cv_map_180(depth_image);
            }

	        cv::medianBlur(depth_image, depth_image, 5);

            sensor_msgs::ImagePtr depthImage = cvImagetoMsg(depth_image, sensor_msgs::image_encodings::TYPE_16UC1, camera_name + "_depth_optical_frame");
            timeNow = ros::Time::now();
            depthImage->header.stamp = timeNow;
            depthImage->header.frame_id = camera_name + "_depth_optical_frame";
            depth_camera_info.header.stamp = timeNow;
            depth_camera_info.header.frame_id = camera_name + "_depth_optical_frame";
            depth_info_pub.publish(depth_camera_info);
            depth_pub.publish(*depthImage);
        }

        if (color_hd_pub.getNumSubscribers() > 0 || color_hd_info_pub.getNumSubscribers() > 0) {
            uint8_t * color_raw = (uint8_t *)dev->get_frame_data(rs::stream::color);
            cv::Mat color_image(color_intrin.height,color_intrin.width,CV_8UC3,color_raw, cv::Mat::AUTO_STEP);

            if (rotate_image_180) {
                rotate_cv_map_180(color_image);
            }

            sensor_msgs::ImagePtr colorImage = cvImagetoMsg(color_image, sensor_msgs::image_encodings::RGB8, camera_name + "_rgb_optical_frame");
            timeNow = ros::Time::now();
            colorImage->header.stamp = timeNow;
            colorImage->header.frame_id = camera_name + "_rgb_optical_frame";
            rgb_hd_camera_info.header.stamp = timeNow;
            rgb_hd_camera_info.header.frame_id = camera_name + "_rgb_optical_frame";
            color_hd_info_pub.publish(rgb_hd_camera_info);
            color_hd_pub.publish(*colorImage);
        }

        if (color_pub.getNumSubscribers() > 0 || color_info_pub.getNumSubscribers() > 0) {
            uint8_t * color_raw = (uint8_t *)dev->get_frame_data(rs::stream::color);
            cv::Mat color_hd_image(color_intrin.height,color_intrin.width,CV_8UC3,color_raw, cv::Mat::AUTO_STEP);
            cv::Mat color_image;
            cv::resize(color_hd_image, color_image, cv::Size(rgb_camera_info.width,rgb_camera_info.height), cv::INTER_CUBIC);

            if (rotate_image_180) {
                rotate_cv_map_180(color_image);
            }

            sensor_msgs::ImagePtr colorImage = cvImagetoMsg(color_image, sensor_msgs::image_encodings::RGB8, camera_name + "_rgb_optical_frame");
            timeNow = ros::Time::now();
            colorImage->header.stamp = timeNow;
            colorImage->header.frame_id = camera_name + "_rgb_optical_frame";
            rgb_camera_info.header.stamp = timeNow;
            rgb_camera_info.header.frame_id = camera_name + "_rgb_optical_frame";
            color_info_pub.publish(rgb_camera_info);
            color_pub.publish(*colorImage);
        }

        if (ir_pub.getNumSubscribers() > 0 || ir_info_pub.getNumSubscribers() > 0) {
            uint16_t * ir_raw = (uint16_t *)dev->get_frame_data(rs::stream::infrared);
            cv::Mat ir_image(ir_intrin.height,ir_intrin.width,CV_16UC1,ir_raw, cv::Mat::AUTO_STEP);

            if (rotate_image_180) {
                rotate_cv_map_180(ir_image);
            }

            sensor_msgs::ImagePtr irImage = cvImagetoMsg(ir_image, sensor_msgs::image_encodings::TYPE_16UC1, camera_name + "_depth_optical_frame");
            timeNow = ros::Time::now();
            irImage->header.stamp = timeNow;
            irImage->header.frame_id = camera_name + "_depth_optical_frame";
            ir_camera_info.header.stamp = timeNow;
            ir_camera_info.header.frame_id = camera_name + "_depth_optical_frame";
            ir_info_pub.publish(ir_camera_info);
            ir_pub.publish(*irImage);
        }

        ros::spinOnce();
        r.sleep();
    }
} catch(const rs::error & e) {  // If there is an error calling rs function
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
} catch(const std::exception & e) {  // some other error
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
