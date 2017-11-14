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

/**
* This ROS node provides services to subcribe to the realsense camera topics
* and provide a single instance of data.
*/

// Class declaration
class ImageCapture {
private:
    ros::NodeHandle nh_;

    // Server objects
    ros::ServiceServer srv_realsense_depth_camera_info;
    ros::ServiceServer srv_realsense_depth_image_raw;
    ros::ServiceServer srv_realsense_depth_image_raw_m;
    ros::ServiceServer srv_realsense_depth_image_rect;
    ros::ServiceServer srv_realsense_depth_registered_camera_info;
    ros::ServiceServer srv_realsense_depth_registered_image_rect;
    ros::ServiceServer srv_realsense_depth_registered_points;
    ros::ServiceServer srv_realsense_ir_camera_info;
    ros::ServiceServer srv_realsense_ir_image_raw;
    ros::ServiceServer srv_realsense_ir_image_rect;
    ros::ServiceServer srv_realsense_rgb_camera_info;
    ros::ServiceServer srv_realsense_rgb_image_raw;
    ros::ServiceServer srv_realsense_rgb_image_rect;
    ros::ServiceServer srv_realsense_all_images;

public:
    ImageCapture(void);
    void start(void);

    // Subscriber callback declarations
    void sub_realsense_depth_camera_info(const sensor_msgs::CameraInfo& msg);
    void sub_realsense_depth_image_raw(const sensor_msgs::Image& msg);
    void sub_realsense_depth_image_raw_m(const sensor_msgs::Image& msg);
    void sub_realsense_depth_image_rect(const sensor_msgs::Image& msg);
    void sub_realsense_depth_registered_camera_info(
        const sensor_msgs::CameraInfo& msg);
    void sub_realsense_depth_registered_image_rect(const sensor_msgs::Image& msg);
    void
    sub_realsense_depth_registered_points(const sensor_msgs::PointCloud2& msg);
    void sub_realsense_ir_camera_info(const sensor_msgs::CameraInfo& msg);
    void sub_realsense_ir_image_raw(const sensor_msgs::Image& msg);
    void sub_realsense_ir_image_rect(const sensor_msgs::Image& msg);
    void sub_realsense_rgb_camera_info(const sensor_msgs::CameraInfo& msg);
    void sub_realsense_rgb_image_raw(const sensor_msgs::Image& msg);
    void sub_realsense_rgb_image_rect(const sensor_msgs::Image& msg);

    // Subscriber service callback declarations
    bool get_realsense_depth_camera_info(
        acrv_realsense_ros::get_camera_info::Request& req,
        acrv_realsense_ros::get_camera_info::Response& res);
    bool get_realsense_depth_image_raw(
        acrv_realsense_ros::get_camera_image::Request& req,
        acrv_realsense_ros::get_camera_image::Response& res);
    bool get_realsense_depth_image_raw_m(
        acrv_realsense_ros::get_camera_image::Request& req,
        acrv_realsense_ros::get_camera_image::Response& res);
    bool get_realsense_depth_image_rect(
        acrv_realsense_ros::get_camera_image::Request& req,
        acrv_realsense_ros::get_camera_image::Response& res);
    bool get_realsense_depth_registered_camera_info(
        acrv_realsense_ros::get_camera_info::Request& req,
        acrv_realsense_ros::get_camera_info::Response& res);
    bool get_realsense_depth_registered_image_rect(
        acrv_realsense_ros::get_camera_image::Request& req,
        acrv_realsense_ros::get_camera_image::Response& res);
    bool get_realsense_depth_registered_points(
        acrv_realsense_ros::get_camera_cloud::Request& req,
        acrv_realsense_ros::get_camera_cloud::Response& res);
    bool get_realsense_ir_camera_info(
        acrv_realsense_ros::get_camera_info::Request& req,
        acrv_realsense_ros::get_camera_info::Response& res);
    bool get_realsense_ir_image_raw(
        acrv_realsense_ros::get_camera_image::Request& req,
        acrv_realsense_ros::get_camera_image::Response& res);
    bool get_realsense_ir_image_rect(
        acrv_realsense_ros::get_camera_image::Request& req,
        acrv_realsense_ros::get_camera_image::Response& res);
    bool get_realsense_rgb_camera_info(
        acrv_realsense_ros::get_camera_info::Request& req,
        acrv_realsense_ros::get_camera_info::Response& res);
    bool get_realsense_rgb_image_raw(
        acrv_realsense_ros::get_camera_image::Request& req,
        acrv_realsense_ros::get_camera_image::Response& res);
    bool get_realsense_rgb_image_rect(
        acrv_realsense_ros::get_camera_image::Request& req,
        acrv_realsense_ros::get_camera_image::Response& res);
    bool get_realsense_all_images(
        acrv_realsense_ros::get_all_images::Request& req,
        acrv_realsense_ros::get_all_images::Response& res);

    // Variables to store service instances
    sensor_msgs::CameraInfo realsense_depth_camera_info;
    sensor_msgs::Image realsense_depth_image_raw;
    sensor_msgs::Image realsense_depth_image_raw_m;
    sensor_msgs::Image realsense_depth_image_rect;
    sensor_msgs::CameraInfo realsense_depth_registered_camera_info;
    sensor_msgs::Image realsense_depth_registered_image_rect;
    sensor_msgs::PointCloud2 realsense_depth_registered_points;
    sensor_msgs::CameraInfo realsense_ir_camera_info;
    sensor_msgs::Image realsense_ir_image_raw;
    sensor_msgs::Image realsense_ir_image_rect;
    sensor_msgs::CameraInfo realsense_rgb_camera_info;
    sensor_msgs::Image realsense_rgb_image_raw;
    sensor_msgs::Image realsense_rgb_image_rect;
};

// Class constructor - advertises services
ImageCapture::ImageCapture(void)
    : nh_("~")
{
    srv_realsense_depth_camera_info = nh_.advertiseService(
        std::string("/acrv_realsense_ros/get_realsense_depth_camera_info"),
        &ImageCapture::get_realsense_depth_camera_info, this);
    srv_realsense_depth_image_raw = nh_.advertiseService(
        std::string("/acrv_realsense_ros/get_realsense_depth_image_raw"),
        &ImageCapture::get_realsense_depth_image_raw, this);
    srv_realsense_depth_image_raw_m = nh_.advertiseService(
        std::string("/acrv_realsense_ros/get_realsense_depth_image_raw_m"),
        &ImageCapture::get_realsense_depth_image_raw_m, this);
    srv_realsense_depth_image_rect = nh_.advertiseService(
        std::string("/acrv_realsense_ros/get_realsense_depth_image_rect"),
        &ImageCapture::get_realsense_depth_image_rect, this);
    srv_realsense_depth_registered_camera_info = nh_.advertiseService(
        std::string(
            "/acrv_realsense_ros/get_realsense_depth_registered_camera_info"),
        &ImageCapture::get_realsense_depth_registered_camera_info, this);
    srv_realsense_depth_registered_image_rect = nh_.advertiseService(
        std::string(
            "/acrv_realsense_ros/get_realsense_depth_registered_image_rect"),
        &ImageCapture::get_realsense_depth_registered_image_rect, this);
    srv_realsense_depth_registered_points = nh_.advertiseService(
        std::string("/acrv_realsense_ros/get_realsense_depth_registered_points"),
        &ImageCapture::get_realsense_depth_registered_points, this);
    srv_realsense_ir_camera_info = nh_.advertiseService(
        std::string("/acrv_realsense_ros/get_realsense_ir_camera_info"),
        &ImageCapture::get_realsense_ir_camera_info, this);
    srv_realsense_ir_image_raw = nh_.advertiseService(
        std::string("/acrv_realsense_ros/get_realsense_ir_image_raw"),
        &ImageCapture::get_realsense_ir_image_raw, this);
    srv_realsense_ir_image_rect = nh_.advertiseService(
        std::string("/acrv_realsense_ros/get_realsense_ir_image_rect"),
        &ImageCapture::get_realsense_ir_image_rect, this);
    srv_realsense_rgb_camera_info = nh_.advertiseService(
        std::string("/acrv_realsense_ros/get_realsense_rgb_camera_info"),
        &ImageCapture::get_realsense_rgb_camera_info, this);
    srv_realsense_rgb_image_raw = nh_.advertiseService(
        std::string("/acrv_realsense_ros/get_realsense_rgb_image_raw"),
        &ImageCapture::get_realsense_rgb_image_raw, this);
    srv_realsense_rgb_image_rect = nh_.advertiseService(
        std::string("/acrv_realsense_ros/get_realsense_rgb_image_rect"),
        &ImageCapture::get_realsense_rgb_image_rect, this);
    srv_realsense_all_images = nh_.advertiseService(
        std::string("/acrv_realsense_ros/get_realsense_all_images"),
        &ImageCapture::get_realsense_all_images, this);
}

// Subscriber callbaccks
void ImageCapture::sub_realsense_depth_camera_info(
    const sensor_msgs::CameraInfo& msg)
{
    this->realsense_depth_camera_info = msg;
    ROS_INFO_STREAM("Depth camera info received.");
}

void ImageCapture::sub_realsense_depth_image_raw(
    const sensor_msgs::Image& msg)
{
    this->realsense_depth_image_raw = msg;
    ROS_INFO_STREAM("Depth image raw received.");
}

void ImageCapture::sub_realsense_depth_image_raw_m(
    const sensor_msgs::Image& msg)
{
    this->realsense_depth_image_raw_m = msg;
    ROS_INFO_STREAM("Depth image raw m received.");
}

void ImageCapture::sub_realsense_depth_image_rect(
    const sensor_msgs::Image& msg)
{
    this->realsense_depth_image_rect = msg;
    ROS_INFO_STREAM("Depth image rect received.");
}

void ImageCapture::sub_realsense_depth_registered_camera_info(
    const sensor_msgs::CameraInfo& msg)
{
    this->realsense_depth_registered_camera_info = msg;
    ROS_INFO_STREAM("Depth registered camera info received.");
}

void ImageCapture::sub_realsense_depth_registered_image_rect(
    const sensor_msgs::Image& msg)
{
    this->realsense_depth_registered_image_rect = msg;
    ROS_INFO_STREAM("Depth registered image rect received.");
}

void ImageCapture::sub_realsense_depth_registered_points(
    const sensor_msgs::PointCloud2& msg)
{
    this->realsense_depth_registered_points = msg;
    ROS_INFO_STREAM("Depth registered points received.");
}

void ImageCapture::sub_realsense_ir_camera_info(
    const sensor_msgs::CameraInfo& msg)
{
    this->realsense_ir_camera_info = msg;
    ROS_INFO_STREAM("ir camera info received.");
}

void ImageCapture::sub_realsense_ir_image_raw(const sensor_msgs::Image& msg)
{
    this->realsense_ir_image_raw = msg;
    ROS_INFO_STREAM("ir image raw received.");
}

void ImageCapture::sub_realsense_ir_image_rect(const sensor_msgs::Image& msg)
{
    this->realsense_ir_image_rect = msg;
    ROS_INFO_STREAM("ir image rect received.");
}

void ImageCapture::sub_realsense_rgb_camera_info(
    const sensor_msgs::CameraInfo& msg)
{
    this->realsense_rgb_camera_info = msg;
    ROS_INFO_STREAM("rgb camera info received.");
}

void ImageCapture::sub_realsense_rgb_image_raw(const sensor_msgs::Image& msg)
{
    this->realsense_rgb_image_raw = msg;
    ROS_INFO_STREAM("rgb image raw received.");
}

void ImageCapture::sub_realsense_rgb_image_rect(const sensor_msgs::Image& msg)
{
    this->realsense_rgb_image_rect = msg;
    ROS_INFO_STREAM("rgb image rect received.");
}

// Service callback for depth camera info
bool ImageCapture::get_realsense_depth_camera_info(
    acrv_realsense_ros::get_camera_info::Request& req,
    acrv_realsense_ros::get_camera_info::Response& res)
{
    // Create new node handle and subscriber for one time capture instance
    ros::CallbackQueue pcbq;
    ros::NodeHandle pnh;
    pnh.setCallbackQueue(&pcbq);
    auto sub = pnh.subscribe(std::string("/realsense/depth/camera_info"), 1,
        &ImageCapture::sub_realsense_depth_camera_info, this);
    ROS_INFO_STREAM("Waiting for depth camera info.");
    auto now = ros::Time::now();
    while (ros::Time::now() - now < ros::Duration(1.0)) {
        pcbq.callAvailable();
    }
    res.camera_info = this->realsense_depth_camera_info;
    ROS_INFO_STREAM("Depth camera info captured.");
    return true;
}

// Service callback for depth image raw
bool ImageCapture::get_realsense_depth_image_raw(
    acrv_realsense_ros::get_camera_image::Request& req,
    acrv_realsense_ros::get_camera_image::Response& res)
{
    // Create new node handle and subscriber for one time capture instance
    ros::CallbackQueue pcbq;
    ros::NodeHandle pnh;
    pnh.setCallbackQueue(&pcbq);
    auto sub = pnh.subscribe(std::string("/realsense/depth/image_raw"), 1,
        &ImageCapture::sub_realsense_depth_image_raw, this);
    ROS_INFO_STREAM("Waiting for depth image raw...");
    auto now = ros::Time::now();
    while (ros::Time::now() - now < ros::Duration(1.0)) {
        pcbq.callAvailable();
    }
    res.camera_image = this->realsense_depth_image_raw;
    ROS_INFO_STREAM("Depth image raw captured.");
    return true;
}

// Service callback for depth image raw converted (m)
bool ImageCapture::get_realsense_depth_image_raw_m(
    acrv_realsense_ros::get_camera_image::Request& req,
    acrv_realsense_ros::get_camera_image::Response& res)
{
    // Create new node handle and subscriber for one time capture instance
    ros::CallbackQueue pcbq;
    ros::NodeHandle pnh;
    pnh.setCallbackQueue(&pcbq);
    auto sub = pnh.subscribe(std::string("/realsense/depth/image_raw_m"), 1,
        &ImageCapture::sub_realsense_depth_image_raw_m, this);
    ROS_INFO_STREAM("Waiting for depth image raw converted (m)...");
    auto now = ros::Time::now();
    while (ros::Time::now() - now < ros::Duration(1.0)) {
        pcbq.callAvailable();
    }
    res.camera_image = this->realsense_depth_image_raw_m;
    ROS_INFO_STREAM("Depth image raw converted (m) captured.");
    return true;
}

// Service callback for depth image rect
bool ImageCapture::get_realsense_depth_image_rect(
    acrv_realsense_ros::get_camera_image::Request& req,
    acrv_realsense_ros::get_camera_image::Response& res)
{
    // Create new node handle and subscriber for one time capture instance
    ros::CallbackQueue pcbq;
    ros::NodeHandle pnh;
    pnh.setCallbackQueue(&pcbq);
    auto sub = pnh.subscribe(std::string("/realsense/depth/image_rect"), 1,
        &ImageCapture::sub_realsense_depth_image_rect, this);
    ROS_INFO_STREAM("Waiting for rectified depth image...");
    auto now = ros::Time::now();
    while (ros::Time::now() - now < ros::Duration(1.0)) {
        pcbq.callAvailable();
    }
    res.camera_image = this->realsense_depth_image_rect;
    ROS_INFO_STREAM("Depth image rect captured.");
    return true;
}

// Service callback for depth registered camera info
bool ImageCapture::get_realsense_depth_registered_camera_info(
    acrv_realsense_ros::get_camera_info::Request& req,
    acrv_realsense_ros::get_camera_info::Response& res)
{
    // Create new node handle and subscriber for one time capture instance
    ros::CallbackQueue pcbq;
    ros::NodeHandle pnh;
    pnh.setCallbackQueue(&pcbq);
    auto sub = pnh.subscribe(
        std::string("/realsense/depth_registered/camera_info"), 1,
        &ImageCapture::sub_realsense_depth_registered_camera_info, this);
    ROS_INFO_STREAM("Waiting for depth registered camera info...");
    auto now = ros::Time::now();
    while (ros::Time::now() - now < ros::Duration(1.0)) {
        pcbq.callAvailable();
    }
    res.camera_info = this->realsense_depth_registered_camera_info;
    ROS_INFO_STREAM("Depth registered camera info captured.");
    return true;
}

// Service callback for depth registered image rect
bool ImageCapture::get_realsense_depth_registered_image_rect(
    acrv_realsense_ros::get_camera_image::Request& req,
    acrv_realsense_ros::get_camera_image::Response& res)
{
    // Create new node handle and subscriber for one time capture instance
    ros::CallbackQueue pcbq;
    ros::NodeHandle pnh;
    pnh.setCallbackQueue(&pcbq);
    auto sub = pnh.subscribe(
        std::string("/realsense/depth_registered/image_rect"), 1,
        &ImageCapture::sub_realsense_depth_registered_image_rect, this);
    ROS_INFO_STREAM("Waiting for rectified depth registered image...");
    auto now = ros::Time::now();
    while (ros::Time::now() - now < ros::Duration(1.0)) {
        pcbq.callAvailable();
    }
    res.camera_image = this->realsense_depth_registered_image_rect;
    ROS_INFO_STREAM("Depth registered image rect captured.");
    return true;
}

// Service callback for depth registered points
bool ImageCapture::get_realsense_depth_registered_points(
    acrv_realsense_ros::get_camera_cloud::Request& req,
    acrv_realsense_ros::get_camera_cloud::Response& res)
{
    // Create new node handle and subscriber for one time capture instance
    ros::CallbackQueue pcbq;
    ros::NodeHandle pnh;
    pnh.setCallbackQueue(&pcbq);
    auto sub = pnh.subscribe(std::string("/realsense/depth_registered/points"), 1,
        &ImageCapture::sub_realsense_depth_registered_points, this);
    ROS_INFO_STREAM("Waiting for rectified depth registered point cloud...");
    auto now = ros::Time::now();
    while (ros::Time::now() - now < ros::Duration(1.0)) {
        pcbq.callAvailable();
    }
    res.camera_cloud = this->realsense_depth_registered_points;
    ROS_INFO_STREAM("Depth registered point cloud captured.");
    return true;
}

// Service callback for ir camera info
bool ImageCapture::get_realsense_ir_camera_info(
    acrv_realsense_ros::get_camera_info::Request& req,
    acrv_realsense_ros::get_camera_info::Response& res)
{
    // Create new node handle and subscriber for one time capture instance
    ros::CallbackQueue pcbq;
    ros::NodeHandle pnh;
    pnh.setCallbackQueue(&pcbq);
    auto sub = pnh.subscribe(std::string("/realsense/ir/camera_info"), 1,
        &ImageCapture::sub_realsense_ir_camera_info, this);
    ROS_INFO_STREAM("Waiting for ir camera info...");
    auto now = ros::Time::now();
    while (ros::Time::now() - now < ros::Duration(1.0)) {
        pcbq.callAvailable();
    }
    res.camera_info = this->realsense_ir_camera_info;
    ROS_INFO_STREAM("ir camera info captured.");
    return true;
}

// Service callback for ir image raw
bool ImageCapture::get_realsense_ir_image_raw(
    acrv_realsense_ros::get_camera_image::Request& req,
    acrv_realsense_ros::get_camera_image::Response& res)
{
    // Create new node handle and subscriber for one time capture instance
    ros::CallbackQueue pcbq;
    ros::NodeHandle pnh;
    pnh.setCallbackQueue(&pcbq);
    auto sub = pnh.subscribe(std::string("/realsense/ir/image_raw"), 1,
        &ImageCapture::sub_realsense_ir_image_raw, this);
    ROS_INFO_STREAM("Waiting for ir image raw...");
    auto now = ros::Time::now();
    while (ros::Time::now() - now < ros::Duration(1.0)) {
        pcbq.callAvailable();
    }
    res.camera_image = this->realsense_ir_image_raw;
    ROS_INFO_STREAM("ir image raw captured.");
    return true;
}

// Service callback for ir image rect
bool ImageCapture::get_realsense_ir_image_rect(
    acrv_realsense_ros::get_camera_image::Request& req,
    acrv_realsense_ros::get_camera_image::Response& res)
{
    // Create new node handle and subscriber for one time capture instance
    ros::CallbackQueue pcbq;
    ros::NodeHandle pnh;
    pnh.setCallbackQueue(&pcbq);
    auto sub = pnh.subscribe(std::string("/realsense/ir/image_rect"), 1,
        &ImageCapture::sub_realsense_ir_image_rect, this);
    ROS_INFO_STREAM("Waiting for ir image rect...");
    auto now = ros::Time::now();
    while (ros::Time::now() - now < ros::Duration(1.0)) {
        pcbq.callAvailable();
    }
    res.camera_image = this->realsense_ir_image_rect;
    ROS_INFO_STREAM("ir image rect captured.");
    return true;
}

// Service callback for rgb camera info
bool ImageCapture::get_realsense_rgb_camera_info(
    acrv_realsense_ros::get_camera_info::Request& req,
    acrv_realsense_ros::get_camera_info::Response& res)
{
    // Create new node handle and subscriber for one time capture instance
    ros::CallbackQueue pcbq;
    ros::NodeHandle pnh;
    pnh.setCallbackQueue(&pcbq);
    auto sub = pnh.subscribe(std::string("/realsense/rgb/camera_info"), 1,
        &ImageCapture::sub_realsense_rgb_camera_info, this);
    ROS_INFO_STREAM("Waiting for rgb camera info...");
    auto now = ros::Time::now();
    while (ros::Time::now() - now < ros::Duration(1.0)) {
        pcbq.callAvailable();
    }
    res.camera_info = this->realsense_rgb_camera_info;
    ROS_INFO_STREAM("rgb camera info captured.");
    return true;
}

// Service callback for rgb image raw
bool ImageCapture::get_realsense_rgb_image_raw(
    acrv_realsense_ros::get_camera_image::Request& req,
    acrv_realsense_ros::get_camera_image::Response& res)
{
    // Create new node handle and subscriber for one time capture instance
    ros::CallbackQueue pcbq;
    ros::NodeHandle pnh;
    pnh.setCallbackQueue(&pcbq);
    auto sub = pnh.subscribe(std::string("/realsense/rgb/image_raw"), 1,
        &ImageCapture::sub_realsense_rgb_image_raw, this);
    ROS_INFO_STREAM("Waiting for rgb image raw...");
    auto now = ros::Time::now();
    while (ros::Time::now() - now < ros::Duration(1.0)) {
        pcbq.callAvailable();
    }
    res.camera_image = this->realsense_rgb_image_raw;
    ROS_INFO_STREAM("rgb image raw captured.");
    return true;
}

// Service callback for rgb image rect
bool ImageCapture::get_realsense_rgb_image_rect(
    acrv_realsense_ros::get_camera_image::Request& req,
    acrv_realsense_ros::get_camera_image::Response& res)
{
    // Create new node handle and subscriber for one time capture instance
    ros::CallbackQueue pcbq;
    ros::NodeHandle pnh;
    pnh.setCallbackQueue(&pcbq);
    auto sub = pnh.subscribe(std::string("/realsense/rgb/image_rect"), 1,
        &ImageCapture::sub_realsense_rgb_image_rect, this);
    ROS_INFO_STREAM("Waiting for rgb image rect...");
    auto now = ros::Time::now();
    while (ros::Time::now() - now < ros::Duration(1.0)) {
        pcbq.callAvailable();
    }
    res.camera_image = this->realsense_rgb_image_rect;
    ROS_INFO_STREAM("rgb image rect captured.");
    return true;
}

// Service callback for all images
bool ImageCapture::get_realsense_all_images(
    acrv_realsense_ros::get_all_images::Request& req,
    acrv_realsense_ros::get_all_images::Response& res)
{
    // Create new node handle and subscriber for one time capture instance
    ros::CallbackQueue pcbq;
    ros::NodeHandle pnh;
    pnh.setCallbackQueue(&pcbq);

    // Create subscribers for all images and cloud
    auto sub_image_depth_image_raw = pnh.subscribe(std::string("/realsense/depth/image_raw"), 1,
        &ImageCapture::sub_realsense_depth_image_raw, this);
    ROS_INFO_STREAM("depth_image_raw subscribed");
    auto sub_image_depth_image_raw_m = pnh.subscribe(std::string("/realsense/depth/image_raw_m"), 1,
        &ImageCapture::sub_realsense_depth_image_raw_m, this);
    ROS_INFO_STREAM("depth_image_raw_m subscribed");
    auto sub_image_depth_image_rect = pnh.subscribe(std::string("/realsense/depth/image_rect"), 1,
        &ImageCapture::sub_realsense_depth_image_rect, this);
    ROS_INFO_STREAM("depth_image_rect subscribed");
    auto sub_image_depth_registered_image_rect = pnh.subscribe(std::string("/realsense/depth_registered/image_rect"), 1,
        &ImageCapture::sub_realsense_depth_registered_image_rect, this);
    ROS_INFO_STREAM("depth_registered_image_rect subscribed");
    auto sub_cloud_depth_registered_points = pnh.subscribe(std::string("/realsense/depth_registered/points"), 1,
        &ImageCapture::sub_realsense_depth_registered_points, this);
    ROS_INFO_STREAM("depth_registered_points subscribed");
    auto sub_image_ir_image_raw = pnh.subscribe(std::string("/realsense/ir/image_raw"), 1,
        &ImageCapture::sub_realsense_ir_image_raw, this);
    ROS_INFO_STREAM("ir_image_raw subscribed");
    auto sub_image_ir_image_rect = pnh.subscribe(std::string("/realsense/ir/image_rect"), 1,
        &ImageCapture::sub_realsense_ir_image_rect, this);
    ROS_INFO_STREAM("ir_image_rect subscribed");
    auto sub_image_rgb_image_raw = pnh.subscribe(std::string("/realsense/rgb/image_raw"), 1,
        &ImageCapture::sub_realsense_rgb_image_raw, this);
    ROS_INFO_STREAM("rgb_image_raw subscribed");
    auto sub_image_rgb_image_rect = pnh.subscribe(std::string("/realsense/rgb/image_rect"), 1,
        &ImageCapture::sub_realsense_rgb_image_rect, this);
    ROS_INFO_STREAM("rgb_image_rect subscribed");

    auto now = ros::Time::now();
    while (ros::Time::now() - now < ros::Duration(1.0)) {
        pcbq.callAvailable();
    }
    // Save all values into object
    res.image_depth_image_raw = this->realsense_depth_image_raw;
    res.image_depth_image_raw_m = this->realsense_depth_image_raw_m;
    res.image_depth_image_rect = this->realsense_depth_image_rect;
    res.image_depth_registered_image_rect = this->realsense_depth_registered_image_rect;
    res.cloud_depth_registered_points = this->realsense_depth_registered_points;
    res.image_ir_image_raw = this->realsense_ir_image_raw;
    res.image_ir_image_rect = this->realsense_ir_image_rect;
    res.image_rgb_image_raw = this->realsense_rgb_image_raw;
    res.image_rgb_image_rect = this->realsense_rgb_image_rect;
    ROS_INFO_STREAM("All images saved.");
    return true;
}

// Node spin
void ImageCapture::start(void)
{
    ROS_INFO("Image Capture Service started.");
    ros::spin();
}

// Main function
int main(int argc, char** argv)
{
    std::srand(std::time(0));
    // Initialise node
    ros::init(argc, argv, "realsense_capture_service");
    // Instantiate object
    ImageCapture ic;
    // Start ROS node
    ic.start();
    return 0;
}
