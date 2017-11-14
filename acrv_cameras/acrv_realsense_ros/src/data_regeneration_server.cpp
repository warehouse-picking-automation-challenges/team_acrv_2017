#include <ros/ros.h>
#include <ros/console.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>
#include <math.h>
#include <std_srvs/Trigger.h>
#include <thread>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>


/*
NOTE
 * Only full paths work when setting the folder to read images from

TODO
 * improve ros parameter interface to make it more robust
 * add improved ctrl+c handling to stop errors on exit
 * submit issue to pcl github as saving the pointcloud on a slow computer results with an incorrect pcd (colours wrong)
*/


image_transport::CameraPublisher color_hd_pub;
image_transport::CameraPublisher color_pub;
image_transport::CameraPublisher ir_pub;
image_transport::CameraPublisher depth_pub;

sensor_msgs::CameraInfo color_hd_camera_info;
sensor_msgs::CameraInfo color_camera_info;
sensor_msgs::CameraInfo depth_camera_info;
sensor_msgs::CameraInfo ir_camera_info;

std::string read_folder_path;
std::string save_folder_path;


// cv::Mat to sensor_msgs::Image
sensor_msgs::ImagePtr cv_mat_to_sensor_msgs_image(cv::Mat &image, std::string encoding, std::string frame) {
    static int header_sequence_id = 0;

    std_msgs::Header header;

    header.seq = header_sequence_id++;
    header.stamp = ros::Time::now();
    header.frame_id = frame;

    sensor_msgs::ImagePtr a;

    try {
        cv_bridge::CvImage::Ptr cv_msg_ptr = cv_bridge::CvImage::Ptr(new cv_bridge::CvImage(header, encoding, image));
        a = cv_msg_ptr->toImageMsg();
    } catch (...) {
        ROS_ERROR("Yoooo");
    }

    return a;
}

// sensor_msgs::Image to cv::Mat
cv_bridge::CvImagePtr sensor_msgs_image_to_cv_mat(const sensor_msgs::Image& msg, std::string encoding) {
    // ROS_ERROR_STREAM("Yo " << msg.encoding << " received " << encoding);
    // ROS_ERROR_STREAM("Yo " << msg.step);
    cv_bridge::CvImagePtr cv_im;
    try {
        cv_im = cv_bridge::toCvCopy(msg, encoding);
    }
    catch(cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge yo exception: %s", e.what());
    }
    return cv_im;
}

void fill_color_camera_info(sensor_msgs::CameraInfo &cam_info) {
    // COLOR SD
    cam_info.header.frame_id ="realsense_rgb_optical_frame";
    cam_info.width = 640;
    cam_info.height = 360;
    cam_info.distortion_model = "plumb_bob";

    cam_info.D = {0.0,0.0,0.0,0.0,0.0};
    cam_info.K = {1386.69458007812500000, 0.0, 945.50714111328125000, 0.0, 1386.69482421875000000, 545.70349121093750000, 0.0, 0.0, 1.0};
    cam_info.P = {1386.2740478515625, 0.0, 963.1763769978643, 0.0, 0.0, 1389.9100341796875, 533.4262212853137, 0.0, 0.0, 0.0, 1.0, 0.0};
    cam_info.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    cam_info.K[0] = cam_info.K[0]/3.0;  // fx
    cam_info.K[4] = cam_info.K[4]/3.0;  // fy
    cam_info.K[2] = cam_info.K[2]/3.0;  // ppx
    cam_info.K[5] = cam_info.K[5]/3.0;  // ppy
    cam_info.P[0] = cam_info.P[0]/3.0;  // fx
    cam_info.P[5] = cam_info.P[5]/3.0;  // fy
    cam_info.P[2] = cam_info.P[2]/3.0;  // ppx
    cam_info.P[6] = cam_info.P[6]/3.0;  // ppy
}

void fill_color_hd_camera_info(sensor_msgs::CameraInfo &cam_info) {
    // COLOR HD
    cam_info.header.frame_id ="realsense_rgb_optical_frame";
    cam_info.width = 1920;
    cam_info.height = 1080;
    cam_info.distortion_model = "plumb_bob";

    cam_info.D = {0.0,0.0,0.0,0.0,0.0};
    cam_info.K = {1386.69458007812500000, 0.0, 945.50714111328125000, 0.0, 1386.69482421875000000, 545.70349121093750000, 0.0, 0.0, 1.0};
    cam_info.P = {1386.2740478515625, 0.0, 963.1763769978643, 0.0, 0.0, 1389.9100341796875, 533.4262212853137, 0.0, 0.0, 0.0, 1.0, 0.0};
    cam_info.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
}

void fill_ir_camera_info(sensor_msgs::CameraInfo &cam_info) {
    // IR and DEPTH
    cam_info.header.frame_id ="realsense_depth_optical_frame";
    cam_info.width = 640;
    cam_info.height = 480;
    cam_info.distortion_model = "plumb_bob";

    cam_info.D = {-0.12261842462089015, -0.07876736265833705, 0.00060544807387231, -0.0029881970033455068, 0.06835591980110216};
    cam_info.K = {478.76515738695576, 0.0, 326.9909319103222, 0.0, 478.932963387425, 239.17186207918877, 0.0, 0.0, 1.0};
    cam_info.P = {442.51873779296875, 0.0, 325.70964372812887, 0.0, 0.0, 460.9183349609375, 239.35298833222623, 0.0, 0.0, 0.0, 1.0, 0.0};
    cam_info.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
}

void from_16UC1_mm_to_16UC1_scaled(cv::Mat& input_uint16, cv::Mat& output_uint16) {
    // Convert a uint16 depth image from realsense to the uint16 save format
    // Realsense depth images are uint16 in mm
    cv::Mat temp_float64(input_uint16.size().height, input_uint16.size().width, CV_64FC1);
    input_uint16.convertTo(temp_float64, CV_64FC1);
    temp_float64 /= 1000.0;  // reverse the 1000.0 multiplication published by realsense camera node to get back to metres
    temp_float64 *= 65536.0/2.0;  // scale to the range we want so as to fill our uint16 with 0-2 metres
    temp_float64.convertTo(output_uint16, CV_16UC1);
}

void from_16UC1_scaled_to_16UC1_mm(cv::Mat& input_uint16, cv::Mat& output_uint16) {
    // Convert the uint16 save format to a float32 depth in metres format ros likes
    cv::Mat temp_float64(input_uint16.size().height, input_uint16.size().width, CV_64FC1);
    input_uint16.convertTo(temp_float64, CV_64FC1);
    temp_float64 *= 2.0/65536.0;  // reverse the scaling to get depth in metres
    temp_float64 *= 1000.0; // convert from m to mm
    temp_float64.convertTo(output_uint16, CV_16UC1);
}

void from_32FC1_m_to_16UC1_scaled(cv::Mat& input_float32, cv::Mat& output_uint16) {
    // Convert a float32 deoth in metres format to the uint16 save format
    cv::Mat temp_float64(input_float32.size().height, input_float32.size().width, CV_64FC1);
    input_float32.convertTo(temp_float64, CV_64FC1);
    temp_float64 *= 65536.0/2.0;  // scale to the range we want so as to fill our uint16 with 0-2 metres
    temp_float64.convertTo(output_uint16, CV_16UC1);
}

void from_8UC1_to_16UC1(cv::Mat& input_uint8, cv::Mat& output_uint16) {
    // Convert a uint8 format to uint16 format
    cv::Mat temp_float64(input_uint8.size().height, input_uint8.size().width, CV_64FC1);
    input_uint8.convertTo(temp_float64, CV_64FC1);
    temp_float64 /= 256.0;  // scale down to 0-1
    temp_float64 *= 65536.0;  // scale up to fill uint16
    temp_float64.convertTo(output_uint16, CV_16UC1);
}

bool publish_raw_data_once_service(std_srvs::Trigger::Request &req,
                                   std_srvs::Trigger::Response &res) {
    ROS_INFO_STREAM("Service has been called...");
    ROS_INFO_STREAM("/data_regeneration_server_node/read_folder_path = " << read_folder_path);
    ROS_INFO_STREAM("/data_regeneration_server_node/save_folder_path = " << save_folder_path);

    // Create a folder for saving the new images
    // Ensure the stringstream is properly cleared - see: http://stackoverflow.com/questions/2848087/how-to-clear-stringstream
    // save_folder_path.str(std::string());
    // save_folder_path.clear();
    // save_folder_path << "../data/img_" <<  std::setfill('0') << std::setw(5) << save_image_number;
    // const int dir_err = system(("mkdir -p " + save_folder_path).c_str());
    const int dir_err = system(("mkdir -p " + save_folder_path).c_str());
    if (-1 == dir_err) {
        printf("Error creating directory!\n");
        exit(1);
    }

    // Load raw-hd-color, raw-ir and raw-scaled-depth images
    cv::Mat color_hd_image_cv, color_hd_image_cv_bgr;
    color_hd_image_cv_bgr = cv::imread(read_folder_path + "color.png", CV_LOAD_IMAGE_COLOR);
    if (color_hd_image_cv_bgr.empty() == true) {
        res.success = false;
        return true;
    }
    cvtColor(color_hd_image_cv_bgr, color_hd_image_cv, CV_BGR2RGB);
    cv::Mat ir_image_cv_raw;
    ir_image_cv_raw = cv::imread(read_folder_path + "ir.png", CV_LOAD_IMAGE_ANYDEPTH);
    if (ir_image_cv_raw.empty() == true) {
        res.success = false;
        return true;
    }
    cv::Mat ir_image_cv(ir_image_cv_raw.size().height, ir_image_cv_raw.size().width, CV_16UC1);
    if (ir_image_cv_raw.type() == 0) {  // type 0 == 8UC1 as per -> https://stackoverflow.com/questions/10167534/how-to-find-out-what-type-of-a-mat-object-is-with-mattype-in-opencv
        from_8UC1_to_16UC1(ir_image_cv_raw, ir_image_cv);
    } else {
        ir_image_cv = ir_image_cv_raw;
    }
    cv::Mat depth_image_cv_16UC1_scaled;
    depth_image_cv_16UC1_scaled = cv::imread(read_folder_path + "depth.png", CV_LOAD_IMAGE_ANYDEPTH);
    if (depth_image_cv_16UC1_scaled.empty() == true) {
        res.success = false;
        return true;
    }

    // It seems that some depth images are scaled and others aren't. Need a way to differentiate and scale accordingly.
    // A non-scaled image example had max depth of 786
    // A scaled image example had max depth of 25854
    // Choose 10000 as threshold
    double min, max;
    cv::minMaxIdx(depth_image_cv_16UC1_scaled, &min, &max);
    // printf("min: %f, max: %f\n", min, max);

    cv::Mat depth_image_cv_16UC1_mm(depth_image_cv_16UC1_scaled.size().height, depth_image_cv_16UC1_scaled.size().width, CV_16UC1);
    if (max < 10000.0) {
        // Don't convert unscaled depth images
        depth_image_cv_16UC1_mm = depth_image_cv_16UC1_scaled;
    } else {
        // Convert scaled 16UC1 depth image to 16UC1 depth image in mm as published by realsense
        from_16UC1_scaled_to_16UC1_mm(depth_image_cv_16UC1_scaled, depth_image_cv_16UC1_mm);
    }

    // Create SD color image from HD color image
    // cv::Mat crop_img = color_hd_image_cv(cv::Rect(240,0,1440,1080));
    cv::Mat color_image_cv;
    cv::resize(color_hd_image_cv, color_image_cv, cv::Size(640,360), cv::INTER_CUBIC);

    // Convert CV images to sensor_msgs/Image
    try {
        sensor_msgs::ImagePtr color_hd_image_msg = cv_mat_to_sensor_msgs_image(color_hd_image_cv, sensor_msgs::image_encodings::RGB8, "realsense_rgb_optical_frame");
        sensor_msgs::ImagePtr color_image_msg = cv_mat_to_sensor_msgs_image(color_image_cv, sensor_msgs::image_encodings::RGB8, "realsense_rgb_optical_frame");
        sensor_msgs::ImagePtr ir_image_msg = cv_mat_to_sensor_msgs_image(ir_image_cv, sensor_msgs::image_encodings::TYPE_16UC1, "realsense_depth_optical_frame");
        sensor_msgs::ImagePtr depth_image_msg = cv_mat_to_sensor_msgs_image(depth_image_cv_16UC1_mm, sensor_msgs::image_encodings::TYPE_16UC1, "realsense_depth_optical_frame");

        // Update timestamps
        ros::Time time_now = ros::Time::now();
        color_hd_image_msg->header.stamp = time_now;
        color_image_msg->header.stamp = time_now;
        ir_image_msg->header.stamp = time_now;
        depth_image_msg->header.stamp = time_now;

        // Publish raw images (and camera info's) - camera info filled in main()
        // These images are equivalent to what the realsense publishes
        color_hd_pub.publish(*color_hd_image_msg, color_hd_camera_info, time_now);
        color_pub.publish(*color_image_msg, color_camera_info, time_now);
        ir_pub.publish(*ir_image_msg, ir_camera_info, time_now);
        depth_pub.publish(*depth_image_msg, depth_camera_info, time_now);
    } catch(...) {
        ROS_ERROR_STREAM("ir_image_raw failed");
    }

    // TODO add wait until all subscribers have triggered and so saved their data

    res.success = true;
    return true;
}

void sub_realsense_depth_image_raw_callback(const sensor_msgs::Image& msg) {
    ROS_INFO_STREAM("sub_realsense_depth_image_raw_callback has been triggered");
try {
    cv_bridge::CvImagePtr im = sensor_msgs_image_to_cv_mat(msg, sensor_msgs::image_encodings::TYPE_16UC1);

    cv::Mat im_scaled(im->image.size().height, im->image.size().width, CV_16UC1);
    from_16UC1_mm_to_16UC1_scaled(im->image, im_scaled);

    std::stringstream img_path_fn;
    img_path_fn << save_folder_path << "depth_raw.png";
    ROS_INFO_STREAM("Saving image: " << img_path_fn.str());
    ROS_INFO_STREAM("Success = " << cv::imwrite(img_path_fn.str(), im_scaled));
} catch(...) {
    ROS_ERROR_STREAM("ir_image_raw failed");
}
}

void sub_realsense_depth_image_rect_callback(const sensor_msgs::Image& msg) {
    ROS_INFO_STREAM("sub_realsense_depth_image_rect_callback has been triggered");
try {
    cv_bridge::CvImagePtr im = sensor_msgs_image_to_cv_mat(msg, sensor_msgs::image_encodings::TYPE_32FC1);

    cv::Mat im_scaled(im->image.size().height, im->image.size().width, CV_16UC1);
    from_32FC1_m_to_16UC1_scaled(im->image, im_scaled);

    std::stringstream img_path_fn;
    img_path_fn << save_folder_path << "depth_rect.png";
    ROS_INFO_STREAM("Saving image: " << img_path_fn.str());
    ROS_INFO_STREAM("Success = " << cv::imwrite(img_path_fn.str(), im_scaled));
} catch(...) {
    ROS_ERROR_STREAM("ir_image_raw failed");
}
}

void sub_realsense_ir_image_raw_callback(const sensor_msgs::Image& msg) {
    ROS_INFO_STREAM("sub_realsense_ir_image_raw_callback has been triggered");

    try {
        // ROS_ERROR_STREAM("Yo " << msg.encoding);
        cv_bridge::CvImagePtr im = sensor_msgs_image_to_cv_mat(msg, sensor_msgs::image_encodings::TYPE_16UC1);

        std::stringstream img_path_fn;
        img_path_fn << save_folder_path << "ir_raw.png";
        ROS_INFO_STREAM("Saving image: " << img_path_fn.str());
        ROS_INFO_STREAM("Success = " << cv::imwrite(img_path_fn.str(), im->image));
    } catch(...) {
        ROS_ERROR_STREAM("ir_image_raw failed");
    }
}

void sub_realsense_ir_image_rect_callback(const sensor_msgs::Image& msg) {
    ROS_INFO_STREAM("sub_realsense_ir_image_rect_callback has been triggered");
    try {
        cv_bridge::CvImagePtr im = sensor_msgs_image_to_cv_mat(msg, sensor_msgs::image_encodings::TYPE_16UC1);

        std::stringstream img_path_fn;
        img_path_fn << save_folder_path << "ir_rect.png";
        ROS_INFO_STREAM("Saving image: " << img_path_fn.str());
        ROS_INFO_STREAM("Success = " << cv::imwrite(img_path_fn.str(), im->image));
    } catch(...) {
        ROS_ERROR_STREAM("ir_image_rect failed");
    }
}

void sub_realsense_rgb_image_raw_callback(const sensor_msgs::Image& msg) {
    ROS_INFO_STREAM("sub_realsense_rgb_image_raw_callback has been triggered");
try {
    cv_bridge::CvImagePtr im = sensor_msgs_image_to_cv_mat(msg, sensor_msgs::image_encodings::BGR8);

    std::stringstream img_path_fn;
    img_path_fn << save_folder_path << "color_raw.png";
    ROS_INFO_STREAM("Saving image: " << img_path_fn.str());
    ROS_INFO_STREAM("Success = " << cv::imwrite(img_path_fn.str(), im->image));
} catch(...) {
    ROS_ERROR_STREAM("ir_image_raw failed");
}
}

void sub_realsense_rgb_image_rect_callback(const sensor_msgs::Image& msg) {
    ROS_INFO_STREAM("sub_realsense_rgb_image_rect_callback has been triggered");
try {
    cv_bridge::CvImagePtr im = sensor_msgs_image_to_cv_mat(msg, sensor_msgs::image_encodings::BGR8);

    std::stringstream img_path_fn;
    img_path_fn << save_folder_path << "color_rect.png";
    ROS_INFO_STREAM("Saving image: " << img_path_fn.str());
    ROS_INFO_STREAM("Success = " << cv::imwrite(img_path_fn.str(), im->image));
} catch(...) {
    ROS_ERROR_STREAM("ir_image_raw failed");
}
}

void sub_realsense_rgb_hd_image_raw_callback(const sensor_msgs::Image& msg) {
    ROS_INFO_STREAM("sub_realsense_rgb_hd_image_raw_callback has been triggered");
try {
    cv_bridge::CvImagePtr im = sensor_msgs_image_to_cv_mat(msg, sensor_msgs::image_encodings::BGR8);

    std::stringstream img_path_fn;
    img_path_fn << save_folder_path << "color_hd_raw.png";
    ROS_INFO_STREAM("Saving image: " << img_path_fn.str());
    ROS_INFO_STREAM("Success = " << cv::imwrite(img_path_fn.str(), im->image));
} catch(...) {
    ROS_ERROR_STREAM("ir_image_raw failed");
}
}

void sub_realsense_rgb_hd_image_rect_callback(const sensor_msgs::Image& msg) {
    ROS_INFO_STREAM("sub_realsense_rgb_hd_image_rect_callback has been triggered");
try {
    cv_bridge::CvImagePtr im = sensor_msgs_image_to_cv_mat(msg, sensor_msgs::image_encodings::BGR8);

    std::stringstream img_path_fn;
    img_path_fn << save_folder_path << "color_hd_rect.png";
    ROS_INFO_STREAM("Saving image: " << img_path_fn.str());
    ROS_INFO_STREAM("Success = " << cv::imwrite(img_path_fn.str(), im->image));
} catch(...) {
    ROS_ERROR_STREAM("ir_image_raw failed");
}
}

void sub_realsense_depth_registered_image_rect_callback(const sensor_msgs::Image& msg) {
    ROS_INFO_STREAM("sub_realsense_depth_registered_image_rect_callback has been triggered");
try {
    cv_bridge::CvImagePtr im = sensor_msgs_image_to_cv_mat(msg, sensor_msgs::image_encodings::TYPE_32FC1);

    cv::Mat im_scaled(im->image.size().height, im->image.size().width, CV_16UC1);
    from_32FC1_m_to_16UC1_scaled(im->image, im_scaled);

    std::stringstream img_path_fn;
    img_path_fn << save_folder_path << "depth_aligned.png";
    ROS_INFO_STREAM("Saving image: " << img_path_fn.str());
    ROS_INFO_STREAM("Success = " << cv::imwrite(img_path_fn.str(), im_scaled));
} catch(...) {
    ROS_ERROR_STREAM("ir_image_raw failed");
}
}

void sub_realsense_depth_hd_registered_image_rect_callback(const sensor_msgs::Image& msg) {
    ROS_INFO_STREAM("sub_realsense_depth_hd_registered_image_rect_callback has been triggered");
try {
    cv_bridge::CvImagePtr im = sensor_msgs_image_to_cv_mat(msg, sensor_msgs::image_encodings::TYPE_32FC1);

    cv::Mat im_scaled(im->image.size().height, im->image.size().width, CV_16UC1);
    from_32FC1_m_to_16UC1_scaled(im->image, im_scaled);

    std::stringstream img_path_fn;
    img_path_fn << save_folder_path << "depth_hd_aligned.png";
    ROS_INFO_STREAM("Saving image: " << img_path_fn.str());
    ROS_INFO_STREAM("Success = " << cv::imwrite(img_path_fn.str(), im_scaled));
} catch(...) {
    ROS_ERROR_STREAM("ir_image_raw failed");
}
}

void sub_realsense_depth_registered_points_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_ptr) {
    ROS_INFO_STREAM("sub_realsense_depth_registered_points_callback has been triggered");
try {
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::fromROSMsg(*cloud_ptr, cloud);

    std::stringstream img_path_fn;
    img_path_fn << save_folder_path << "cloud.pcd";
    ROS_INFO_STREAM("Saving cloud " << img_path_fn.str());
    pcl::io::savePCDFile(img_path_fn.str(), cloud, true);  // binary format
    ROS_INFO_STREAM("Check the pcd to see if the save was successful.");
} catch(...) {
    ROS_ERROR_STREAM("ir_image_raw failed");
}
}

void sub_realsense_depth_hd_registered_points_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_ptr) {
    ROS_INFO_STREAM("sub_realsense_depth_hd_registered_points_callback has been triggered");
try {
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::fromROSMsg(*cloud_ptr, cloud);

    std::stringstream img_path_fn;
    img_path_fn << save_folder_path << "cloud_hd.pcd";
    ROS_INFO_STREAM("Saving cloud " << img_path_fn.str());
    pcl::io::savePCDFile(img_path_fn.str(), cloud, true);  // binary format
    ROS_INFO_STREAM("Check the pcd to see if the save was successful.");
} catch(...) {
    ROS_ERROR_STREAM("ir_image_raw failed");
}
}

int main(int argc, char * argv[]) try {
    ros::init(argc, argv, "data_regeneration_server_node");

    ros::NodeHandle nh("data_regeneration_server_node");
    image_transport::ImageTransport image_transport(nh);

    // image_transport will create /realsense/*/camera_info topics for these streams
    color_hd_pub = image_transport.advertiseCamera("/realsense/rgb_hd/image_raw", 1);
    color_pub = image_transport.advertiseCamera("/realsense/rgb/image_raw", 1);
    depth_pub = image_transport.advertiseCamera("/realsense/depth/image_raw", 1);
    ir_pub = image_transport.advertiseCamera("/realsense/ir/image_raw", 1);

    // By passing in a name for the nodehandle above, the service below will
    // fall under that namespace. So this service can be found at:
    // /data_regeneration_server_node/publish_raw_data_once
    ros::ServiceServer service = nh.advertiseService("publish_raw_data_once", publish_raw_data_once_service);

    ros::Subscriber sub_realsense_depth_image_raw = nh.subscribe("/realsense/depth/image_raw", 20, sub_realsense_depth_image_raw_callback);
    ros::Subscriber sub_realsense_depth_image_rect = nh.subscribe("/realsense/depth/image_rect", 20, sub_realsense_depth_image_rect_callback);
    ros::Subscriber sub_realsense_ir_image_raw = nh.subscribe("/realsense/ir/image_raw", 20, sub_realsense_ir_image_raw_callback);
    ros::Subscriber sub_realsense_ir_image_rect = nh.subscribe("/realsense/ir/image_rect", 20, sub_realsense_ir_image_rect_callback);
    ros::Subscriber sub_realsense_rgb_image_raw = nh.subscribe("/realsense/rgb/image_raw", 20, sub_realsense_rgb_image_raw_callback);
    ros::Subscriber sub_realsense_rgb_image_rect = nh.subscribe("/realsense/rgb/image_rect", 20, sub_realsense_rgb_image_rect_callback);
    ros::Subscriber sub_realsense_rgb_hd_image_raw = nh.subscribe("/realsense/rgb_hd/image_raw", 20, sub_realsense_rgb_hd_image_raw_callback);
    ros::Subscriber sub_realsense_rgb_hd_image_rect = nh.subscribe("/realsense/rgb_hd/image_rect", 20, sub_realsense_rgb_hd_image_rect_callback);
    ros::Subscriber sub_realsense_depth_registered_image_rect = nh.subscribe("/realsense/depth_registered/image_rect", 20, sub_realsense_depth_registered_image_rect_callback);
    ros::Subscriber sub_realsense_depth_hd_registered_image_rect = nh.subscribe("/realsense/depth_hd_registered/image_rect", 20, sub_realsense_depth_hd_registered_image_rect_callback);
    ros::Subscriber sub_realsense_depth_registered_points = nh.subscribe("/realsense/depth_registered/points", 20, sub_realsense_depth_registered_points_callback);
    ros::Subscriber sub_realsense_depth_hd_registered_points = nh.subscribe("/realsense/depth_hd_registered/points", 20, sub_realsense_depth_hd_registered_points_callback);

    fill_color_hd_camera_info(color_hd_camera_info);
    fill_color_camera_info(color_camera_info);
    fill_ir_camera_info(depth_camera_info);
    fill_ir_camera_info(ir_camera_info);

    ros::Rate r(40);

    while(ros::ok()) {
        // These are just here because I didn't feel like making the nodehandle global.
        // Probably better in the service callback
        // I'm using parameter server so that I don't need to create a custom service message
        nh.getParam("read_folder_path", read_folder_path);
        nh.getParam("save_folder_path", save_folder_path);

        ros::spinOnce();
        r.sleep();
    }
}
// some other error
catch(const std::exception & e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
