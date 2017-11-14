#include <ros/ros.h>
#include <ros/console.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <librealsense/rs.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <opencv2/core/core.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>
#include <math.h>
#include <std_srvs/SetBool.h>
#include <thread>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;

ros::Publisher points_pub;
ros::Publisher points_aligned_pub;
image_transport::CameraPublisher color_pub;
image_transport::CameraPublisher color_reg_depth_pub;
image_transport::CameraPublisher depth_reg_color_pub;
image_transport::CameraPublisher ir_pub;
image_transport::CameraPublisher depth_pub;
image_transport::CameraPublisher depth_aligned_pub;
sensor_msgs::CameraInfo color_camera_info;
sensor_msgs::CameraInfo depth_camera_info;
sensor_msgs::CameraInfo ir_camera_info;
sensor_msgs::CameraInfo color_reg_camera_info;
camera_info_manager::CameraInfoManager *color_camera_info_man;
camera_info_manager::CameraInfoManager *color_reg_camera_info_man;
camera_info_manager::CameraInfoManager *ir_camera_info_man;
camera_info_manager::CameraInfoManager *depth_camera_info_man;
rs::device * dev;

// cv Mat to sensor_msgs::Image conversion
sensor_msgs::ImagePtr cvImagetoMsg(cv::Mat &image,std::string encoding, std::string frame){

    static int header_sequence_id = 0;

    //cv_bridge::CvImage::Ptr cv_msg_ptr = new cv_bridge::CvImage::Ptr();
    std_msgs::Header header;

    header.seq = header_sequence_id++;
    header.stamp = ros::Time::now();
    header.frame_id = frame;

    cv_bridge::CvImage::Ptr cv_msg_ptr = cv_bridge::CvImage::Ptr(new cv_bridge::CvImage(header,encoding,image));

    return cv_msg_ptr->toImageMsg();
}

void intrinsToCameraInfo(rs::intrinsics &intrins, sensor_msgs::CameraInfo &cam_info) {

    cam_info.height = intrins.height;
    cam_info.width = intrins.width;
    cam_info.distortion_model = "Plumb Bob";
    cam_info.D = {intrins.coeffs[0], intrins.coeffs[1], intrins.coeffs[2], intrins.coeffs[3], intrins.coeffs[4]};
    cam_info.K = {intrins.fx, 0, intrins.ppx, 0, intrins.fy, intrins.ppy, 0, 0, 1};
    cam_info.P = {intrins.fx, 0, intrins.ppx, 0, 0, intrins.fy, intrins.ppy, 0, 0, 0, 1, 0};
    cam_info.R = {1, 0, 0, 0, 1, 0, 0, 0, 1};
}

bool enable_disable_callback(std_srvs::SetBool::Request &req,
                               std_srvs::SetBool::Response &res) {
    ROS_INFO_STREAM("Enable /disable callback, device streaming?" << dev->is_streaming() << req.data);
    if (req.data && !dev->is_streaming()){
      ROS_INFO_STREAM("Enable callback");
      dev->enable_stream(rs::stream::depth, rs::preset::best_quality);
      dev->enable_stream(rs::stream::color, rs::preset::best_quality);
      dev->enable_stream(rs::stream::color, rs::preset::best_quality);
      dev->enable_stream(rs::stream::infrared, rs::preset::best_quality);
      dev->start();
      ROS_INFO_STREAM("Enable callback finished");
    } else if (!req.data && dev->is_streaming()) {
      ROS_INFO_STREAM("Disable callback");
      dev->stop();
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      dev->disable_stream(rs::stream::depth);
      dev->disable_stream(rs::stream::color);
      dev->disable_stream(rs::stream::color);
      dev->disable_stream(rs::stream::infrared);
      ROS_INFO_STREAM("Disable callback finished");
    } else {
      res.success = true;
      ROS_INFO_STREAM(res.success);
      return true;
    }
    res.success = true;
    ROS_INFO_STREAM(res.success);
    return true;
}

int main(int argc, char * argv[]) try
{
    // initialise the node
    ros::init(argc, argv, "acrv_r200_ros");  // Note this node name can be overwritten by a launch file if one is being used
    ros::NodeHandle n("~");
    image_transport::ImageTransport image_transport(n);
    rs::log_to_console(rs::log_severity::warn);

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

    // enable the various camera streams
    dev->enable_stream(rs::stream::depth, rs::preset::best_quality);
    dev->enable_stream(rs::stream::color, rs::preset::best_quality);
    dev->enable_stream(rs::stream::color, rs::preset::best_quality);
    // dev->enable_stream(rs::stream::color, 1920, 1080, rs::format::rgb8, 0);
    dev->enable_stream(rs::stream::infrared, rs::preset::best_quality);
    dev->start();

    if(dev->supports_option(rs::option::color_enable_auto_white_balance)){
    int value = 0;
    dev->set_option(rs::option::color_enable_auto_white_balance,value);

    ROS_INFO_STREAM("Colour exposure supported, current value:" << dev->get_option(rs::option::color_enable_auto_white_balance));
    }

    std::string camera_name;
    if (n.getParam("camera_name", camera_name)) {
        ROS_INFO_STREAM("\"camera_name\" = " << camera_name);
    } else {
        ROS_ERROR_STREAM("\"camera_name\" not provided.");
        return 0;
    }

    // start publishers
    color_pub = image_transport.advertiseCamera("/realsense/rgb/image_raw", 1 );
    color_reg_depth_pub = image_transport.advertiseCamera("/realsense/rgb_depth_aligned/image_raw", 1 );
    depth_reg_color_pub = image_transport.advertiseCamera("/realsense/depth_rgb_aligned/image_raw", 1 );
    depth_pub = image_transport.advertiseCamera("/realsense/depth/image_raw", 1 );
    depth_aligned_pub = image_transport.advertiseCamera("/realsense/depth_aligned/image_raw", 1 );
    ir_pub = image_transport.advertiseCamera("/realsense/ir/image_raw", 1 );
    points_pub = n.advertise<sensor_msgs::PointCloud2>("/realsense/points", 1 );
    points_aligned_pub = n.advertise<sensor_msgs::PointCloud2>("/realsense/points_aligned", 1 );

    ros::ServiceServer service = n.advertiseService("/ros_realsense/enable_disable", enable_disable_callback);

    ros::NodeHandle rgb_handle("~/rgb/");
    ros::NodeHandle rgb_depth_handle("~/rgb_depth_aligned/");
    ros::NodeHandle ir_handle("~/ir/");
    ros::NodeHandle depth_handle("~/depth/");

    color_camera_info_man = new camera_info_manager::CameraInfoManager(rgb_handle,"/" + camera_name + "_rgb_optical_frame");
    color_reg_camera_info_man = new camera_info_manager::CameraInfoManager(rgb_depth_handle,"/" + camera_name + "_depth_optical_frame");
    ir_camera_info_man = new camera_info_manager::CameraInfoManager(ir_handle,"/" + camera_name + "_depth_optical_frame");
    depth_camera_info_man = new camera_info_manager::CameraInfoManager(depth_handle,"/" + camera_name + "_depth_optical_frame");

    //obtain camera intrinsics and extrinsics
    rs::intrinsics depth_intrin = dev->get_stream_intrinsics(rs::stream::depth);
    rs::intrinsics color_intrin = dev->get_stream_intrinsics(rs::stream::color);
    rs::intrinsics ir_intrin = dev->get_stream_intrinsics(rs::stream::infrared);
    rs::intrinsics color_aligned_intrin = dev->get_stream_intrinsics(rs::stream::color_aligned_to_depth);
    rs::extrinsics depth_to_color = dev->get_extrinsics(rs::stream::depth, rs::stream::color);
    rs::extrinsics color_to_depth = dev->get_extrinsics(rs::stream::color, rs::stream::depth);
    float scale = dev->get_depth_scale();

    intrinsToCameraInfo(color_intrin,color_camera_info);
    intrinsToCameraInfo(depth_intrin,depth_camera_info);
    intrinsToCameraInfo(ir_intrin,ir_camera_info);
    intrinsToCameraInfo(color_aligned_intrin,color_reg_camera_info);

    color_camera_info_man->setCameraInfo(color_camera_info);
    color_reg_camera_info_man->setCameraInfo(color_reg_camera_info);
    ir_camera_info_man->setCameraInfo(ir_camera_info);
    depth_camera_info_man->setCameraInfo(depth_camera_info);
    ros::Rate r(40);
    int emitter_enabled = 1;
    int prev_emitter_enabled = emitter_enabled;

    while(ros::ok())
    {
        // wait for frames from device
        if(dev->is_streaming()) {
            if (n.getParam("emitter_enabled", emitter_enabled)) {
                if (emitter_enabled != prev_emitter_enabled) {
                    dev->set_option(rs::option::r200_emitter_enabled, emitter_enabled);
                    prev_emitter_enabled = emitter_enabled;
                    for (int i = 0; i < 30; i++) {
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
        uint8_t * color_raw;
        const uint16_t *depth_raw;

        cv::Mat color_reg_image(depth_intrin.height,depth_intrin.width,CV_8UC3);
        PointCloud::Ptr color_reg_depth_cloud(new PointCloud);
        color_reg_depth_cloud->points.resize(depth_intrin.width * depth_intrin.height);
        color_reg_depth_cloud->header.frame_id = "world";
        color_reg_depth_cloud->is_dense = false;
        color_reg_depth_cloud->height = depth_intrin.height;
        color_reg_depth_cloud->width = depth_intrin.width;

        cv::Mat depth_reg_color_image(color_intrin.height,color_intrin.width,CV_8UC3);
        PointCloud::Ptr depth_reg_color_cloud(new PointCloud);
        depth_reg_color_cloud->points.resize(color_intrin.width * color_intrin.height);
        depth_reg_color_cloud->header.frame_id = "world";
        depth_reg_color_cloud->is_dense = false;
        depth_reg_color_cloud->height = color_intrin.height;
        depth_reg_color_cloud->width = color_intrin.width;

        // get color stream if pointcloud or color image subscribed to
        if (points_pub.getNumSubscribers() > 0 || color_pub.getNumSubscribers() > 0 || color_reg_depth_pub.getNumSubscribers() > 0) {
            color_raw = (uint8_t *)dev->get_frame_data(rs::stream::color);
        }

        // only if the pointcloud is subscibed to
        if (points_pub.getNumSubscribers() > 0 || color_reg_depth_pub.getNumSubscribers() > 0)
        {
            depth_raw = (const uint16_t *)dev->get_frame_data(rs::stream::depth);

            // instantiate pcl xyzrgb pointcloud

            for (int dy=0; dy<depth_intrin.height; ++dy){

                for (int dx=0; dx<depth_intrin.width; ++dx){

                    //obtain the depth value and apply scale factor
                    uint16_t depth_value = depth_raw[dy * depth_intrin.width + dx];
                    float depth_in_meters = depth_value * scale;
                    // depth_in_meters = 0.326335990598878*pow(depth_in_meters,3.0) +
                    //                   0.228008715981488*pow(depth_in_meters,2.0) +
                    //                   1.026086470601403*depth_in_meters -
                    //                   0.004441013854035;

                    // Skip over pixels with a depth value of zero, which is used to indicate no data
                    if(depth_value == 0) {
                        color_reg_image.at<cv::Vec3b>(cv::Point(dx,dy)) = cv::Vec3b(0,0,0);
                        continue;
                    }

                    // Map from pixel coordinates in the depth image to pixel coordinates in the color image
                    rs::float2 depth_pixel = {(float)dx, (float)dy};
                    rs::float3 depth_point = depth_intrin.deproject(depth_pixel, depth_in_meters);
                    rs::float3 color_point = depth_to_color.transform(depth_point);
                    rs::float2 color_pixel = color_intrin.project(color_point);

                    // Use the color from the nearest color pixel, ignore this point falls outside the color image
                    const int cx = (int)std::round(color_pixel.x), cy = (int)std::round(color_pixel.y);
                    if (!(cx < 0 || cy < 0 || cx >= color_intrin.width || cy >= color_intrin.height))
                    {

                        //obtain pointer to current colour pixel
                        const uint8_t * color_ptr = color_raw + (cy * color_intrin.width + cx) * 3;

                        // create xyzrgb point
                        pcl::PointXYZRGB point(*(color_ptr), *(color_ptr+1), *(color_ptr+2));
                        point.x = depth_point.x;
                        point.y = depth_point.y;
                        point.z = depth_point.z;

                        //add point to color_reg_depth_cloud
                        color_reg_depth_cloud->points[dy * depth_intrin.width + dx] = point;
                        color_reg_image.at<cv::Vec3b>(cv::Point(dx,dy)) = cv::Vec3b(point.getRGBVector3i()[0],point.getRGBVector3i()[1],point.getRGBVector3i()[2]);
                    } else {
                        color_reg_image.at<cv::Vec3b>(cv::Point(dx,dy)) = cv::Vec3b(0,0,0);
                    }
                }
            }
        }

        // only if the pointcloud aligned or depth reg color is subscibed to
        if (points_aligned_pub.getNumSubscribers() > 0 || depth_reg_color_pub.getNumSubscribers() > 0)
        {
            depth_raw = (const uint16_t *)dev->get_frame_data(rs::stream::depth);

            // instantiate pcl xyzrgb pointcloud

            // Pre-fill image with 0's to account for pixels that fall outside of depth image sweep
            for (int h=0;h<color_intrin.height;h++) {
                for (int w=0;w<color_intrin.width;w++) {
                    depth_reg_color_image.at<cv::Vec3b>(cv::Point(w,h)) = cv::Vec3b(0,0,0);
                }
            }

            for (int dy=0; dy<depth_intrin.height; ++dy){

                for (int dx=0; dx<depth_intrin.width; ++dx){

                    // Obtain the depth value and apply scale factor
                    uint16_t depth_value = depth_raw[dy * depth_intrin.width + dx];
                    float depth_in_meters = depth_value * scale;
                    // depth_in_meters = 0.326335990598878*pow(depth_in_meters,3.0) +
                    //                   0.228008715981488*pow(depth_in_meters,2.0) +
                    //                   1.026086470601403*depth_in_meters -
                    //                   0.004441013854035;

                    // Map from pixel coordinates in the depth image to pixel coordinates in the color image
                    rs::float2 depth_pixel = {(float)dx, (float)dy};
                    rs::float3 depth_point = depth_intrin.deproject(depth_pixel, depth_in_meters);
                    rs::float3 color_point = depth_to_color.transform(depth_point);
                    rs::float2 color_pixel = color_intrin.project(color_point);

                    // Use the color from the nearest color pixel, ignore this point falls outside the color image
                    const int cx = (int)std::round(color_pixel.x), cy = (int)std::round(color_pixel.y);
                    if (!(cx < 0 || cy < 0 || cx >= color_intrin.width || cy >= color_intrin.height))
                    {
                        // Obtain pointer to current colour pixel
                        const uint8_t * color_ptr = color_raw + (cy * color_intrin.width + cx) * 3;

                        // Create xyzrgb point
                        pcl::PointXYZRGB point(*(color_ptr), *(color_ptr+1), *(color_ptr+2));
                        point.x = color_point.x;
                        point.y = color_point.y;
                        point.z = color_point.z;

                        // Add point to depth_reg_color_cloud
                        depth_reg_color_cloud->points[dy * depth_intrin.width + dx] = point;

                        if (depth_value == 0) {
                            depth_reg_color_image.at<cv::Vec3b>(cv::Point(cx,cy)) = cv::Vec3b(0,0,0);
                        } else {
                            depth_reg_color_image.at<cv::Vec3b>(cv::Point(cx,cy)) = cv::Vec3b(point.getRGBVector3i()[0],point.getRGBVector3i()[1],point.getRGBVector3i()[2]);
                        }
                    }
                }
            }
        }

        // only if the pointcloud is subscibed to
        if (points_pub.getNumSubscribers() > 0)
        {
            // test if color_reg_depth_cloud empty
            if ((*color_reg_depth_cloud).points.size() != 0) {
                //convert pcl pointcloud to sensor_msgs pointcloud2, publish
                pcl::PCLPointCloud2 pcl_xyz_pc2;
                pcl::toPCLPointCloud2 (*color_reg_depth_cloud, pcl_xyz_pc2);
                sensor_msgs::PointCloud2 realsense_xyz_cloud2;
                pcl_conversions::moveFromPCL(pcl_xyz_pc2, realsense_xyz_cloud2);
                realsense_xyz_cloud2.header.stamp = ros::Time::now();
                realsense_xyz_cloud2.header.frame_id = "/" + camera_name + "_depth_optical_frame";
                points_pub.publish(realsense_xyz_cloud2);
            }
        }

        // only if the pointcloud aligned is subscibed to
        if (points_aligned_pub.getNumSubscribers() > 0)
        {
            // test if color_reg_depth_cloud empty
            if ((*depth_reg_color_cloud).points.size() != 0) {
                //convert pcl pointcloud to sensor_msgs pointcloud2, publish
                pcl::PCLPointCloud2 pcl_xyz_pc2;
                pcl::toPCLPointCloud2 (*depth_reg_color_cloud, pcl_xyz_pc2);
                sensor_msgs::PointCloud2 realsense_xyz_cloud2;
                pcl_conversions::moveFromPCL(pcl_xyz_pc2, realsense_xyz_cloud2);
                realsense_xyz_cloud2.header.stamp = ros::Time::now();
                realsense_xyz_cloud2.header.frame_id = "/" + camera_name + "_rgb_optical_frame";
                points_aligned_pub.publish(realsense_xyz_cloud2);
            }
        }

        // only if the depth image topic is subscribed to
        if (depth_pub.getNumSubscribers() > 0)
        {
            uint16_t *depth_raw2 = (uint16_t *)dev->get_frame_data(rs::stream::depth);
            // convert to cv image and publish
            cv::Mat depth_image_raw(depth_intrin.height,depth_intrin.width,CV_16UC1,depth_raw2, cv::Mat::AUTO_STEP);

            cv::Mat depth_image(depth_intrin.height,depth_intrin.width,CV_16UC1);

            cv::Mat depth_image_scaled(depth_intrin.height,depth_intrin.width,CV_64FC1);

            depth_image_raw.copyTo(depth_image);
            depth_image_raw.convertTo(depth_image_scaled,CV_64FC1);

            depth_image_scaled *= 1000.0f*scale;

            depth_image_scaled.convertTo(depth_image,CV_16UC1);

            sensor_msgs::ImagePtr depthImage = cvImagetoMsg(depth_image,sensor_msgs::image_encodings::MONO16,"/" + camera_name + "_depth_optical_frame");
            depth_camera_info = depth_camera_info_man->getCameraInfo();
            depth_camera_info.header.frame_id ="/" + camera_name + "_depth_optical_frame";
            depth_pub.publish(*depthImage,depth_camera_info,depthImage->header.stamp);
        }

        // only if the depth image topic is subscribed to
        if (depth_aligned_pub.getNumSubscribers() > 0)
        {
            uint16_t *depth_raw2 = (uint16_t *)dev->get_frame_data(rs::stream::depth_aligned_to_color);
            // convert to cv image and publish
            cv::Mat depth_image_raw(depth_intrin.height,depth_intrin.width,CV_16UC1,depth_raw2, cv::Mat::AUTO_STEP);

            cv::Mat depth_image(depth_intrin.height,depth_intrin.width,CV_16UC1);

            cv::Mat depth_image_scaled(depth_intrin.height,depth_intrin.width,CV_64FC1);

            depth_image_raw.copyTo(depth_image);
            depth_image_raw.convertTo(depth_image_scaled,CV_64FC1);

            depth_image_scaled *= 1000.0f*scale;

            depth_image_scaled.convertTo(depth_image,CV_16UC1);

            sensor_msgs::ImagePtr depthImage = cvImagetoMsg(depth_image,sensor_msgs::image_encodings::MONO16,"/" + camera_name + "_rgb_optical_frame");
            depth_camera_info = depth_camera_info_man->getCameraInfo();
            depth_camera_info.header.frame_id ="/" + camera_name + "_rgb_optical_frame";
            depth_aligned_pub.publish(*depthImage,depth_camera_info,depthImage->header.stamp);
        }

        // only if color image subscribed to
        if (color_pub.getNumSubscribers() > 0)
        {
            // convert to cv image and publish
            cv::Mat color_image(color_intrin.height,color_intrin.width,CV_8UC3,color_raw, cv::Mat::AUTO_STEP);
            sensor_msgs::ImagePtr colorImage = cvImagetoMsg(color_image,sensor_msgs::image_encodings::RGB8,"/" + camera_name + "_rgb_optical_frame");
            color_camera_info = color_camera_info_man->getCameraInfo();
            color_camera_info.header.frame_id ="/" + camera_name + "_rgb_optical_frame";
            color_pub.publish(*colorImage,color_camera_info,colorImage->header.stamp);
        }

        // only if depth-aligned color image subscribed to
        if (color_reg_depth_pub.getNumSubscribers() > 0)
        {
            // publish color image that has been aligned to depth
            sensor_msgs::ImagePtr colorImage = cvImagetoMsg(color_reg_image,sensor_msgs::image_encodings::RGB8,"/" + camera_name + "_depth_optical_frame");
            color_reg_camera_info = color_reg_camera_info_man->getCameraInfo();
            color_reg_camera_info.header.frame_id ="/" + camera_name + "_depth_optical_frame";
            color_reg_depth_pub.publish(*colorImage,color_reg_camera_info,colorImage->header.stamp);
        }

        // only if rgb image with depth aligned to it subscribed to
        if (depth_reg_color_pub.getNumSubscribers() > 0)
        {
            // publish color image that has had depth aligned to it
            sensor_msgs::ImagePtr colorImage = cvImagetoMsg(depth_reg_color_image,sensor_msgs::image_encodings::RGB8,"/" + camera_name + "_rgb_optical_frame");
            color_reg_camera_info = color_reg_camera_info_man->getCameraInfo();
            color_reg_camera_info.header.frame_id ="/" + camera_name + "_rgb_optical_frame";
            depth_reg_color_pub.publish(*colorImage,color_reg_camera_info,colorImage->header.stamp);
        }

        //only if ir subscribed to
        if (ir_pub.getNumSubscribers() > 0)
        {
            // obtain data, convert to cv image and publish
            uint16_t * ir_raw = (uint16_t *)dev->get_frame_data(rs::stream::infrared);
            cv::Mat ir_image(ir_intrin.height,ir_intrin.width,CV_16UC1,ir_raw, cv::Mat::AUTO_STEP);
            sensor_msgs::ImagePtr irImage = cvImagetoMsg(ir_image,sensor_msgs::image_encodings::MONO16,"/" + camera_name + "_depth_optical_frame");
            ir_camera_info = ir_camera_info_man->getCameraInfo();
            ir_camera_info.header.frame_id ="/" + camera_name + "_depth_optical_frame";
            ir_pub.publish(*irImage,ir_camera_info,irImage->header.stamp);
        }

        ros::spinOnce();
        r.sleep();

    }

}
// If there is an error calling rs function
catch(const rs::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
// some other error
catch(const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
