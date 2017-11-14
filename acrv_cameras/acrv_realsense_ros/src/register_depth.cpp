#include <ros/ros.h>
#include <ros/console.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <librealsense/rs.hpp>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <opencv2/core/core.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>
#include <math.h>
#include <thread>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>
#include <boost/algorithm/string.hpp>


namespace rs
{
  /// \brief Video stream intrinsics
  struct my_intrinsics : rs_intrinsics
  {
      float       hfov() const                                                        { return (atan2f(ppx + 0.5f, fx) + atan2f(width - (ppx + 0.5f), fx)) * 57.2957795f; }
      float       vfov() const                                                        { return (atan2f(ppy + 0.5f, fy) + atan2f(height - (ppy + 0.5f), fy)) * 57.2957795f; }
      distortion  my_model() const                                                       { return (distortion)rs_intrinsics::model; }

                  // Helpers for mapping between pixel coordinates and texture coordinates
      float2      pixel_to_texcoord(const float2 & pixel) const                       { return {(pixel.x+0.5f)/width, (pixel.y+0.5f)/height}; }
      float2      texcoord_to_pixel(const float2 & coord) const                       { return {coord.x*width - 0.5f, coord.y*height - 0.5f}; }

                  // Helpers for mapping from image coordinates into 3D space
      float3      deproject(const float2 & pixel, float depth) const                  { float3 point = {}; rs_deproject_pixel_to_point(&point.x, this, &pixel.x, depth); return point; }
      float3      deproject_from_texcoord(const float2 & coord, float depth) const    { return deproject(texcoord_to_pixel(coord), depth); }

                  // Helpers for mapping from 3D space into image coordinates
      float2      project(const float3 & point) const                                 { float2 pixel = {}; rs_project_point_to_pixel(&pixel.x, this, &point.x); return pixel; }
      float2      project_to_texcoord(const float3 & point) const                     { return pixel_to_texcoord(project(point)); }

      bool        operator == (const intrinsics & r) const                            { return memcmp(this, &r, sizeof(r)) == 0; }

  };
}


std::string camera_name;

rs::my_intrinsics depth_intrin;
rs::extrinsics depth_to_color;
rs::my_intrinsics color_intrin;

ros::Publisher depth_registered_pub;
ros::Publisher depth_registered_camera_info_pub;
ros::Publisher rgb_sync_depth_registered_pub;
ros::Publisher rgb_sync_depth_registered_camera_info_pub;

double x_param, y_param, z_param, yaw_param, pitch_param, roll_param;

// sensor_msgs::CameraInfo rgb_sync_depth_registered_camera_info, depth_camera_info;


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
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_mat_to_sensor_msgs_image FAILED with: %s", e.what());
    }

    return a;
}

// sensor_msgs::Image to cv::Mat
cv_bridge::CvImagePtr sensor_msgs_image_to_cv_mat(const sensor_msgs::Image& msg, std::string encoding) {
    cv_bridge::CvImagePtr cv_im;
    try {
        cv_im = cv_bridge::toCvCopy(msg, encoding);
    }
    catch(cv_bridge::Exception& e) {
        ROS_ERROR("sensor_msgs_image_to_cv_mat FAILED with: %s", e.what());
    }
    return cv_im;
}

void callback(const sensor_msgs::Image& depth_raw_msg, const sensor_msgs::CameraInfo& depth_camera_info_msg, const sensor_msgs::Image& rgb_rect_msg, const sensor_msgs::CameraInfo& rgb_camera_info_msg) {

    sensor_msgs::CameraInfo rgb_sync_depth_registered_camera_info = rgb_camera_info_msg;
    sensor_msgs::CameraInfo depth_camera_info = depth_camera_info_msg;
    sensor_msgs::Image rgb_sync_depth_registered_msg = rgb_rect_msg;

    color_intrin.model = RS_DISTORTION_NONE;
    color_intrin.height = rgb_sync_depth_registered_camera_info.height;
    color_intrin.width = rgb_sync_depth_registered_camera_info.width;

    color_intrin.fx=rgb_sync_depth_registered_camera_info.K[0];
    color_intrin.fy=rgb_sync_depth_registered_camera_info.K[4];
    color_intrin.ppx=rgb_sync_depth_registered_camera_info.K[2];
    color_intrin.ppy=rgb_sync_depth_registered_camera_info.K[5];
    color_intrin.coeffs[0]=rgb_sync_depth_registered_camera_info.D[0];
    color_intrin.coeffs[1]=rgb_sync_depth_registered_camera_info.D[1];
    color_intrin.coeffs[2]=rgb_sync_depth_registered_camera_info.D[2];
    color_intrin.coeffs[3]=rgb_sync_depth_registered_camera_info.D[3];
    color_intrin.coeffs[4]=rgb_sync_depth_registered_camera_info.D[4];


    depth_intrin.model = RS_DISTORTION_INVERSE_BROWN_CONRADY;
    depth_intrin.height = depth_camera_info.height;
    depth_intrin.width = depth_camera_info.width;

    depth_intrin.fx=depth_camera_info.K[0];
    depth_intrin.fy=depth_camera_info.K[4];
    depth_intrin.ppx=depth_camera_info.K[2];
    depth_intrin.ppy=depth_camera_info.K[5];
    depth_intrin.coeffs[0]=depth_camera_info.D[0];
    depth_intrin.coeffs[1]=depth_camera_info.D[1];
    depth_intrin.coeffs[2]=depth_camera_info.D[2];
    depth_intrin.coeffs[3]=depth_camera_info.D[3];
    depth_intrin.coeffs[4]=depth_camera_info.D[4];

    // Transform depth to rgb
    // x y z yaw pitch roll
    // -0.02426723 -0.00047571 -0.00023367 -0.0001231  -0.01946618 -0.01579435
    // angle's in degrees to support online converter here: http://danceswithcode.net/engineeringnotes/rotations_in_3d/demo3D/rotations_in_3d_tool.html
    // -0.00705311045808 -1.115329957245 -0.9049495951547

    // double yaw = -0.0001231;
    // double pitch = -0.01946618;
    // double roll = -0.01579435;
    // yaw in ros is roll in realsense
    // pitch in ros is yaw in realsense
    // roll in ros is pitch in realsense

    // Current camera is one used to collect dataset
    double pitch = roll_param;
    double yaw = pitch_param;
    double roll = yaw_param;
    // double yaw = roll_param;
    // double pitch = pitch_param;
    // double roll = yaw_param;
    // double roll = roll_param;
    // double pitch = pitch_param;
    // double yaw = yaw_param;

    double x = x_param;
    double y = y_param;
    double z = z_param;

    // Flip from ros to realsense - I guess convention is different
    x = -x;
    y = -y;
    z = -z;

    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitX());

    // NOTE flipping the unit axes as below didn't work
    // Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitY());
    // Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitX());
    // Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitZ());

    // Y is yaw
    // Z is roll
    // X is pitch

    Eigen::Quaternion<double> q = rollAngle * pitchAngle * yawAngle;

    Eigen::Matrix3d rotationMatrix = q.matrix();

    // ROS_INFO_STREAM(rotationMatrix);

    depth_to_color.rotation[0]=rotationMatrix(0,0);
    depth_to_color.rotation[1]=rotationMatrix(0,1);
    depth_to_color.rotation[2]=rotationMatrix(0,2);
    depth_to_color.rotation[3]=rotationMatrix(1,0);
    depth_to_color.rotation[4]=rotationMatrix(1,1);
    depth_to_color.rotation[5]=rotationMatrix(1,2);
    depth_to_color.rotation[6]=rotationMatrix(2,0);
    depth_to_color.rotation[7]=rotationMatrix(2,1);
    depth_to_color.rotation[8]=rotationMatrix(2,2);
    // depth_to_color.rotation[0]=0.9998;depth_to_color.rotation[1]=0.0004;depth_to_color.rotation[2]=-0.0195;
    // depth_to_color.rotation[3]=-0.0002;depth_to_color.rotation[4]=0.9998;depth_to_color.rotation[5]=0.0157;
    // depth_to_color.rotation[6]=0.0194;depth_to_color.rotation[7]=-0.0158;depth_to_color.rotation[8]=0.9996;
    // depth_to_color.translation[0]=0.0239998574975702;
    // depth_to_color.translation[1]=-0.0000631696874657;
    // depth_to_color.translation[2]=0.0002111893155177;
    depth_to_color.translation[0]=x;
    depth_to_color.translation[1]=y;
    depth_to_color.translation[2]=z;


    cv_bridge::CvImagePtr depth_cv_ptr = sensor_msgs_image_to_cv_mat(depth_raw_msg, sensor_msgs::image_encodings::TYPE_16UC1);

    cv::Mat depth_registered_cv(color_intrin.height, color_intrin.width, CV_32FC1);

    // Pre-fill image with 0's to account for pixels that fall outside of depth image sweep
    for (int h=0;h<color_intrin.height;h++) {
        for (int w=0;w<color_intrin.width;w++) {
            depth_registered_cv.at<float>(cv::Point(w,h)) = 0;
        }
    }

    for (int dy=0; dy<depth_intrin.height; ++dy){
        for (int dx=0; dx<depth_intrin.width; ++dx){
            // Obtain the depth value and apply scale factor
            uint16_t depth_value = depth_cv_ptr->image.at<uint16_t>(cv::Point(dx,dy));
            float depth_in_meters = depth_value / 1000.0;

            // Map from pixel coordinates in the depth image to pixel coordinates in the color image
            rs::float2 depth_pixel = {(float)dx, (float)dy};
            rs::float3 depth_point = depth_intrin.deproject(depth_pixel, depth_in_meters);
            rs::float3 color_point = depth_to_color.transform(depth_point);
            rs::float2 color_pixel = color_intrin.project(color_point);

            // Choose the nearest pixel in the color image to the reprojected depth pixel
            // Ignore this point if it falls outside the color image
            const int cx = (int)std::round(color_pixel.x), cy = (int)std::round(color_pixel.y);
            if (!(cx < 0 || cy < 0 || cx >= color_intrin.width || cy >= color_intrin.height))
            {
                depth_registered_cv.at<float>(cv::Point(cx,cy)) = depth_in_meters;
            }
        }
    }

    sensor_msgs::ImagePtr depth_registered_msg_ptr = cv_mat_to_sensor_msgs_image(depth_registered_cv, sensor_msgs::image_encodings::TYPE_32FC1, camera_name + "_rgb_optical_frame");

    // Update timestamps
    ros::Time time_now = ros::Time::now();
    depth_registered_msg_ptr->header.stamp = time_now;
    rgb_sync_depth_registered_camera_info.header.stamp = time_now;
    rgb_sync_depth_registered_msg.header.stamp = time_now;

    depth_registered_pub.publish(*depth_registered_msg_ptr);
    depth_registered_camera_info_pub.publish(rgb_sync_depth_registered_camera_info);
    rgb_sync_depth_registered_pub.publish(rgb_sync_depth_registered_msg);
    rgb_sync_depth_registered_camera_info_pub.publish(rgb_sync_depth_registered_camera_info);
}

int main(int argc, char * argv[]) {
    ros::init(argc, argv, "register_depth_node");
    ros::NodeHandle n("~");

    if (n.getParam("camera_name", camera_name)) {
        ROS_INFO_STREAM("\"camera_name\" = " << camera_name);
    } else {
        ROS_ERROR_STREAM("\"camera_name\" not provided.");
        return 0;
    }

    std::string xyzypr;
    std::vector<std::string> xyzypr_vector;
    if (n.getParam("xyzypr", xyzypr)) {
        ROS_INFO_STREAM("\"xyzypr\" = " << xyzypr);
        boost::split(xyzypr_vector, xyzypr, boost::is_any_of(" "));
    } else {
        ROS_ERROR_STREAM("\"xyzypr\" not provided.");
        return 0;
    }

    x_param = std::stod(xyzypr_vector[0]);
    y_param = std::stod(xyzypr_vector[1]);
    z_param = std::stod(xyzypr_vector[2]);
    yaw_param = std::stod(xyzypr_vector[3]);
    pitch_param = std::stod(xyzypr_vector[4]);
    roll_param = std::stod(xyzypr_vector[5]);

    ROS_INFO_STREAM("xyzypr = " << x_param
                    << ", " << y_param
                    << ", " << z_param
                    << ", " << yaw_param
                    << ", " << pitch_param
                    << ", " << roll_param);

    message_filters::Subscriber<sensor_msgs::Image> depth_image_raw_sub(n, "/" + camera_name + "/depth/image_raw", 5);
    message_filters::Subscriber<sensor_msgs::CameraInfo> depth_camera_info_sub(n, "/" + camera_name + "/depth/camera_info", 5);
    message_filters::Subscriber<sensor_msgs::Image> rgb_image_rect_sub(n, "/" + camera_name + "/rgb/image_raw", 5);
    message_filters::Subscriber<sensor_msgs::CameraInfo> rgb_camera_info_sub(n, "/" + camera_name + "/rgb/camera_info", 5);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image, sensor_msgs::CameraInfo> MySyncPolicy;

    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(5), depth_image_raw_sub, depth_camera_info_sub, rgb_image_rect_sub, rgb_camera_info_sub);
    sync.registerCallback(callback);

    depth_registered_pub = n.advertise<sensor_msgs::Image>("/" + camera_name + "/depth_registered/image_rect", 30);
    depth_registered_camera_info_pub = n.advertise<sensor_msgs::CameraInfo>("/" + camera_name + "/depth_registered/camera_info", 30);
    rgb_sync_depth_registered_pub = n.advertise<sensor_msgs::Image>("/" + camera_name + "/rgb_sync_depth_registered/image_rect", 30);
    rgb_sync_depth_registered_camera_info_pub = n.advertise<sensor_msgs::CameraInfo>("/" + camera_name + "/rgb_sync_depth_registered/camera_info", 30);

    ros::Rate r(30);

    // Continuously publish rgb image and depth_inpainted image
    while(ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }
}
