/*
Copyright 2016 Australian Centre for Robotic Vision
*/

#include "tf/transform_datatypes.h"
#include <pcl/filters/passthrough.h>
#include <math.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <vector>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <apc_3d_vision/split_labelled_point_cloud.h>
#include <apc_3d_vision.hpp>
#include <eigen_conversions/eigen_msg.h>
#include "tf_conversions/tf_eigen.h"
#include <map>
#include <utility>
#include <vector>
#include <stdio.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread/thread.hpp>
#include <apc_msgs/CropCloud.h>
#include <pcl/filters/statistical_outlier_removal.h>
// global value to listen to transforms
boost::shared_ptr<tf::TransformListener> tf_listener_ptr;
tf::Transform tftote;
Apc3dVision apc_vis;
ros::Publisher debug1;
ros::Publisher debug2;

// Set a global value to us PI in shorthand
#define PI 3.14159265
// values to help with Calibration
int value = 0;
int valueMax = 255;
bool CALIBRATE = false;
bool publishTF = false;
// value to set calibration window name
static const std::string Window = "CALIBRATE windows";
// The results from calibration
int Hlow; // = 20; //20
int Hhigh; // = 40; //32
int Slow; // = 0;
int Shigh; // = 0;
int Vlow; // = 255;
int Vhigh; // = 255;

bool Hinside;

float Xmin;
float Xmax;
float Ymin;
float Ymax;
float Zmin;
float Zmax;

int img_top;
int img_bottom;
int img_left;
int img_right;

bool lookupTransform(const std::string &fromFrame, const std::string &toFrame,
                     tf::StampedTransform &foundTransform) {
  try {
    ros::Time now = ros::Time::now();
    ros::Time zero = ros::Time(0);
    if (!tf_listener_ptr)
      ROS_ERROR("asdfsdaf");

    bool tf_success = tf_listener_ptr->waitForTransform(
        fromFrame, toFrame, zero, ros::Duration(3.0));

    if (tf_success) {
      tf_listener_ptr->lookupTransform(fromFrame, toFrame, zero,
                                       foundTransform);
    } else {
      ROS_WARN("Could not lookup transform.");
    }
  } catch (tf::TransformException ex) {
    ROS_ERROR("TransformException: %s", ex.what());
    return false;
  } catch (...) {
    ROS_ERROR("Unknown exception.");
    return false;
  }
  return true;
}
// Callback functions to set the values of the calibration
void ChangeHlow(int, void *) { Hlow = uint8_t(value); }
void ChangeSlow(int, void *) { Slow = uint8_t(value); }
void ChangeVlow(int, void *) { Vlow = uint8_t(value); }
void ChangeHhigh(int, void *) { Hhigh = uint8_t(value); }
void ChangeShigh(int, void *) { Shigh = uint8_t(value); }
void ChangeVhigh(int, void *) { Vhigh = uint8_t(value); }

bool split_labelled_point_cloud(apc_msgs::CropCloud::Request &req,
                                apc_msgs::CropCloud::Response &res) {
  ros::NodeHandle nh("~");

  bool crop_image;
  try {
    crop_image = req.crop_image.data;
  } catch (...) {
    crop_image = false;
  }

  bool crop_cloud;
  try {
    crop_cloud = req.crop_cloud.data;
  } catch (...) {
    crop_cloud = false;
  }

  std::string storage_name = req.storage_name.data;

  if(crop_cloud) {
    nh.getParam(std::string("/storage_specs/") + storage_name + "/h_low", Hlow);
    nh.getParam(std::string("/storage_specs/") + storage_name + "/h_high", Hhigh);
    nh.getParam(std::string("/storage_specs/") + storage_name + "/s_low", Slow);
    nh.getParam(std::string("/storage_specs/") + storage_name + "/s_high", Shigh);
    nh.getParam(std::string("/storage_specs/") + storage_name + "/v_low", Vlow);
    nh.getParam(std::string("/storage_specs/") + storage_name + "/v_high", Vhigh);

    nh.getParam(std::string("/storage_specs/") + storage_name + "/h_inside", Hinside);

    nh.getParam(std::string("/storage_specs/") + storage_name + "/x_max", Xmax);
    nh.getParam(std::string("/storage_specs/") + storage_name + "/x_min", Xmin);
    nh.getParam(std::string("/storage_specs/") + storage_name + "/y_max", Ymax);
    nh.getParam(std::string("/storage_specs/") + storage_name + "/y_min", Ymin);
    nh.getParam(std::string("/storage_specs/") + storage_name + "/z_max", Zmax);
    nh.getParam(std::string("/storage_specs/") + storage_name + "/z_min", Zmin);

    // Get the pointcloud out of the Request
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> req_points;
    req_points.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    try {
      pcl::fromROSMsg(req.input_cloud, *req_points);
    } catch (...) {
      // std::cout << "There is now points in this message!" << std::endl;
      res.success.data = false;
      return false;
    }

    int kept = 0;
    int removed = 0;
    int outside = 0;

    pcl::PointCloud<pcl::PointXYZRGB> croppedCloud_outlier;
    pcl::PointCloud<pcl::PointXYZRGB> croppedCloud_inlier;
    pcl::PointCloud<pcl::PointXYZRGB> croppedCloud_posonly;
    if (CALIBRATE) {
      cv::Mat Color_cropped = cv::Mat(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
      int i = 0;
      BOOST_FOREACH (pcl::PointXYZRGB point, *req_points) {
        i++;
        if (point.r == 0) {
          continue;
        }
        cv::Mat M = cv::Mat(1, 1, CV_8UC3, cv::Scalar(point.r, point.g, point.b));
        cv::Mat M_hsv;
        cvtColor(M, M_hsv, CV_RGB2HSV);
        if (((M_hsv.data[0] > Hlow) && (M_hsv.data[0] < Hhigh)) &&
            ((M_hsv.data[1] < Slow) && (M_hsv.data[1] < Shigh)) &&
            ((M_hsv.data[2] > Vlow) && (M_hsv.data[2] < Vhigh))) {
          croppedCloud_outlier.push_back(point);
          Color_cropped.data[3 * i + 2] = point.r;
          Color_cropped.data[3 * i + 1] = point.g;
          Color_cropped.data[3 * i] = point.b;
        }
      }

    } else {

      pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
      sor.setMeanK(25);
      sor.setStddevMulThresh(1.0);
      sor.setInputCloud(req_points);
      sor.filter(*req_points);

      BOOST_FOREACH (pcl::PointXYZRGB point, *req_points) {
      //for(auto &point: *req_points) {
        if (point.r == 0) {
          continue;
        }
        cv::Mat M = cv::Mat(1, 1, CV_8UC3, cv::Scalar(point.r, point.g, point.b));
        cv::Mat M_hsv;
        cvtColor(M, M_hsv, CV_RGB2HSV);
        if (
          (point.x < Xmin) || (point.x > Xmax) ||
          (point.y < Ymin) || (point.y > Ymax) ||
          (point.z < Zmin) || (point.z > Zmax)
        ) {
          outside++;
        }
        else if (
            ((Hinside && ((M_hsv.data[0] > Hlow) && (M_hsv.data[0] < Hhigh))) ||
            (!Hinside && ((M_hsv.data[0] < Hlow) || (M_hsv.data[0] > Hhigh)))) &&
            ((M_hsv.data[1] > Slow) && (M_hsv.data[1] < Shigh)) &&
            ((M_hsv.data[2] > Vlow) && (M_hsv.data[2] < Vhigh))
        ) {
          // Keep the point
          croppedCloud_posonly.push_back(point);
          croppedCloud_inlier.push_back(point);
          kept++;
        } else {
          // Remove the point
          croppedCloud_posonly.push_back(point);
          croppedCloud_outlier.push_back(point);
          removed++;
        }
      }
      std::cout << "Cropping Tote" << std::endl;
    }

    sensor_msgs::PointCloud2 ros_cloud_inliers;
    sensor_msgs::PointCloud2 ros_cloud_outliers;
    sensor_msgs::PointCloud2 ros_cloud_posonly;
    try {
      pcl::toROSMsg(croppedCloud_inlier, ros_cloud_inliers);
      pcl::toROSMsg(croppedCloud_outlier, ros_cloud_outliers);
      pcl::toROSMsg(croppedCloud_posonly, ros_cloud_posonly);
      ros_cloud_inliers.header.stamp = ros::Time::now();
      ros_cloud_inliers.header.frame_id = req.input_cloud.header.frame_id;
      ros_cloud_outliers.header.stamp = ros::Time::now();
      ros_cloud_outliers.header.frame_id = req.input_cloud.header.frame_id;
      ros_cloud_posonly.header.stamp = ros::Time::now();
      ros_cloud_posonly.header.frame_id = req.input_cloud.header.frame_id;
      res.segmented_cloud_inliers = ros_cloud_inliers;
      res.segmented_cloud_outliers = ros_cloud_outliers;
      res.segmented_cloud_posonly = ros_cloud_posonly;
    }
    catch (...) {
      std::cout << "You filtered everything and there's nothing in this cloud!"
                << std::endl;
      res.success.data = false;
      return false;
    }
  }

  if(crop_image) {
    nh.getParam(std::string("/storage_specs/") + storage_name + "/img_top", img_top);
    nh.getParam(std::string("/storage_specs/") + storage_name + "/img_bottom", img_bottom);
    nh.getParam(std::string("/storage_specs/") + storage_name + "/img_left", img_left);
    nh.getParam(std::string("/storage_specs/") + storage_name + "/img_right", img_right);


    // Load the image.
    cv_bridge::CvImagePtr image_ptr;
    cv::Mat img;
    try{
        image_ptr = cv_bridge::toCvCopy(req.image, "");
        img = image_ptr->image;
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    // Load the Depth
    cv_bridge::CvImagePtr depth_ptr;
    cv::Mat depth;
    try{
        depth_ptr = cv_bridge::toCvCopy(req.depth, "");
        depth = depth_ptr->image;
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    // Create a rectangular mask.
    cv::Mat mask = cv::Mat::zeros(img.size(), CV_8U);
    mask(cv::Rect(img_left, img_top, img_right-img_left, img_bottom-img_top)) = 1;

    // Crop the image.
    cv::Mat cropped_img = cv::Mat::zeros(img.size(), img.type());
    img.copyTo(cropped_img, mask);
    image_ptr->image = cropped_img;
    image_ptr->toImageMsg(res.cropped_image);

    // Crop the Depth.
    cv::Mat cropped_depth = cv::Mat::zeros(depth.size(), depth.type());
    depth.copyTo(cropped_depth, mask);
    depth_ptr->image = cropped_depth;
    depth_ptr->toImageMsg(res.cropped_depth);

  }


  res.success.data = true;
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "cropTote");
  ros::NodeHandle nh_("~");
  // transform broadcaster to broadcast the recieved TF
  tf::TransformBroadcaster br;
  // Advertise the service to crop the cloud
  ros::ServiceServer service = nh_.advertiseService(
      "/apc_3d_vision/Crop_Tote_cloud", &split_labelled_point_cloud);
  tf_listener_ptr.reset(new tf::TransformListener());

  debug1 = nh_.advertise<sensor_msgs::PointCloud2>("/debug1", 1);
  ;
  debug2 = nh_.advertise<sensor_msgs::PointCloud2>("/debug2", 1);
  ;

  ros::AsyncSpinner spinner(2);
  spinner.start();
  if (CALIBRATE) {
    cv::namedWindow(Window);
    // char TrackbarName[50];
    // sprintf(TrackbarName, "Hlow %d", valueMax);
    cv::createTrackbar("H>Hlow", Window, &value, valueMax, ChangeHlow);

    // char TrackbarName2[50];
    // sprintf(TrackbarName2, "Slow %d", valueMax);
    cv::createTrackbar("S>Slow", Window, &value, valueMax, ChangeSlow);
    // char TrackbarName3[50];
    // sprintf(TrackbarName3, "Vlow %d", valueMax);
    cv::createTrackbar("V>Vlow", Window, &value, valueMax, ChangeVlow);

    // char TrackbarName4[50];
    // sprintf(TrackbarName4, "Hhigh %d", valueMax);
    cv::createTrackbar("Hhigh<H", Window, &value, valueMax, ChangeHhigh);
    // char TrackbarName5[50];
    // sprintf(TrackbarName5, "Shigh %d", valueMax);
    cv::createTrackbar("Shigh<S", Window, &value, valueMax, ChangeShigh);
    // char TrackbarName6[50];
    // sprintf(TrackbarName6, "Vhigh %d", valueMax);
    cv::createTrackbar("Vhigh<V", Window, &value, valueMax, ChangeVhigh);
  }
  ros::Rate rosrate(10);
  while (ros::ok()) {
    // if (publishTF)
      // br.sendTransform(tf::StampedTransform(tftote, ros::Time::now(), "torso",
      //                                       "totePosition"));
    rosrate.sleep();
  }

  return 0;
}
