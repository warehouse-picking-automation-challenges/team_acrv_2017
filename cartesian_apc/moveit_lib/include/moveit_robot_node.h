#ifndef MOVEIT_ROBOT_NODE_H
#define MOVEIT_ROBOT_NODE_H

//////////////////////////////////
//ROS Includes

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

//////////////////////////////////
//OPENCV Includes

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>


//////////////////////////////////
//STD Includes

#include <string>
#include <deque>
#include <vector>



#endif // MOVEIT_ROBOT_NODE_H
