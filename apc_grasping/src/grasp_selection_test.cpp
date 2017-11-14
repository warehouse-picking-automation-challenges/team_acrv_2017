#include <string>
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <apc_grasping/pcl_filters.h>

#include <apc_msgs/DetectGraspCandidates.h>

#include <pcl/common/geometry.h>
#include <pcl/kdtree/kdtree_flann.h>

ros::ServiceClient grasp_service_client;
ros::NodeHandle *nh_;

void object_cloud_callback(const sensor_msgs::PointCloud2ConstPtr msg) {

  apc_msgs::DetectGraspCandidates srv;

  srv.request.cloud = *msg;
  srv.request.boundary_threshold = 0.025;

  if (grasp_service_client.call(srv)) {

    ROS_INFO("Got grasp poses");
    for (unsigned int i = 0;
         i < srv.response.grasp_candidates.grasp_utilities.size(); i++) {
      ROS_INFO_STREAM(
          "Grasp Sorted:" << srv.response.grasp_candidates.grasp_utilities[i]);
    }
  }
}

int main(int argc, char **argv) {

  std::string topicObjectCloud, graspPosesTopic, boundariesTopic,
      marker_array_topic;

  ros::init(argc, argv, "grasp_detection_test");

  nh_ = new ros::NodeHandle("~");

  nh_->param("object_cloud_topic", topicObjectCloud,
             std::string("/realsense/points"));
  nh_->param("grasp_poses_topic", graspPosesTopic, std::string("/grasp_poses"));
  nh_->param("object_boundaries_topic", boundariesTopic,
             std::string("/object_boundaries"));
  nh_->param("marker_array_topic", marker_array_topic,
             std::string("/marker_array_topic"));

  ros::Subscriber object_cloud_subscriber =
      nh_->subscribe(topicObjectCloud, 10, object_cloud_callback);

  grasp_service_client = nh_->serviceClient<apc_msgs::DetectGraspCandidates>(
      "/apc_grasping/detect_grasp_candidates");

  ros::spin();

  //      ros::ServiceServer grasp_service =
  //          nh_.advertiseService("/apc_grasping/local_grasp_estimate",
  //          &grasp_estimation);

  //      ros::Publisher grasp_pub = nh_->advertise<geometry_msgs::PoseArray>(
  //                  graspPosesTopic,
  //                  0 );

  //      ros::Publisher boundaries_pub =
  //      nh_->advertise<sensor_msgs::PointCloud2>(
  //                  boundariesTopic,
  //                  0 );
}
