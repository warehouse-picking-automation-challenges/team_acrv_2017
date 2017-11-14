#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>

#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/octree/octree_pointcloud_occupancy.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

/* Messages */
#include "apc_msgs/BoundingBoxDepth.h"

/* Services */
#include "apc_msgs/ObjectPlacementPoseFromCloud.h"

/*
NOTE
Custom service definitions should have the same name as the service that employs
them. I.E. the find_free_space service should have a custom service definition
called FindFreeSpace.
*/

// TODO convert to non-colour pointcloud at start
// TODO update draw_object function to operate on position and angle


int main(int argc, char **argv) {
    ros::init(argc, argv, "test_object_placement_pose_from_cloud_node");
    ros::NodeHandle n;

    // Load a pointcloud from file
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> storage_cloud;
    storage_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("/home/adamwtow/ros_ws/storage.pcd", *storage_cloud) == -1) {
        PCL_ERROR("Couldn't read file /home/adamwtow/ros_ws/storage.pcd \n");
        return 0;
    } else {
        ROS_INFO_STREAM("Point cloud loaded...");
    }

    // Convert to ros pointcloud message
    sensor_msgs::PointCloud2 storage_cloud_ros;
    pcl::toROSMsg(*storage_cloud, storage_cloud_ros);

    // Setup a find_free_space service client
    ros::ServiceClient service_handle = n.serviceClient<apc_msgs::ObjectPlacementPoseFromCloud>("/object_placement_pose_from_cloud");

    geometry_msgs::Pose transform_from_cam_to_world;
    transform_from_cam_to_world.position.x = 0.0;
    transform_from_cam_to_world.position.y = 0.0;
    transform_from_cam_to_world.position.z = 0.0;
    transform_from_cam_to_world.orientation.x = 0.0;
    transform_from_cam_to_world.orientation.y = 0.0;
    transform_from_cam_to_world.orientation.z = 0.0;
    transform_from_cam_to_world.orientation.w = 1.0;

    // Instantiate a service class and fill the request
    apc_msgs::ObjectPlacementPoseFromCloud octomap_node_service_message;
    octomap_node_service_message.request.cropped_cloud = storage_cloud_ros;
    octomap_node_service_message.request.camera_to_world = transform_from_cam_to_world;
    octomap_node_service_message.request.object_longest_side_length.data = 0.1;
    octomap_node_service_message.request.object_middlest_side_length.data = 0.1;
    octomap_node_service_message.request.object_shortest_side_length.data = 0.1;
    octomap_node_service_message.request.resolution.data = 0.005;
    // octomap_node_service_message.request.camera_to_world = storage_cloud_ros;

    // sensor_msgs/PointCloud2 cropped_cloud
    // geometry_msgs/Pose camera_to_world
    // std_msgs/Float32 object_longest_side_length
    // std_msgs/Float32 object_middlest_side_length
    // std_msgs/Float32 object_shortest_side_length
    // std_msgs/Float32 resolution
    // ---
    // std_msgs/Float32 x
    // std_msgs/Float32 y
    // std_msgs/Float32 z
    // std_msgs/Float32 degrees_to_global_y
    // std_msgs/Bool success


    // Call the service
    if (service_handle.call(octomap_node_service_message)) {
        ROS_INFO_STREAM("Success = " << octomap_node_service_message.response.success);
    } else {
        ROS_INFO_STREAM("Failed to call service!");
    }

    return 0;
}
