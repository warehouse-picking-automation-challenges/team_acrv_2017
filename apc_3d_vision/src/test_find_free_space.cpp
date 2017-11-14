#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

/* Messages */
#include "apc_msgs/BoundingBoxDepth.h"

/* Services */
#include "apc_msgs/ReturnFreeCentroids.h"

/*
NOTE
Custom service definitions should have the same name as the service that employs
them. I.E. the find_free_space service should have a custom service definition
called FindFreeSpace.
*/

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_find_free_space");
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
    ros::ServiceClient find_free_space_client = n.serviceClient<apc_msgs::ReturnFreeCentroids>("/apc_3d_vision/find_free_space");

    // Instantiate a service class and fill the request
    apc_msgs::ReturnFreeCentroids find_free_space_srv;
    find_free_space_srv.request.cloudSearch = storage_cloud_ros;

    // Call the service
    if (find_free_space_client.call(find_free_space_srv)) {
        ROS_INFO_STREAM("Success = " << find_free_space_srv.response.success);
    } else {
        ROS_INFO_STREAM("Failed to call service!");
    }

    ros::spin();

    return 0;
}
