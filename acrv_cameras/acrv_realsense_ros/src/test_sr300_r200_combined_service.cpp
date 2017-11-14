#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

/* Services */
#include "apc_msgs/GetCombinedSR300R200Cloud.h"

/*
NOTE
Custom service definitions should have the same name as the service that employs
them. I.E. the find_free_space service should have a custom service definition
called FindFreeSpace.
*/

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_sr300_r200_combined_service");
    ros::NodeHandle n;

    // Setup a sr300_r200_combined_service service client
    ros::ServiceClient sr300_r200_combined_service_client = n.serviceClient<apc_msgs::GetCombinedSR300R200Cloud>("/get_combined_sr300_r200_cloud");

    // Instantiate a service class and fill the request
    apc_msgs::GetCombinedSR300R200Cloud get_combined_cloud_srv;

    pcl::PointCloud<pcl::PointXYZRGB> combined_cloud;

    // Call the service
    if (sr300_r200_combined_service_client.call(get_combined_cloud_srv)) {
        pcl::fromROSMsg(get_combined_cloud_srv.response.combined_cloud, combined_cloud);
        pcl::io::savePCDFileASCII("combined_cloud.pcd", combined_cloud);
        ROS_INFO_STREAM("Saved combined_cloud.pcd");
    } else {
        ROS_INFO_STREAM("Failed to call service!");
    }

    // ros::spin();

    return 0;
}
