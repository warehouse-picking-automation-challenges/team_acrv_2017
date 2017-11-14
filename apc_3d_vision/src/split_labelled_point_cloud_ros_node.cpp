/*
Copyright 2016 Australian Centre for Robotic Vision
*/

#include <ros/ros.h>
#include <apc_3d_vision/split_labelled_point_cloud.h>  // links to my srv file
#include <apc_3d_vision.hpp>

#include <map>
#include <utility>
#include <vector>

Apc3dVision apc_vis;

bool split_labelled_point_cloud(
    apc_3d_vision::split_labelled_point_cloud::Request &req,
    apc_3d_vision::split_labelled_point_cloud::Response &res) {

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZL>> labelled_points;
    labelled_points.reset(new pcl::PointCloud<pcl::PointXYZL>);
    std::map<int, boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>>
        labelled_points_map;

    pcl::fromROSMsg(req.labelled_points, *labelled_points);

    std::cout << "Labelled Point Cloud Size: " << labelled_points->size()
        << std::endl;

    // Pre-allocate memory for split point clouds
    std::vector<int> labels(apc_vis.get_labels_in_point_cloud(labelled_points));
    std::cout << "There are " << labels.size() << " labels\n";
    int numberOfLabels = labels.size();
    for (int i = 0; i < numberOfLabels; i++) {
        labelled_points_map[i].reset(new pcl::PointCloud<pcl::PointXYZ>);
    }
    apc_vis.split_labelled_point_cloud(labelled_points, labelled_points_map);

    std::cout << "Is labelled_points_map empty? "
        << labelled_points_map.empty() << std::endl;

    std::pair<int, boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> o_pair;

    int i = 0;

    BOOST_FOREACH(o_pair, labelled_points_map) {
        sensor_msgs::PointCloud2 temp_message;
        pcl::toROSMsg(*o_pair.second, temp_message);
        res.labelled_points_array.push_back(temp_message);
        i++;
    }

    std::cout << "labelled_points_array is " << i << " elements long.\n";

    res.success.data = true;
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "split_labelled_point_cloud_ros_node");

    ros::NodeHandle nh_("~");

    ros::ServiceServer service = nh_.advertiseService(
        "/apc_3d_vision/split_labelled_point_cloud", &split_labelled_point_cloud);

    ros::AsyncSpinner spinner(2);
    spinner.start();

    while (ros::ok()) {}

    return 0;
}
