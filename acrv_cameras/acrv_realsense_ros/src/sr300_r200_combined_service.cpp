#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <string>
#include <vector>

#include <ros/callback_queue.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/octree/octree_pointcloud_occupancy.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/pca.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
// #include <pcl/segmentation/segment_differences.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/registration/icp.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/kdtree/kdtree.h>
#include <pcl/impl/pcl_base.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/search/impl/organized.hpp>
#include <pcl/features/impl/normal_3d.hpp>
// #include <pcl/registration/sample_consensus_prerejective.h>
// #include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/mls.h>

#include <pcl_ros/point_cloud.h>

// Service definitions:
#include <apc_msgs/GetCombinedSR300R200Cloud.h>


boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> r200_cloud;
boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> sr300_cloud;

ros::Subscriber sub_r200_points;
ros::Subscriber sub_sr300_points;

bool r200_cloud_received = false;
bool sr300_cloud_received = false;


void r200_cloud_callback(const sensor_msgs::PointCloud2::ConstPtr msg) {
    // ROS_INFO_STREAM("Received r200 msg");
    pcl::fromROSMsg(*msg, *r200_cloud);

    pcl::io::savePCDFileASCII("r200_cloud.pcd", *r200_cloud);

    // Use max depth of 2m for R200
    double max_depth = 1.5;
    pcl::PointCloud<pcl::PointXYZRGB>::iterator itr = (*r200_cloud).begin();
    for (; itr != (*r200_cloud).end(); ++itr) {
        if (itr->z > max_depth) {
           itr->z = NAN;
        }
    }
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*r200_cloud, *r200_cloud, indices);

    pcl::io::savePCDFileASCII("r200_cloud_cropped_depth.pcd", *r200_cloud);

    r200_cloud_received = true;
    sub_r200_points.shutdown();
}

void sr300_cloud_callback(const sensor_msgs::PointCloud2::ConstPtr msg) {
    // ROS_INFO_STREAM("Received sr300 msg");
    pcl::fromROSMsg(*msg, *sr300_cloud);
    // std::vector<int> indices;
    // pcl::removeNaNFromPointCloud(*sr300_cloud, *sr300_cloud, indices);
    // pcl::io::savePCDFileASCII("sr300_cloud.pcd", *sr300_cloud);
    sr300_cloud_received = true;
    sub_sr300_points.shutdown();
}

bool approximate_voxel_grid(
    const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>> input_cloud,
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> output_cloud,
    double leaf_size, bool verbose) {
    if (input_cloud->size() == 0) {
        std::cout << "You gave me an empty cloud mate. What are you even doing." << std::endl;
        return false;
    }

    pcl::VoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
    approximate_voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
    approximate_voxel_filter.setInputCloud(input_cloud);
    approximate_voxel_filter.filter(*output_cloud);

    if (verbose) {
        std::cout << "Downsampled cloud contains: " << output_cloud->size()
                  << std::endl;
    }

    return true;
}

bool outlier_removal(
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> input_cloud,
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> output_cloud,
    double outlierNumberOfSamples, double stdDevMulThresh) {
    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(input_cloud);
    sor.setMeanK(outlierNumberOfSamples);
    sor.setStddevMulThresh(stdDevMulThresh);
    sor.filter(*output_cloud);

    return true;
}

bool get_combined_sr300_r200_cloud_service_callback(apc_msgs::GetCombinedSR300R200Cloud::Request &req,
                                                    apc_msgs::GetCombinedSR300R200Cloud::Response &res) {
    //
    ros::NodeHandle n;

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> _sr300_cloud;
    _sr300_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> _r200_cloud;
    _r200_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

    sub_sr300_points = n.subscribe("/realsense_wrist_local/depth_registered/points", 1, sr300_cloud_callback);
    ROS_INFO_STREAM("Waiting for sr300 cloud");
    n.setParam("/realsense_camera_ag_node/emitter_enabled", 0);  // Just using this as an extra wait
    n.setParam("/realsense_camera_ag_node/emitter_enabled", 1);
    while (!sr300_cloud_received) {
        ros::spinOnce();
    }
    ROS_INFO_STREAM("Got sr300 cloud");
    sr300_cloud_received = false;
    pcl::copyPointCloud(*sr300_cloud, *_sr300_cloud);
    n.setParam("/realsense_camera_ag_node/emitter_enabled", 0);
    n.setParam("/realsense_camera_r200_node/emitter_enabled", 1);

    sub_r200_points = n.subscribe("/realsense/points", 1, r200_cloud_callback);
    ROS_INFO_STREAM("Waiting for r200 cloud");
    while (!r200_cloud_received) {
        ros::spinOnce();
    }
    ROS_INFO_STREAM("Got r200 cloud");
    r200_cloud_received = false;
    pcl::copyPointCloud(*r200_cloud, *_r200_cloud);
    n.setParam("/realsense_camera_ag_node/emitter_enabled", 1);
    n.setParam("/realsense_camera_r200_node/emitter_enabled", 0);

    /*
    Downsample Input Cloud
    */
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> l_input_cloud;
    l_input_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

    approximate_voxel_grid(_r200_cloud, l_input_cloud, 0.01, true);
    // l_input_cloud = _r200_cloud;

    /*
    Downsample Target Cloud
    */
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> l_target_cloud;
    l_target_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

    approximate_voxel_grid(_sr300_cloud, l_target_cloud, 0.01, true);
    // l_target_cloud = _sr300_cloud;

    outlier_removal(l_input_cloud, l_input_cloud, 50.0, 1.0);
    outlier_removal(l_target_cloud, l_target_cloud, 50.0, 1.0);

    pcl::io::savePCDFileBinary("sr300_cloud_approximate_outlier_binary.pcd", *l_target_cloud);
    pcl::io::savePCDFileBinary("r200_cloud_approximate_outlier_binary.pcd", *l_input_cloud);

    // /*

    // Initializing Iterative Closest Point (ICP).
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

    // Setting point cloud to be aligned to.
    icp.setInputTarget(l_target_cloud);

    // Set the max correspondence distance to 5cm (e.g., correspondences with
    // higher distances will be ignored)
    // icp.setMaxCorrespondenceDistance(0.1);
    icp.setMaxCorrespondenceDistance(0.05);  // worked best
    // icp.setMaxCorrespondenceDistance(0.01);
    // Set the maximum number of iterations (criterion 1)
    // icp.setMaximumIterations(50);
    // Set the transformation epsilon (criterion 2)
    // icp.setTransformationEpsilon(1e-8);
    // Set the euclidean distance difference epsilon (criterion 3)
    // icp.setEuclideanFitnessEpsilon(1);

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> reg_result;
    reg_result.reset(new pcl::PointCloud<pcl::PointXYZ>);
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> transformed;
    transformed.reset(new pcl::PointCloud<pcl::PointXYZ>);

    icp.setInputSource(transformed);

    double fitness_score = 1.0;
    double lowest_fitness_score = 100.0;
    Eigen::Affine3f best_guess = Eigen::Affine3f::Identity();
    Eigen::Affine3f best_final = Eigen::Affine3f::Identity();
    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

    for (double x = -0.1; x < 0.1; x+=0.05) {
        for (double y = -0.1; y < 0.1; y+=0.05) {
            for (double z = -0.1; z < 0.1; z+=0.05) {
                ROS_INFO_STREAM("x = " << x << ", y = " << y << ", z = " << z);
                transform_2.translation() << x, y, z;

                pcl::transformPointCloud(*l_input_cloud, *transformed, transform_2);

                try {
                    icp.align(*reg_result);
                } catch (...) {}

                fitness_score = icp.getFitnessScore();
                if (fitness_score < lowest_fitness_score && icp.hasConverged()) {
                    lowest_fitness_score = fitness_score;
                    best_guess = transform_2;
                    best_final = icp.getFinalTransformation();
                }
            }
        }
    }

    Eigen::Affine3f combined_transform = best_final * best_guess;

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> _r200_cloud_transformed;
    _r200_cloud_transformed.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::transformPointCloud(*r200_cloud, *_r200_cloud_transformed, combined_transform);

    pcl::toROSMsg(*_r200_cloud_transformed, res.combined_cloud);

    pcl::io::savePCDFileBinary("sr300_cloud_binary.pcd", *sr300_cloud);

    // */

    /*

    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> _r200_cloud_transformed;
    _r200_cloud_transformed.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Setting scale dependent NDT parameters
    // Setting minimum transformation difference for termination condition.
    ndt.setTransformationEpsilon(0.01);  // 0.001
    // Setting maximum step size for More-Thuente line search.
    ndt.setStepSize(0.1);  // 0.02
    // Setting Resolution of NDT grid structure (VoxelGridCovariance).
    ndt.setResolution(0.05);  // 1.0

    // Setting max number of registration iterations.
    ndt.setMaximumIterations(10);

    // Setting point cloud to be aligned.
    ndt.setInputSource(l_input_cloud);
    // Setting point cloud to be aligned to.
    ndt.setInputTarget(l_target_cloud);

    // Set initial alignment estimate found using robot odometry.
    Eigen::AngleAxisf init_rotation (0.0, Eigen::Vector3f::UnitZ ());
    Eigen::Translation3f init_translation (0.0, 0.0, 0.0);
    Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix ();

    // Calculating required rigid transform to align the input cloud to the target cloud.
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    ndt.align (*output_cloud, init_guess);

    std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged ()
            << " score: " << ndt.getFitnessScore () << std::endl;

    // Transforming unfiltered, input cloud using found transform.
    pcl::transformPointCloud (*r200_cloud, *_r200_cloud_transformed, ndt.getFinalTransformation());

    pcl::toROSMsg(*_r200_cloud_transformed, res.combined_cloud);

    pcl::io::savePCDFileBinary("sr300_cloud_binary.pcd", *sr300_cloud);

    */

    ROS_INFO_STREAM("Done");

    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "sr300_r200_combined_service");

    ros::NodeHandle n;

    // TODO service that asks for a point r200_cloud
    // Service should:
    //  * grab sr300 pc
    //  * turn off sr300 emitter
    //  * turn on r200 emitter
    //  * grab r200 pc
    //  * turn off r200 emitter
    //  * turn on sr300 emitter

    r200_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    sr300_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

    ros::ServiceServer service_handle = n.advertiseService("/get_combined_sr300_r200_cloud", get_combined_sr300_r200_cloud_service_callback);

    ros::Rate rate(30);

    while (n.ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
