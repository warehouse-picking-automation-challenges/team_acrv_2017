#ifndef PCL_FILTERS_H
#define PCL_FILTERS_H

//#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_representation.h>

#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/passthrough.h>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/common/pca.h>

#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d.h>

#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/segmentation/min_cut_segmentation.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

// #include <pcl/visualization/cloud_viewer.h>
// #include <pcl/visualization/pcl_visualizer.h>

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/features/boundary.h>

#include <pcl/common/common.h>
#include <pcl/console/time.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>

//convenient typedefs
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;


namespace pcl_filters {

    pcl::PointCloud<pcl::Boundary>::Ptr boundaryEstimation(pcl::PointCloud<pcl::PointNormal>::Ptr input,
         int k = 0, double radius = 0.0, double angle = M_PI/2.0);

    PointCloud::Ptr filter_cloud(PointCloud::Ptr cloud, int meanK = 50, float std = 1.0);

    PointCloud::Ptr RadiusOutlierRemoval(PointCloud::Ptr cloud, std::string method = "Radius",int kNeighbours = 2, float radius = 0.8);

    // void downSample(PointCloud::Ptr input_cloud, PointCloud::Ptr output_cloud, float leafSize = 0.005f);

    pcl::PointCloud<pcl::PointXYZ>::Ptr smooth_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float radius = 0.03);

    std::vector<PointCloud::Ptr> euclideanClusterRemoval(PointCloud::Ptr cloud, float ClusterTolerance = 0.01, int minClusterSize = 500, int MaxClusterSize = 25000);

    pcl::PointCloud<pcl::Normal>::Ptr compute_normals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float radius = 0.03);

    void compute_normals_down_sampled(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr downSampledCloud, pcl::PointCloud<pcl::Normal>::Ptr normalCloud,  Eigen::Vector3f view_point, float radius = 0.03);


    //PointCloudWithNormals::Ptr compute_normals(PointCloud::Ptr cloud, float radius = 0.03);

    PointCloud::Ptr color_based_region_growing(PointCloud::Ptr cloud);

    PointCloud::Ptr regionGrowingSegmentation(PointCloud::Ptr cloud);

    PointCloud::Ptr minCutBasedSegmentation(pcl::PointCloud <pcl::PointXYZ>::Ptr cloud, pcl::PointCloud <pcl::PointXYZ>::Ptr object, float sigma, float radius, int kNeighbours, float weight);

    pcl::PointCloud<pcl::PointXYZ>::Ptr StatisticalOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int meanK = 50, float std = 1.0);

    template<typename PointType>
    typename pcl::PointCloud<PointType>::Ptr downSample(typename pcl::PointCloud<PointType>::Ptr input_cloud, float leafSize = 0.005f){
        std::cout << "PointCloud before filtering has: " << input_cloud->points.size ()  << " data points." << std::endl; //*
        typename pcl::VoxelGrid<PointType> vg;
        typename pcl::PointCloud<PointType>::Ptr output_cloud = typename pcl::PointCloud<PointType>::Ptr(new typename pcl::PointCloud<PointType>());
        vg.setInputCloud (input_cloud);
        vg.setLeafSize (leafSize, leafSize, leafSize);
        vg.filter (*output_cloud);
        std::cout << "PointCloud after filtering has: " << output_cloud->points.size ()  << " data points." << std::endl; //*

        return output_cloud;
    }

}

namespace ros_pcl_tools{

    void readCloud(const sensor_msgs::PointCloud2::ConstPtr msg, PointCloud::Ptr cloud);

    void writeCloudtoMsg(PointCloud::Ptr cloud, sensor_msgs::PointCloud2::Ptr msg);

    void readImage(const sensor_msgs::Image::ConstPtr msgImage, cv::Mat &image);

}



#endif // PCL_FILTERS_H
