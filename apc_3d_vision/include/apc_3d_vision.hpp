/*
Copyright 2016 Australian Centre for Robotic Vision
*/

#ifndef APC_3D_VISION
#define APC_3D_VISION

#include <stdlib.h>     /* getenv */

#include <cv_bridge/cv_bridge.h>

#include <pcl_conversions/pcl_conversions.h>

// Point cloud library
#include <Eigen/Core>

#include <pcl/common/io.h>  // pcl::copyPointCloud
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/pca.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/segment_differences.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/surface/mls.h>

#include <boost/thread/thread.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <string>
#include <vector>
#include <iostream>
#include <map>

// Types
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::FPFHEstimationOMP<PointNT, PointNT, FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;


class Apc3dVision {
 public:
    class ndt_params_t {  // type name ndt_params_t
     public:
        ndt_params_t(
            boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> _input_cloud,
            boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> _target_cloud)
            : input_cloud(_input_cloud), target_cloud(_target_cloud) {}

        const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>>
            input_cloud;
        const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>>
            target_cloud;
        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> output_cloud;
        double input_cloud_leaf_size;
        bool is_downsample_input_cloud;
        double target_cloud_leaf_size;
        bool is_downsample_target_cloud;
        double transformationEpsilon;
        double stepSize;
        double resolution;
        int maxIterations;
        bool verbose;
        Eigen::Affine3f transform;
    };

    class icp_params_t {  // type name ndt_params_t
     public:
        icp_params_t(
            boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> _input_cloud,
            boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> _target_cloud)
            : input_cloud(_input_cloud), target_cloud(_target_cloud) {}

        const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>>
            input_cloud;
        const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>>
            target_cloud;
        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> output_cloud;
        double input_cloud_leaf_size;
        bool is_downsample_input_cloud;
        double target_cloud_leaf_size;
        bool is_downsample_target_cloud;
        Eigen::Affine3f transform;
        double lowest_fitness_score;
        bool verbose;
    };

    class pca_params_t {
     public:
        pca_params_t(
            const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>>
                _input_cloud) : input_cloud(_input_cloud) {}

        const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>>
            input_cloud;
        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> output_cloud;
        Eigen::Matrix<double, 3, 3> eigenvectors;
        Eigen::Vector4d centroid;
        bool verbose;
    };

    class align_params_t {
     public:
        align_params_t(
            boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> _input_cloud,
            boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> _target_cloud)
            : input_cloud(_input_cloud), target_cloud(_target_cloud) {}

        const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>>
            input_cloud;
        const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>>
            target_cloud;
        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> output_cloud;
        double input_cloud_leaf_size;
        bool is_downsample_input_cloud;
        double target_cloud_leaf_size;
        bool is_downsample_target_cloud;
        double transformationEpsilon;
        double stepSize;
        double resolution;
        int maxIterations;
        bool verbose;
    };

    struct align_prerejective_params_t {
        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> input_cloud;
        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> target_cloud;
        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> output_cloud;
        double leaf_size;
        double normalRadiusSearch;
        double featureRadiusSearch;
        int maxIterations;
        int numberOfSamples;
        int correspondenceRandomness;
        double similarityThreshold;
        double maxCorrespondenceDistance;
        double inlierFraction;
        bool verbose;
    };

    struct segment_differences_params_t {
        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> input_cloud;
        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> target_cloud;
        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> output_cloud;
        double distanceThreshold;
        bool verbose;
    };


    Apc3dVision();

    ~Apc3dVision();

    bool load_pcd_file(std::string fileName,
        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> output_cloud);

    bool save_pcd_file(std::string fileName,
        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> input_cloud);

    bool approximate_voxel_grid(
        const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>> input_cloud,
        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> output_cloud,
        double leaf_size, bool verbose);

    bool align_ndt(ndt_params_t *params);

    bool align_icp(icp_params_t *params);

    bool align_icp_tote(icp_params_t *params);

    bool align_pca(pca_params_t *params);

    bool align(align_params_t *params);

    // http://pointclouds.org/documentation/tutorials/alignment_prerejective.php#alignment-prerejective
    bool align_prerejective(align_prerejective_params_t params);

    bool segment_differences(segment_differences_params_t params);

    bool smooth_cloud(
        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud,
        boost::shared_ptr<pcl::PointCloud<pcl::PointNormal>> output_cloud,
        float radius);

    bool outlier_removal(
        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud,
        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> output_cloud,
        double outlierNumberOfSamples, double stdDevMulThresh);

    std::vector<int> get_labels_in_point_cloud(
        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZL>> input_cloud);

    bool split_labelled_point_cloud(
        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZL>> input_cloud,
        std::map<int, boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>>
        output_clouds);
};

#endif
