/*
Copyright 2016 Australian Centre for Robotic Vision
*/

#include <apc_3d_vision.hpp>

#include <math.h> /* sin */
#include <iostream>
#include <map>
#include <string>
#include <utility>
#include <vector>

Apc3dVision::Apc3dVision() {}

Apc3dVision::~Apc3dVision() {}

bool Apc3dVision::load_pcd_file(
    std::string fileName,
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> output_cloud) {
    // load the file
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(fileName, *output_cloud) == -1) {
        // ROS_ERROR_STREAM("Couldn't read file " << fileName << "\n");
        return false;
    } else {
        // ROS_INFO_STREAM("Loaded file " << fileName << "\n");
        return true;
    }
}

bool Apc3dVision::save_pcd_file(
    std::string fileName,
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> input_cloud) {
    // remove NAN points from the input_cloud
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*input_cloud, *input_cloud, indices);

    pcl::io::savePCDFileASCII(fileName, *input_cloud);
    std::cerr << "Saved " << input_cloud->points.size() << " data points to "
              << fileName << std::endl;

    return true;
}

bool Apc3dVision::approximate_voxel_grid(
    const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>> input_cloud,
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> output_cloud,
    double leaf_size, bool verbose) {
    if (input_cloud->size() == 0) {
        std::cout << "YOOOOOOOOOOO" << std::endl;
        return false;
    }

    // pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
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

bool Apc3dVision::align_ndt(Apc3dVision::ndt_params_t *params) {
    /*
    Downsample Input Cloud
    */
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> l_input_cloud;
    l_input_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

    if (params->is_downsample_input_cloud) {
        approximate_voxel_grid(params->input_cloud, l_input_cloud,
                               params->input_cloud_leaf_size, params->verbose);
    } else {
        // Copy the point cloud
        // l_input_cloud.reset(
        //     new pcl::PointCloud<pcl::PointXYZ>(*(params->input_cloud)));
        pcl::copyPointCloud(*(params->input_cloud), *l_input_cloud);
    }

    /*
    Downsample Target Cloud
    */
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> l_target_cloud;
    l_target_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

    if (params->is_downsample_target_cloud) {
        approximate_voxel_grid(params->target_cloud, l_target_cloud,
                               params->target_cloud_leaf_size, params->verbose);
    } else {
        // Copy the point cloud
        // l_target_cloud.reset(
        //     new pcl::PointCloud<pcl::PointXYZ>(*(params->target_cloud)));
        pcl::copyPointCloud(*(params->target_cloud), *l_target_cloud);
    }

    if (params->verbose) {
        // Visualisation
        pcl::visualization::PCLVisualizer visu1("NDT Initial Guess");
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
            input_colour(l_input_cloud, 0.0, 255.0, 0.0);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
            output_colour(l_target_cloud, 255.0, 0.0, 0.0);
        visu1.addPointCloud(l_input_cloud, input_colour, "input");
        visu1.addPointCloud(l_target_cloud, output_colour, "target");
        visu1.addCoordinateSystem(0.1);
        visu1.spin();
    }

    // Initializing Normal Distributions Transform (NDT).
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

    if (params->verbose) {
        std::cout << "transformationEpsilon = " << params->transformationEpsilon
                  << std::endl
                  << "stepSize = " << params->stepSize << std::endl
                  << "resolution = " << params->resolution << std::endl;
    }

    // Setting scale dependent NDT parameters
    // Setting minimum transformation difference for termination condition.
    ndt.setTransformationEpsilon(params->transformationEpsilon);  // 0.001
    // Setting maximum step size for More-Thuente line search.
    ndt.setStepSize(params->stepSize);  // 0.02
    // Setting Resolution of NDT grid structure (VoxelGridCovariance).
    ndt.setResolution(params->resolution);  // 1.0

    // Setting max number of registration iterations.
    ndt.setMaximumIterations(params->maxIterations);

    // Setting point cloud to be aligned.
    ndt.setInputSource(l_input_cloud);
    // Setting point cloud to be aligned to.
    ndt.setInputTarget(l_target_cloud);

    /*
    Calculating required rigid transform to align the input cloud to the target
    cloud.
    */
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> reg_result;
    reg_result.reset(new pcl::PointCloud<pcl::PointXYZ>);
    ndt.align(*reg_result);  // TODO(adamtow) Should we check return value?

    if (params->verbose) {
        std::cout << "Normal Distributions Transform has converged: "
                  << ndt.hasConverged()
                  << "\nFinal num iterations: " << ndt.getFinalNumIteration()
                  << "\nTransformation probability: "
                  << ndt.getTransformationProbability()
                  // << " score: " << ndt.getFitnessScore()  // Causes seg fault
                  << std::endl;
    }

    // Transforming unfiltered, input cloud using found transform.
    pcl::transformPointCloud(*(params->input_cloud), *(params->output_cloud),
                             ndt.getFinalTransformation());

    // Store transform in params
    params->transform = ndt.getFinalTransformation();

    if (params->verbose) {
        // Visualisation
        pcl::visualization::PCLVisualizer visu1("NDT Alignment");
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
            input_colour(params->target_cloud, 0.0, 255.0, 0.0);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
            output_colour(params->output_cloud, 255.0, 0.0, 0.0);
        visu1.addPointCloud(params->target_cloud, input_colour, "original");
        visu1.addPointCloud(params->output_cloud, output_colour, "aligned");
        visu1.addCoordinateSystem(0.1);
        visu1.spin();
    }

    // Temporarily view the downsampled final transform
    // pcl::transformPointCloud(
    //     *l_input_cloud,
    //     *(params->output_cloud),
    //     ndt.getFinalTransformation());
    //
    // if (params->verbose) {
    //     // Visualisation
    //     pcl::visualization::PCLVisualizer visu1("NDT Alignment");
    //     pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
    //         input_colour(l_target_cloud, 0.0, 255.0, 0.0);
    //     pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
    //         output_colour(params->output_cloud, 255.0, 0.0, 0.0);
    //     visu1.addPointCloud(l_target_cloud, input_colour, "original");
    //     visu1.addPointCloud(params->output_cloud, output_colour, "aligned");
    //     visu1.addCoordinateSystem(0.1);
    //     visu1.spin();
    // }

    if (ndt.hasConverged()) {
        return true;
    } else {
        return false;
    }
}

bool Apc3dVision::align_icp(Apc3dVision::icp_params_t *params) {
    /*
    Downsample Input Cloud
    */
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> l_input_cloud;
    l_input_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

    if (params->is_downsample_input_cloud) {
        approximate_voxel_grid(params->input_cloud, l_input_cloud,
                               params->input_cloud_leaf_size, params->verbose);
    } else {
        // Copy the point cloud
        // l_input_cloud.reset(
        //     new pcl::PointCloud<pcl::PointXYZ>(*(params->input_cloud)));
        pcl::copyPointCloud(*(params->input_cloud), *l_input_cloud);
    }

    /*
    Downsample Target Cloud
    */
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> l_target_cloud;
    l_target_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

    if (params->is_downsample_target_cloud) {
        approximate_voxel_grid(params->target_cloud, l_target_cloud,
                               params->target_cloud_leaf_size, params->verbose);
    } else {
        // Copy the point cloud
        // l_target_cloud.reset(
        //     new pcl::PointCloud<pcl::PointXYZ>(*(params->target_cloud)));
        pcl::copyPointCloud(*(params->target_cloud), *l_target_cloud);
    }

    if (params->verbose) {
        // Visualisation
        pcl::visualization::PCLVisualizer visu1("Before ICP");
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
            input_colour(l_input_cloud, 0.0, 255.0, 0.0);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
            output_colour(l_target_cloud, 255.0, 0.0, 0.0);
        visu1.addPointCloud(l_input_cloud, input_colour, "input");
        visu1.addPointCloud(l_target_cloud, output_colour, "target");
        visu1.addCoordinateSystem(0.1);
        visu1.spin();
    }

    // icp.setInputCloud(cloud_in);
    // icp.setInputTarget(cloud_out);
    // pcl::PointCloud<pcl::PointXYZ> Final;
    // icp.align(Final);
    // std::cout << "has converged:" << icp.hasConverged() << " score: " <<
    // icp.getFitnessScore() << std::endl;
    // std::cout << icp.getFinalTransformation() << std::endl;

    // Initializing Iterative Closest Point (ICP).
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

    // Setting point cloud to be aligned.
    // icp.setInputSource(l_input_cloud);
    // Setting point cloud to be aligned to.
    icp.setInputTarget(l_target_cloud);

    // Set the max correspondence distance to 5cm (e.g., correspondences with
    // higher distances will be ignored)
    // icp.setMaxCorrespondenceDistance(0.05);
    // Set the maximum number of iterations (criterion 1)
    // icp.setMaximumIterations(50);
    // Set the transformation epsilon (criterion 2)
    // icp.setTransformationEpsilon(1e-8);
    // Set the euclidean distance difference epsilon (criterion 3)
    // icp.setEuclideanFitnessEpsilon(1);

    /*
    Calculating required rigid transform to align the input cloud to the target
    cloud.
    */
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> reg_result;
    reg_result.reset(new pcl::PointCloud<pcl::PointXYZ>);
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> transformed;
    transformed.reset(new pcl::PointCloud<pcl::PointXYZ>);
    icp.setInputSource(transformed);
    // double theta = 0.0;
    double x_trans = 0.0;
    double y_trans = 0.0;
    double z_trans = 0.0;
    double fitness_score = 1.0;
    double lowest_fitness_score = 1.0;
    Eigen::Affine3f best_guess = Eigen::Affine3f::Identity();
    Eigen::Affine3f best_final = Eigen::Affine3f::Identity();
    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

    for (int i = 0; i <= 10; i++) {
        // Transform across the y,z plane diagonally in 10cm increments
        // Start from -0.5m,-0.5m and move to 0.5m,0.5m
        y_trans = (i - 5.0) / 10.0;
        z_trans = (i - 5.0) / 10.0;

        /*  METHOD #2: Using a Affine3f
        This method is easier and less error prone
        */

        // Define a translation of 2.5 meters on the x axis.
        transform_2.translation() << x_trans, y_trans, z_trans;

        // The same rotation matrix as before; theta radians arround X axis
        // theta = i/10.0;
        // transform_2.rotate(
        //     Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitX()));

        // Print the transformation
        // printf("\nMethod #2: using an Affine3f\n");
        // std::cout << transform_2.matrix() << std::endl;

        // Executing the transformation
        // Transforming the input cloud around is much faster than Transforming
        // the model around as it has less points
        pcl::transformPointCloud(*l_input_cloud, *transformed, transform_2);

        try {
            icp.align(*reg_result);
        } catch (...) {
        }

        fitness_score = icp.getFitnessScore();
        if (fitness_score < lowest_fitness_score && icp.hasConverged()) {
            lowest_fitness_score = fitness_score;
            best_guess = transform_2;
            best_final = icp.getFinalTransformation();
        }

        // if (params->verbose) {
        //     std::cout << "ICP converged?: "
        //         << icp.hasConverged()
        //         << " score: " << icp.getFitnessScore()  // Causes seg fault
        //         << std::endl;
        // }
    }

    // Store the lowest_fitness_score for winning candidate
    params->lowest_fitness_score = lowest_fitness_score;

    if (lowest_fitness_score != 1.0) {
        Eigen::Affine3f combined_transform = best_final * best_guess;
        params->transform = combined_transform;
        pcl::transformPointCloud(*l_input_cloud, *(params->output_cloud),
                                 combined_transform);

        // Temporarily view the downsampled final transform
        // pcl::transformPointCloud(
        //     *l_input_cloud,
        //     *l_input_cloud,
        //     best_guess);
        //
        // pcl::transformPointCloud(
        //     *l_input_cloud,
        //     *(params->output_cloud),
        //     best_final);

        if (params->verbose) {
            // Visualisation
            pcl::visualization::PCLVisualizer visu2("After ICP Downsampled");
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
                input_colour(l_target_cloud, 0.0, 255.0, 0.0);
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
                output_colour(params->output_cloud, 255.0, 0.0, 0.0);
            visu2.addPointCloud(l_target_cloud, input_colour, "original");
            visu2.addPointCloud(params->output_cloud, output_colour, "aligned");
            visu2.addCoordinateSystem(0.1);
            visu2.spin();
        }

        // Transforming unfiltered, input cloud using found transform.
        pcl::transformPointCloud(*(params->input_cloud),
                                 *(params->output_cloud), combined_transform);

        // pcl::transformPointCloud(
        //     *(params->input_cloud),
        //     *(params->output_cloud),
        //     best_guess);
        //
        // pcl::transformPointCloud(
        //     *(params->output_cloud),
        //     *(params->output_cloud),
        //     best_final);

        if (params->verbose) {
            // Visualisation
            pcl::visualization::PCLVisualizer visu1("After ICP");
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
                input_colour(params->target_cloud, 0.0, 255.0, 0.0);
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
                output_colour(params->output_cloud, 255.0, 0.0, 0.0);
            visu1.addPointCloud(params->target_cloud, input_colour, "original");
            visu1.addPointCloud(params->output_cloud, output_colour, "aligned");
            visu1.addCoordinateSystem(0.1);
            visu1.spin();
        }
        return true;
    } else {
        return false;
    }
    // if (icp.hasConverged()) {
    //     return true;
    // } else {
    //     return false;
    // }
}


bool Apc3dVision::align_icp_tote(Apc3dVision::icp_params_t *params) {
    /*
    Downsample Input Cloud
    */
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> l_input_cloud;
    l_input_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

    if (params->is_downsample_input_cloud) {
        approximate_voxel_grid(params->input_cloud, l_input_cloud,
                               params->input_cloud_leaf_size, params->verbose);
    } else {
        // Copy the point cloud
        // l_input_cloud.reset(
        //     new pcl::PointCloud<pcl::PointXYZ>(*(params->input_cloud)));
        pcl::copyPointCloud(*(params->input_cloud), *l_input_cloud);
    }

    /*
    Downsample Target Cloud
    */
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> l_target_cloud;
    l_target_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

    if (params->is_downsample_target_cloud) {
        approximate_voxel_grid(params->target_cloud, l_target_cloud,
                               params->target_cloud_leaf_size, params->verbose);
    } else {
        // Copy the point cloud
        // l_target_cloud.reset(
        //     new pcl::PointCloud<pcl::PointXYZ>(*(params->target_cloud)));
        pcl::copyPointCloud(*(params->target_cloud), *l_target_cloud);
    }

    if (params->verbose) {
        // Visualisation
        pcl::visualization::PCLVisualizer visu1("Before ICP");
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
            input_colour(l_input_cloud, 0.0, 255.0, 0.0);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
            output_colour(l_target_cloud, 255.0, 0.0, 0.0);
        visu1.addPointCloud(l_input_cloud, input_colour, "input");
        visu1.addPointCloud(l_target_cloud, output_colour, "target");
        visu1.addCoordinateSystem(0.1);
        visu1.spin();
    }

    // Initializing Iterative Closest Point (ICP).
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

    // Setting point cloud to be aligned.
    // icp.setInputSource(l_input_cloud);
    // Setting point cloud to be aligned to.
    icp.setInputTarget(l_target_cloud);

    /*
    Calculating required rigid transform to align the input cloud to the target
    cloud.
    */
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> reg_result;
    reg_result.reset(new pcl::PointCloud<pcl::PointXYZ>);
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> transformed;
    transformed.reset(new pcl::PointCloud<pcl::PointXYZ>);
    icp.setInputSource(transformed);
    // double theta = 0.0;
    double x_trans = 0.0;
    double y_trans = 0.0;
    double z_trans = 0.0;
    double fitness_score = 1.0;
    double lowest_fitness_score = 1.0;
    Eigen::Affine3f best_guess = Eigen::Affine3f::Identity();
    Eigen::Affine3f best_final = Eigen::Affine3f::Identity();
    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

    // for (int i = 0; i <= 10; i++) {
    //     // Transform across the y,z plane diagonally in 10cm increments
    //     // Start from -0.5m,-0.5m and move to 0.5m,0.5m
    //     y_trans = (i - 5.0) / 10.0;
    //     z_trans = (i - 5.0) / 10.0;
    //
    //     // Define a translation of 2.5 meters on the x axis.
    //     transform_2.translation() << x_trans, y_trans, z_trans;
    //
    //     // Executing the transformation
    //     // Transforming the input cloud around is much faster than Transforming
    //     // the model around as it has less points
    //     pcl::transformPointCloud(*l_input_cloud, *transformed, transform_2);
    //
    //     try {
    //         icp.align(*reg_result);
    //     } catch (...) {
    //     }
    //
    //     fitness_score = icp.getFitnessScore();
    //     if (fitness_score < lowest_fitness_score && icp.hasConverged()) {
    //         lowest_fitness_score = fitness_score;
    //         best_guess = transform_2;
    //         best_final = icp.getFinalTransformation();
    //     }
    // }


    transform_2.translation() << 0, 0, 0;

    // Executing the transformation
    // Transforming the input cloud around is much faster than Transforming
    // the model around as it has less points
    pcl::transformPointCloud(*l_input_cloud, *transformed, transform_2);

    try {
        icp.align(*reg_result);
    } catch (...) {
    }

    fitness_score = icp.getFitnessScore();
    if (fitness_score < lowest_fitness_score && icp.hasConverged()) {
        lowest_fitness_score = fitness_score;
        best_guess = transform_2;
        best_final = icp.getFinalTransformation();
    }

    // Store the lowest_fitness_score for winning candidate
    params->lowest_fitness_score = lowest_fitness_score;

    if (lowest_fitness_score != 1.0) {
        Eigen::Affine3f combined_transform = best_final * best_guess;
        params->transform = combined_transform;
        pcl::transformPointCloud(*l_input_cloud, *(params->output_cloud),
                                 combined_transform);

        if (params->verbose) {
            // Visualisation
            pcl::visualization::PCLVisualizer visu2("After ICP Downsampled");
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
                input_colour(l_target_cloud, 0.0, 255.0, 0.0);
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
                output_colour(params->output_cloud, 255.0, 0.0, 0.0);
            visu2.addPointCloud(l_target_cloud, input_colour, "original");
            visu2.addPointCloud(params->output_cloud, output_colour, "aligned");
            visu2.addCoordinateSystem(0.1);
            visu2.spin();
        }

        // Transforming unfiltered, input cloud using found transform.
        pcl::transformPointCloud(*(params->input_cloud),
                                 *(params->output_cloud), combined_transform);

        if (params->verbose) {
            // Visualisation
            pcl::visualization::PCLVisualizer visu1("After ICP");
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
                input_colour(params->target_cloud, 0.0, 255.0, 0.0);
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
                output_colour(params->output_cloud, 255.0, 0.0, 0.0);
            visu1.addPointCloud(params->target_cloud, input_colour, "original");
            visu1.addPointCloud(params->output_cloud, output_colour, "aligned");
            visu1.addCoordinateSystem(0.1);
            visu1.spin();
        }
        return true;
    } else {
        return false;
    }
}

bool Apc3dVision::align_pca(Apc3dVision::pca_params_t *params) {
    // Declare local temporary variables
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> local_cloud;
    local_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

    // Calculate the centroid
    Eigen::Vector4d centroid;
    pcl::compute3DCentroid(*(params->input_cloud), centroid);
    params->centroid = centroid;

    // Subtract the centroid from the point cloud
    pcl::demeanPointCloud<pcl::PointXYZ>(*(params->input_cloud), centroid,
                                         *local_cloud);

    /// Compute PCA for the input cloud
    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud(local_cloud);
    // Eigen::Vector3f eigenvalues_float = pca.getEigenValues();
    // Eigen::Vector3d eigenvalues = eigenvalues_float.cast<double>();
    Eigen::Matrix3f eigenvectors_float = pca.getEigenVectors();
    Eigen::Matrix<double, 3, 3> eigenvectors =
        eigenvectors_float.cast<double>();

    // Ensure z-axis direction satisfies right-hand rule
    eigenvectors.col(2) = eigenvectors.col(0).cross(eigenvectors.col(1));

    // PCL_INFO("eigenvalues: %f %f %f\n", eigenvalues[0]/eigenvalues[0],
    //     eigenvalues[1] / eigenvalues[0], eigenvalues[2]/eigenvalues[0]);

    // Normalising these modifies the scaling of my transformed object
    // eigenvalues = eigenvalues.normalized();
    // eigenvectors = eigenvectors.normalized();

    // Eigen::Vector3d axis_0 = eigenvectors.col(0);
    // Eigen::Vector3d axis_1 = eigenvectors.col(1);
    // Eigen::Vector3d axis_2 = eigenvectors.col(2);
    //
    // pcl::PointXYZ axis_0_start;
    // pcl::PointXYZ axis_0_end;
    //
    // axis_0_start.x = centroid[0];
    // axis_0_start.y = centroid[1];
    // axis_0_start.z = centroid[2];
    // axis_0_end.x = centroid[0] + axis_0[0];
    // axis_0_end.y = centroid[1] + axis_0[1];
    // axis_0_end.z = centroid[2] + axis_0[2];

    // Align the principal axes to the objects frame
    Eigen::Matrix<double, 4, 4> transformation(
        Eigen::Matrix<double, 4, 4>::Identity());
    transformation.template block<3, 3>(0, 0) = eigenvectors.transpose();
    // transformation.template block<3, 1>(0, 3) =
    //     -1.f * (transformation.template block<3,3>(0,0) *
    //     centroid.template block<3,1>(0,0));

    params->eigenvectors = eigenvectors;

    // (params->output_cloud).reset(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*local_cloud, *(params->output_cloud),
                             transformation);

    if (params->verbose) {
        // Visualisation
        pcl::visualization::PCLVisualizer visu1("PCA Alignment");
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
            input_colour(local_cloud, 0.0, 255.0, 0.0);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
            output_colour(params->output_cloud, 255.0, 0.0, 0.0);
        visu1.addPointCloud(local_cloud, input_colour, "original");
        visu1.addPointCloud(params->output_cloud, output_colour, "aligned");
        // visu1.addLine<pcl::PointXYZ>(
        //     axis_0_start, axis_0_end, "original_principal_axis_0");
        visu1.addCoordinateSystem(0.1);
        visu1.spin();
    }

    return true;
}

bool Apc3dVision::align(Apc3dVision::align_params_t *params) {
    Apc3dVision::pca_params_t input_pca_params(params->input_cloud);
    input_pca_params.output_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    input_pca_params.verbose = false;
    Apc3dVision::pca_params_t target_pca_params(params->target_cloud);
    target_pca_params.output_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    target_pca_params.verbose = false;

    align_pca(&input_pca_params);
    align_pca(&target_pca_params);

    // std::cout << input_pca_params.output_cloud->size() << std::endl;

    ndt_params_t ndt_params(input_pca_params.output_cloud,
                            target_pca_params.output_cloud);

    ndt_params.output_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    ndt_params.input_cloud_leaf_size = params->input_cloud_leaf_size;
    ndt_params.is_downsample_input_cloud = params->is_downsample_input_cloud;
    ndt_params.target_cloud_leaf_size = params->target_cloud_leaf_size;
    ndt_params.is_downsample_target_cloud = params->is_downsample_target_cloud;
    ndt_params.transformationEpsilon = params->transformationEpsilon;
    ndt_params.stepSize = params->stepSize;
    ndt_params.resolution = params->resolution;
    ndt_params.maxIterations = params->maxIterations;
    ndt_params.verbose = params->verbose;

    bool isShelfRegistered = align_ndt(&ndt_params);

    params->output_cloud = ndt_params.output_cloud;

    return isShelfRegistered;
}

// Align a rigid object to a scene with clutter and occlusions

bool Apc3dVision::align_prerejective(
    Apc3dVision::align_prerejective_params_t params) {
    // Eigen::Vector4f raw_original_centroid;
    // Eigen::Vector4f raw_final_centroid;
    //
    // pcl::compute3DCentroid(
    //     *(params.target_cloud), raw_original_centroid);
    //
    // // Visualisation
    // // pcl::visualization::PCLVisualizer visu1("Before Alignment");
    // // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
    // //     input_colour(params.input_cloud, 0.0, 255.0, 0.0);
    // // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
    // //     target_colour(params.target_cloud, 255.0, 0.0, 0.0);
    // // visu1.addPointCloud(params.input_cloud, input_colour, "input");
    // // visu1.addPointCloud(params.target_cloud, target_colour, "target");
    // // visu1.addCoordinateSystem(0.1);
    // // visu1.spin();
    //
    // Apc3dVision::pca_params_t input_pca_params;
    // Apc3dVision::pca_params_t target_pca_params;
    //
    // input_pca_params.input_cloud = params.input_cloud;
    // // input_pca_params.output_cloud.reset(new
    // pcl::PointCloud<pcl::PointXYZ>);
    // align_pca(&input_pca_params);
    // target_pca_params.input_cloud = params.target_cloud;
    // // target_pca_params.output_cloud.reset(new
    // pcl::PointCloud<pcl::PointXYZ>);
    // target_pca_params.verbose = true;
    // align_pca(&target_pca_params);
    //
    // // --------Do this step after all alignment
    // steps---------------------------
    // // Transform both point clouds back into raw point cloud's frame
    // // Eigen::Matrix<double, 4, 4>
    // //     transformation(Eigen::Matrix<double, 4, 4>::Identity());
    // // transformation.template block<3, 3>(0, 0) =
    // target_pca_params.eigenvectors;
    // // transformation.template block<3, 1>(0, 3) =
    // //     target_pca_params.centroid.template block<3, 1>(0, 0);
    // // pcl::transformPointCloud(
    // //     *(input_pca_params.output_cloud),
    // //     *(input_pca_params.output_cloud),
    // //     transformation);
    // // pcl::transformPointCloud(
    // //     *(target_pca_params.output_cloud),
    // //     *(target_pca_params.output_cloud),
    // //     transformation);
    // // --------Do this step after all alignment
    // steps---------------------------
    //
    // pcl::compute3DCentroid(
    //     *(target_pca_params.output_cloud), raw_final_centroid);
    // std::cout << "Original Centroid: " << raw_original_centroid << std::endl
    //     << "Final Centroid: " << raw_final_centroid << std::endl;
    //
    // // Visualisation
    // // pcl::visualization::PCLVisualizer visu2("After Alignment");
    // // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
    // //     input_colour2(input_pca_params.output_cloud, 0.0, 255.0, 0.0);
    // // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
    // //     target_colour2(target_pca_params.output_cloud, 255.0, 0.0, 0.0);
    // // visu2.addPointCloud(
    // //     input_pca_params.output_cloud, input_colour2, "input");
    // // visu2.addPointCloud(
    // //     target_pca_params.output_cloud, target_colour2, "target");
    // // visu2.addCoordinateSystem(0.1);
    // // visu2.spin();
    //
    // // params.input_cloud = params.output_cloud;
    //
    // // Point clouds
    // PointCloudT::Ptr object (new PointCloudT);
    // PointCloudT::Ptr object_aligned (new PointCloudT);
    // PointCloudT::Ptr scene (new PointCloudT);
    // FeatureCloudT::Ptr object_features (new FeatureCloudT);
    // FeatureCloudT::Ptr scene_features (new FeatureCloudT);
    //
    // pcl::copyPointCloud(*(input_pca_params.output_cloud), *object);  // model
    // pcl::copyPointCloud(*(target_pca_params.output_cloud), *scene);  // raw
    //
    // // -----------------------
    //
    // // double k = 0.2;
    // //
    // // /// Compute PCA for the input cloud
    // // pcl::PCA<pcl::PointXYZ> pca;
    // // pca.setInputCloud(params.input_cloud);
    // // Eigen::Vector3f eigenvalues = pca.getEigenValues();
    // // Eigen::Matrix3f eigenvectors = pca.getEigenVectors();
    // //
    // // PCL_INFO("eigenvalues: %f %f %f\n", eigenvalues[0]/eigenvalues[0],
    // //     eigenvalues[1] / eigenvalues[0], eigenvalues[2]/eigenvalues[0]);
    // // Eigen::Vector3d main_pca_axis = eigenvectors.col(0).cast<double>();
    // // Eigen::Vector3d input_principal_axis =
    // eigenvectors.col(0).cast<double>();
    // // Eigen::Vector3d axis_1 = eigenvectors.col(1).cast<double>();
    // // Eigen::Vector3d axis_2 = eigenvectors.col(2).cast<double>();
    // // std::cout << "Input Cloud main axis" << main_pca_axis << std::endl;
    // //
    // // /// Compute the centroid
    // // Eigen::Vector4d centroid;
    // // pcl::compute3DCentroid(*params.input_cloud, centroid);
    // //
    // // // Eigen::Vector3d pt1;
    // // // Eigen::Vector3d pt2;
    // // pcl::PointXYZ input_pt1;
    // // pcl::PointXYZ input_pt2;
    // //
    // //
    // // input_pt1.x = centroid[0];
    // // input_pt1.y = centroid[1];
    // // input_pt1.z = centroid[2];
    // // input_pt2.x = centroid[0] + k * main_pca_axis[0];
    // // input_pt2.y = centroid[1] + k * main_pca_axis[1];
    // // input_pt2.z = centroid[2] + k * main_pca_axis[2];
    //
    //
    // // ----------------------------
    //
    // /// Compute PCA for the target cloud
    // // pca.setInputCloud(params.target_cloud);
    // // eigenvalues = pca.getEigenValues();
    // // eigenvectors = pca.getEigenVectors();
    // //
    // // PCL_INFO("eigenvalues: %f %f %f\n", eigenvalues[0]/eigenvalues[0],
    // //     eigenvalues[1] / eigenvalues[0], eigenvalues[2]/eigenvalues[0]);
    // // main_pca_axis = eigenvectors.col(0).cast<double>();
    // // Eigen::Vector3d target_principal_axis =
    // eigenvectors.col(0).cast<double>();
    // // std::cout << "Target Cloud main axis" << main_pca_axis << std::endl;
    // //
    // // /// Compute the centroid
    // // pcl::compute3DCentroid(*params.target_cloud, centroid);
    // //
    // // // Eigen::Vector3d pt1;
    // // // Eigen::Vector3d pt2;
    // // pcl::PointXYZ target_pt1;
    // // pcl::PointXYZ target_pt2;
    // //
    // // target_pt1.x = centroid[0];
    // // target_pt1.y = centroid[1];
    // // target_pt1.z = centroid[2];
    // // target_pt2.x = centroid[0] + k * main_pca_axis[0];
    // // target_pt2.y = centroid[1] + k * main_pca_axis[1];
    // // target_pt2.z = centroid[2] + k * main_pca_axis[2];
    // //
    // //
    // // input_principal_axis = input_principal_axis.normalized();
    // // target_principal_axis = target_principal_axis.normalized();
    // // Eigen::Vector3d axis_of_rotation =
    // //     input_principal_axis.cross(target_principal_axis);
    // // double angle_of_rotation =
    // //     acos(input_principal_axis.dot(target_principal_axis));
    // // std::cout << "Angle of Rotation: "
    // //      << angle_of_rotation * 180.0 / 3.14159265 << std::endl;
    // // std::cout << "Axis of Rotation: " << axis_of_rotation << std::endl;
    // //
    // // Eigen::AngleAxisd angle_axis(angle_of_rotation, axis_of_rotation);
    // //
    // // std::cout << "Angle Axis: " << angle_axis << std::endl;
    //
    // // Eigen::Transform t;
    // // t = angle_axis;
    //
    // // Eigen::Affine3d t(angle_axis);
    //
    // // std::cout << "Transform: " << t << std::endl;
    //
    // // Eigen::Matrix<MatScalar, 4, 4> transformation;
    // // transformation = Eigen::Matrix<MatScalar, 4, 4>::Identity();
    // // Eigen::Matrix<double, 4, 4> transformation(Eigen::Matrix<double, 4,
    // 4>::Identity());
    // // transformation.template block<3, 3>(0, 0) =
    // //     eigenvectors.transpose().cast<double>();
    // // // transformation.template block<3, 1>(0, 3) = -1.f *
    // (transformation.template block<3,3>(0,0) * centroid.template
    // block<3,1>(0,0));
    // //
    // // pcl::transformPointCloud(
    // //     *(params.input_cloud), *(params.input_cloud), transformation);
    // // pcl::copyPointCloud(*(params.input_cloud), *object);
    //
    // /// The plane equation
    // // double d = (-1) * (centroid[0] * main_pca_axis[0] +
    // //     centroid[1] * main_pca_axis[1] +
    // //     centroid[2] * main_pca_axis[2]);
    //
    //
    // // pcl::transformPointCloud(
    // //     *(params.input_cloud),
    // //     *(params.output_cloud),
    // //     ndt.getFinalTransformation());
    //
    // // -----------------------
    //
    // // Downsample
    // pcl::console::print_highlight("Downsampling...\n");
    // pcl::VoxelGrid<PointNT> grid;
    // // const float leaf = 0.005f;
    // grid.setLeafSize(params.leaf_size, params.leaf_size, params.leaf_size);
    // grid.setInputCloud(object);
    // grid.filter(*object);
    // grid.setInputCloud(scene);
    // grid.filter(*scene);
    // std::cout << "Downsampled cloud contains " <<
    //     scene->size() << std::endl;
    //
    // // Estimate normals for scene
    // pcl::console::print_highlight("Estimating scene normals...\n");
    // pcl::NormalEstimationOMP<PointNT, PointNT> nest;
    // nest.setRadiusSearch(params.normalRadiusSearch);
    // nest.setInputCloud(scene);
    // nest.compute(*scene);
    //
    // // Estimate features
    // pcl::console::print_highlight("Estimating features...\n");
    // FeatureEstimationT fest;
    // fest.setRadiusSearch(params.featureRadiusSearch);
    // fest.setInputCloud(object);
    // fest.setInputNormals(object);
    // fest.compute(*object_features);
    // fest.setInputCloud(scene);
    // fest.setInputNormals(scene);
    // fest.compute(*scene_features);
    //
    // // Perform alignment
    // pcl::console::print_highlight("Starting alignment...\n");
    // pcl::SampleConsensusPrerejective<PointNT, PointNT, FeatureT> align;
    // align.setInputSource(object);
    // std::cout << "Input cloud contains " <<
    //     object->size() << " points." << std::endl;
    // align.setSourceFeatures(object_features);
    // align.setInputTarget(scene);
    // std::cout << "Target cloud contains " <<
    //     scene->size() << " points." << std::endl;
    // align.setTargetFeatures(scene_features);
    // // Number of RANSAC iterations
    // align.setMaximumIterations(params.maxIterations);
    // // Number of points to sample for generating/prerejecting a pose
    // align.setNumberOfSamples(params.numberOfSamples);
    // // Number of nearest features to use
    // align.setCorrespondenceRandomness(params.correspondenceRandomness);
    // // Polygonal edge length similarity threshold
    // align.setSimilarityThreshold(params.similarityThreshold);
    // // Inlier threshold
    // align.setMaxCorrespondenceDistance(params.maxCorrespondenceDistance);
    // // Required inlier fraction for accepting a pose hypothesis
    // align.setInlierFraction(params.inlierFraction);
    // {
    //     pcl::ScopeTime t("Alignment");
    //     align.align(*object_aligned);
    //     pcl::copyPointCloud(*object_aligned, *(params.output_cloud));
    // }
    //
    // if (align.hasConverged()) {
    //     // Print results
    //     printf("\n");
    //     Eigen::Matrix4f transformation = align.getFinalTransformation();
    //     pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n",
    //         transformation(0, 0),
    //         transformation(0, 1),
    //         transformation(0, 2));
    //     pcl::console::print_info("R = | %6.3f %6.3f %6.3f | \n",
    //         transformation(1, 0),
    //         transformation(1, 1),
    //         transformation(1, 2));
    //     pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n",
    //         transformation(2, 0),
    //         transformation(2, 1),
    //         transformation(2, 2));
    //     pcl::console::print_info("\n");
    //     pcl::console::print_info("t = < %0.3f, %0.3f, %0.3f >\n",
    //         transformation(0, 3),
    //         transformation(1, 3),
    //         transformation(2, 3));
    //     pcl::console::print_info("\n");
    //     pcl::console::print_info("Inliers: %i/%i\n",
    //         align.getInliers().size(),
    //         object->size());
    //
    //     // Show alignment
    //     pcl::visualization::PCLVisualizer visu2("Alignment");
    //     visu2.addPointCloud(scene,
    //         ColorHandlerT(scene, 0.0, 255.0, 0.0), "scene");
    //     visu2.addPointCloud(object_aligned,
    //         ColorHandlerT(object_aligned, 255.0, 0.0, 0.0),
    //         "object_aligned");
    //     visu2.addCoordinateSystem(0.1);
    //     visu2.spin();
    //
    //     // Show original guess
    //     pcl::visualization::PCLVisualizer visu1("Starting Guess");
    //     visu1.addPointCloud(scene,
    //         ColorHandlerT(scene, 0.0, 255.0, 0.0), "scene");
    //     visu1.addPointCloud(object,
    //         ColorHandlerT(object, 255.0, 0.0, 0.0), "object");
    //     visu1.addCoordinateSystem(0.1);
    //     visu1.spin();
    // } else {
    //     pcl::console::print_error("Alignment failed!\n");
    //     return(true);
    // }
    return false;
}

bool Apc3dVision::segment_differences(
    Apc3dVision::segment_differences_params_t params) {
    // Segment Differences

    /* Creating the KdTree object for the search method of the
       extraction
    */
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
        new pcl::search::KdTree<pcl::PointXYZ>);
    // tree->setInputCloud (cloud_filtered);

    pcl::SegmentDifferences<pcl::PointXYZ> segmenter;

    segmenter.setInputCloud(params.input_cloud);    // kinect cloud
    segmenter.setTargetCloud(params.target_cloud);  // registered shelf cloud

    /*
    Threshold the distance threshold (tolerance) for point
    correspondences. (e.g., check if f a point p1 from src has a
    correspondence > threshold than a point p2 from tgt)
    */

    if (params.verbose) {
        std::cout << "distanceThreshold = " << params.distanceThreshold
                  << std::endl;
    }
    segmenter.setDistanceThreshold(params.distanceThreshold);
    /*
    Tree the spatial locator (e.g., kd-tree) used for nearest
    neighbors searching built over target cloud
    */
    segmenter.setSearchMethod(tree);
    if (params.verbose) {
        std::cout << "Segmenting the point cloud..." << std::endl;
    }
    segmenter.segment(*(params.output_cloud));
    if (params.verbose) {
        std::cout << "Point cloud segmented." << std::endl;
    }

    return true;
}

bool Apc3dVision::outlier_removal(
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

bool Apc3dVision::smooth_cloud(
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> input_cloud,
    boost::shared_ptr<pcl::PointCloud<pcl::PointNormal>> output_cloud,
    float radius) {
    // Create a KD-Tree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
        new pcl::search::KdTree<pcl::PointXYZ>);

    /*
    Output has the PointNormal type in order to
        store the normals calculated by MLS
    */
    // pcl::PointCloud<pcl::PointNormal> mls_points;

    // Init object (second point type is for the normals, even if unused)
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

    // If normal estimation is not required, this can be skipped
    // mls.setComputeNormals(true);

    // Set parameters
    mls.setInputCloud(input_cloud);
    mls.setPolynomialFit(true);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(radius);

    // Reconstruct
    mls.process(*output_cloud);

    return true;
}

std::vector<int> Apc3dVision::get_labels_in_point_cloud(
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZL>> input_cloud) {
    //
    std::cout << "YO1" << std::endl;
    std::vector<int> point_cloud_labels;
    int label = 0;
    std::map<int, int> temp_map;
    std::map<int, int>::iterator temp_map_itr;
    pcl::PointCloud<pcl::PointXYZL>::iterator seg_itr = (*input_cloud).begin();
    for (; seg_itr != (*input_cloud).end(); ++seg_itr) {
        label = seg_itr->label;

        temp_map_itr = temp_map.find(label);
        if (temp_map_itr != temp_map.end()) {
            // Already in vector
        } else {
            temp_map.insert(std::pair<int, int>(label, label));
            point_cloud_labels.push_back(label);
        }
    }
    std::cout << "YO2" << std::endl;
    return point_cloud_labels;
}

bool Apc3dVision::split_labelled_point_cloud(
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZL>> input_cloud,
    std::map<int, boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>>
        output_clouds) {
    int numberOfObjects = 0;
    int label = 0;
    std::map<int, boost::shared_ptr<pcl::PointCloud<pcl::PointXYZL>>>
        object_map_l;

    std::map<int, boost::shared_ptr<pcl::PointCloud<pcl::PointXYZL>>>::iterator
        object_itr;

    pcl::PointCloud<pcl::PointXYZL>::iterator seg_itr = (*input_cloud).begin();

    for (; seg_itr != (*input_cloud).end(); ++seg_itr) {
        label = seg_itr->label;
        // std::cout << "Label = " << point_itr->label << std::endl;

        object_itr = object_map_l.find(label);
        if (object_itr != object_map_l.end()) {
            // Add more points to the point cloud
            object_map_l[label]->push_back(*seg_itr);
        } else {
            // Create a new point cloud
            numberOfObjects++;
            object_map_l[label].reset(new pcl::PointCloud<pcl::PointXYZL>);
            object_map_l[label]->push_back(*seg_itr);
        }
    }

    std::pair<int, boost::shared_ptr<pcl::PointCloud<pcl::PointXYZL>>> o_pair;

    BOOST_FOREACH (o_pair, object_map_l) {
        // Must create the memory in the calling function otherwise it gets
        // deleted.
        // output_clouds[o_pair.first].reset(
        //     new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*o_pair.second, *(output_clouds[o_pair.first]));
        // std::cout << "YOOOOOOOOOOO" << std::endl;
        // std::stringstream fileName;
        // fileName << "/home/baxter/co/apc_ws/temp/" << o_pair.first << ".pcd";
        // apc_vis.save_pcd_file(fileName.str(), output_clouds[o_pair.first]);
    }

    std::cout << "Is output_clouds empty? " << output_clouds.empty()
              << std::endl;
    std::cout << "There are " << numberOfObjects
              << " objects in split_labelled_point_cloud function in "
                 "apc_3d_vision.\n";

    return true;
}
