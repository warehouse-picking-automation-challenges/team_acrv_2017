#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf2_ros/transform_broadcaster.h>
#include <string>

#include <pcl/common/geometry.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/common/pca.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <pcl/point_representation.h>

ros::NodeHandle *nh_;
tf::TransformListener *tf_listener;

bool fit_cad_model(apc_msgs::ModelFit::Request &req,
                   apc_msgs::ModelFit::Response &res) {

      // RROS_INFO("[CARTESIAN GRASP] Starting model fit service call request");
      //
      // // ICP Params
      // double leaf = 0.005;
      //
      // // Smoothing Params
      // // radius in cm
      // double smoothingRadius = 0.03;
      //
      // // Statistical outlier removal params
      // // number of nearest points to use for std_dev calculation
      // double outlierNumberOfSamples = 10;
      // // any points passed `n` std devs of query point are removed
      // double stdDevMulThresh = 2;
      //
      // uint numberOfObjects = req.labelled_points_array.size();
      // uint numberOfCandidateModels = req.candidate_cad_model_paths_array.size();
      //
      // boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> rawPC;
      // rawPC.reset(new pcl::PointCloud<pcl::PointXYZ>);
      // boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> modelPC;
      // modelPC.reset(new pcl::PointCloud<pcl::PointXYZ>);
      //
      // double lowest_fitness_score = 1.0;
      // std::vector<std::string>
      //     best_fit_cad_model_file_name_vector(numberOfObjects);
      // std::vector<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>>
      //     best_fit_cad_model_vector(numberOfObjects);
      // std::vector<Eigen::Affine3f>
      //     best_fit_cad_model_pose_vector(numberOfObjects);
      //
      // for (uint i = 0; i < numberOfObjects; i++) {
      //     pcl::fromROSMsg(req.labelled_points_array[i], *rawPC);
      //
      //     // Fill every transform with an identity matrix
      //     best_fit_cad_model_pose_vector[i] = Eigen::Affine3f::Identity();
      //
      //     // Remove statistical outliers from the raw point cloud
      //     apc_vis.outlier_removal(
      //         rawPC, rawPC, outlierNumberOfSamples, stdDevMulThresh);
      //
      //     // Smooth the raw point cloud
      //     boost::shared_ptr<pcl::PointCloud<pcl::PointNormal>> rawPCNormal;
      //     rawPCNormal.reset(new pcl::PointCloud<pcl::PointNormal>);
      //     apc_vis.smooth_cloud(rawPC, rawPCNormal, smoothingRadius);
      //     pcl::copyPointCloud(*rawPCNormal, *rawPC);
      //
      //     // Perform PCA on the raw point cloud
      //     Apc3dVision::pca_params_t input_pca_params(rawPC);
      //     input_pca_params.output_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
      //     input_pca_params.verbose = false;
      //     apc_vis.align_pca(&input_pca_params);
      //
      //     for (uint j = 0; j < numberOfCandidateModels; j++) {
      //         ROS_INFO(
      //             "Trying to match candidate model #%d (of %d) to Object #%d ...",
      //             j+1, numberOfCandidateModels, i+1);
      //
      //         // Load the next cad model
      //         std::string modelFileName =
      //             req.candidate_cad_model_paths_array[j].data;
      //         apc_vis.load_pcd_file(modelFileName, modelPC);
      //
      //         // Perform PCA on the cad model point cloud
      //         Apc3dVision::pca_params_t target_pca_params(modelPC);
      //         target_pca_params.output_cloud.reset(
      //             new pcl::PointCloud<pcl::PointXYZ>);
      //         target_pca_params.verbose = false;
      //         apc_vis.align_pca(&target_pca_params);
      //
      //         // Perform ICP
      //         Apc3dVision::icp_params_t icp_params(
      //             input_pca_params.output_cloud, target_pca_params.output_cloud);
      //
      //         icp_params.output_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
      //         icp_params.input_cloud_leaf_size = leaf;
      //         icp_params.is_downsample_input_cloud = true;
      //         icp_params.target_cloud_leaf_size = leaf;
      //         icp_params.is_downsample_target_cloud = true;
      //         icp_params.verbose = false;
      //
      //         if (apc_vis.align_icp(&icp_params)) {
      //             // Transform both point clouds back into raw point cloud's frame
      //             // First undo transform from ICP
      //             // Do the inverse of the found transform on the target object
      //             // pcl::transformPointCloud(
      //             //     *(target_pca_params.output_cloud),
      //             //     *(target_pca_params.output_cloud),
      //             //     icp_params.transform.inverse());
      //
      //             // Then undo transform from PCA
      //             // PCA transformed both input and target so both must be undone
      //             Eigen::Matrix<double, 4, 4>
      //                 transformation(Eigen::Matrix<double, 4, 4>::Identity());
      //             transformation.template block<3, 3>(0, 0) =
      //                 input_pca_params.eigenvectors;
      //             // Set both models to the centroid of the input (raw cloud)
      //             transformation.template block<3, 1>(0, 3) =
      //                 input_pca_params.centroid.template block<3, 1>(0, 0);
      //             pcl::transformPointCloud(
      //                 *(input_pca_params.output_cloud),
      //                 *(input_pca_params.output_cloud),
      //                 transformation);
      //             // pcl::transformPointCloud(
      //             //     *(target_pca_params.output_cloud),
      //             //     *(target_pca_params.output_cloud),
      //             //     transformation);
      //             Eigen::Matrix<float, 4, 4> h = transformation.cast<float>();
      //             Eigen::Affine3f undoPCATransformation(h);
      //             // Eigen::Affine3f undoPCARotation(input_pca_params.eigenvectors);
      //             // Eigen::Matrix<float, 3, 3>(input_pca_params.eigenvectors));
      //             // Eigen::Affine3f undoPCATranslation(
      //                 // Eigen::Matrix<float, 3, 1>(input_pca_params.centroid));
      //
      //             Eigen::Affine3f undoICPTransform(
      //                 icp_params.transform.inverse());
      //
      //             Eigen::Affine3f finalCADTransform =
      //                 undoPCATransformation * undoICPTransform;
      //                 // undoPCARotation * undoPCATranslation * undoICPTransform;
      //
      //             pcl::transformPointCloud(
      //                 *(target_pca_params.output_cloud),
      //                 *(target_pca_params.output_cloud),
      //                 finalCADTransform);
      //
      //             if (icp_params.lowest_fitness_score < lowest_fitness_score) {
      //                 // a better fit was found and will be copied over
      //                 best_fit_cad_model_file_name_vector[i] = modelFileName;
      //                 best_fit_cad_model_vector[i].reset(
      //                     new pcl::PointCloud<pcl::PointXYZ>);
      //                 pcl::copyPointCloud(*(target_pca_params.output_cloud),
      //                     *(best_fit_cad_model_vector[i]));
      //
      //                 best_fit_cad_model_pose_vector[i] = finalCADTransform;
      //             }
      //         } else {
      //             ROS_INFO("align_icp returned false");
      //         }
      //     }
      // }
      //
      //
      // ROS_INFO("Filling the ROS message...");
      // // Return the best fit cad models to the client
      // // for (uint i = 0; i < numberOfObjects; i++) {
      // assert(best_fit_cad_model_vector.size() == numberOfObjects);
      // for (uint i = 0; i < best_fit_cad_model_vector.size(); i++) {
      //     sensor_msgs::PointCloud2 temp_message;
      //     if (best_fit_cad_model_vector[i]) {
      //         pcl::toROSMsg(*(best_fit_cad_model_vector[i]), temp_message);
      //     }
      //     res.cad_model_points_array.push_back(temp_message);
      // }
      //
      // assert(best_fit_cad_model_pose_vector.size() == numberOfObjects);
      // for (uint i = 0; i < best_fit_cad_model_pose_vector.size(); i++) {
      //     geometry_msgs::PoseStamped poseStamped;
      //     poseStamped.header.stamp = ros::Time::now();
      //     poseStamped.header.frame_id = "kinect2_link";
      //     // or kinect2_ir_optical_frame
      //     tf::poseEigenToMsg(
      //         best_fit_cad_model_pose_vector[i].cast<double>(), poseStamped.pose);
      //     res.cad_model_pose_array.push_back(poseStamped);
      // }
      //
      // res.success.data = true;
      //
      // // Stuff to save. The following variables could be saved off during the
      // // loops such that the reverse transforms only need be performed once per
      // // fit cad model. It is easier however to perform the transforms for all
      // // and save the best (already transformed) cad model.
      //
      // // Eigen::Matrix<double, 3, 3> best_eigenvectors =
      // //     input_pca_params.eigenvectors;
      // // Eigen::Vector4d best_centroid = input_pca_params.centroid;
      // // boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> best_input_cloud;
      // // best_input_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
      // // boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> best_target_cloud;
      // // best_target_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
      // // pcl::copyPointCloud(input_pca_params.output_cloud, best_input_cloud);
      // // pcl::copyPointCloud(target_pca_params.output_cloud, best_target_cloud);
      // // Eigen::Affine3f best_transform = icp_params.transform;
      //
      // // The final transformed raw points
      // // input_pca_params.output_cloud
      //
      // // The final transformed cad model points
      // // target_pca_params.output_cloud
      //
      // ROS_INFO("Finished with this service call request!");
      //
      // return true;

}

int main(int argc, char **argv) {

        ros::init(argc, argv, "fit_cad_model_node");
        nh_ = new ros::NodeHandle("~");
        tf_listener = new tf::TransformListener();

        ros::ServiceServer service = nh_->advertiseService(
                "/cartesian_grasping/model_fitting", fit_cad_model);

        ros::spin();

        return 0;
}
