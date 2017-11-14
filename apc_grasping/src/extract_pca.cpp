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

#include <apc_msgs/ExtractPca.h>

#include <pcl/common/geometry.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/common/pca.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <pcl/point_representation.h>

#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/passthrough.h>

ros::Publisher grip_pose_pub;
ros::NodeHandle *nh_;
tf::TransformListener *tf_listener;

typedef pcl::PointXYZ InputPointType;
typedef pcl::PointNormal PointTypeNormal;
typedef pcl::PointXYZ PointTypeXYZ;
typedef pcl::PointCloud<InputPointType> InputPointCloud;
typedef pcl::PointCloud<PointTypeXYZ> PointCloudXYZ;
typedef pcl::PointCloud<PointTypeNormal> PointCloudNormal;

boost::shared_ptr<pcl::visualization::PCLVisualizer> visualizer;

bool visualize_pca (pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointXYZ axis_0_start, pcl::PointXYZ axis_0_end) {

      ROS_INFO("[CARTESIAN GRASP] Showing PCA Results ...");
      // Visualisation
      pcl::visualization::PCLVisualizer visu1("PCA Alignment");
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> input_colour(input_cloud, 0.0, 255.0, 0.0);
      visu1.addPointCloud(input_cloud, input_colour, "original");
      visu1.addLine<pcl::PointXYZ>(
           axis_0_start, (axis_0_end), 255.0, 0.0, 0.0, "original_principal_axis_0");
      visu1.addCoordinateSystem(0.1);
      visu1.spin();

      return true;
}

geometry_msgs::Pose align_pca(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,bool publish) {

    ROS_INFO("[CARTESIAN GRASP] Starting the principle axis function ...");

    // Declare local temporary variables
    pcl::PointCloud<pcl::PointXYZ>::Ptr local_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Calculate the centroid
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*input_cloud,centroid);
    // Subtract the centroid from the point cloud (Visualization purposes)
    pcl::demeanPointCloud<pcl::PointXYZ>(*input_cloud, centroid,
                                         *local_cloud);

    ROS_INFO("[CARTESIAN GRASP] Computing the PCA");
    // Compute PCA for the input cloud
    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud(local_cloud);

    Eigen::Matrix3f eigenvectors_float = pca.getEigenVectors();
    Eigen::Matrix<double, 3, 3> eigenvectors =
        eigenvectors_float.cast<double>();
    Eigen::Matrix<double, 3, 3> eigenvectors_temp =
        eigenvectors_float.cast<double>();

    eigenvectors.col(0) = eigenvectors_temp.col(1);
    //flip our x axis orientation
    eigenvectors.col(1) = -1 * eigenvectors_temp.col(0);
    // Ensure z-axis direction satisfies right-hand rule
    eigenvectors.col(2) = eigenvectors.col(0).cross(eigenvectors.col(1));

    //Normalise the eigenvectors
    eigenvectors = eigenvectors.normalized();
    //transposing eigenvectors is the same as inversing them which allows us to get a transformation matrix
    //eigenvectors = eigenvectors.transpose();
    tf::Matrix3x3 out;
    tf::matrixEigenToTF(eigenvectors,out);

    tf::Quaternion transformed_q;
    double roll, pitch, yaw;
    out.getRPY(roll, pitch, yaw);
    yaw = yaw * 180.0/3.1415;
    if(yaw < -90) {
      yaw = yaw + 180;
    }
    if(yaw > 90) {
      yaw = yaw - 180;
    }
    if(yaw > 85) {
      yaw = 85;
    }
    if(yaw < -85) {
      yaw = -85;
    }
    yaw = yaw + 180; // The camera is backwards.
    yaw = yaw/180.0*3.1415;
    // transformed_q = tf::createQuaternionFromRPY(roll,pitch,yaw); //roll, pitch, yaw
    transformed_q = tf::createQuaternionFromRPY(0,0,yaw); //roll, pitch, yaw
    ROS_INFO("[CARTESIAN GRASP] RPY: %f, %f, %f\n", roll, pitch, yaw);

    //create a broadcaster if we want to publish the transform
    static tf2_ros::TransformBroadcaster static_broadcaster;

    geometry_msgs::TransformStamped pca_transformStamped;
    geometry_msgs::Pose grip_pose;

    grip_pose.position.x = centroid[0];
    grip_pose.position.y = centroid[1];
    grip_pose.position.z = centroid[2];
    grip_pose.orientation.x = transformed_q.x();
    grip_pose.orientation.y = transformed_q.y();
    grip_pose.orientation.z = transformed_q.z();
    grip_pose.orientation.w = transformed_q.w();

    if (publish) {
      //publish the transform
      pca_transformStamped.header.stamp = ros::Time::now();
      pca_transformStamped.header.frame_id = "realsense_wrist_rgb_optical_frame";
      pca_transformStamped.child_frame_id = "PCA";
      pca_transformStamped.transform.translation.x = centroid[0];
      pca_transformStamped.transform.translation.y = centroid[1];
      pca_transformStamped.transform.translation.z = centroid[2];
      pca_transformStamped.transform.rotation.x = transformed_q.x();
      pca_transformStamped.transform.rotation.y = transformed_q.y();
      pca_transformStamped.transform.rotation.z = transformed_q.z();
      pca_transformStamped.transform.rotation.w = transformed_q.w();

      for(int i = 0; i < 100; i++) {
          static_broadcaster.sendTransform(pca_transformStamped);
          ros::Duration(0.1).sleep();
      }
    }

    ROS_INFO("[CARTESIAN GRASP] Finished calculating PCA, returning pose..");
    //return out pose
    return grip_pose;
}

bool extract_pca(apc_msgs::ExtractPca::Request &req,
                 apc_msgs::ExtractPca::Response &res) {

        ROS_INFO("[CARTESIAN GRASP] Got point cloud, finding surface properties");

        //CREATE SOME OBJECTS

        InputPointCloud::Ptr objectCloud(new InputPointCloud);

        //INPUT OUR POINT CLOUD
        std::string input_frame = "realsense_wrist_rgb_optical_frame";
        pcl::fromROSMsg(req.cloud, *objectCloud); // Put our point cloud into the ObjectCloudlabelled object

        //REMOVE NaN FROM CLOUD
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*objectCloud, *objectCloud,
                                     indices);
        // Check cloud has points
        if (objectCloud->width > 0) {
          // ensure point cloud is in XYZ format
          PointCloudXYZ::Ptr objectCloud_XYZ(new PointCloudXYZ);
          pcl::copyPointCloud(*objectCloud, *objectCloud_XYZ);
          ROS_INFO("[CARTESIAN GRASP] Attempting to find the PCA");

          res.grasp_poses = align_pca(objectCloud_XYZ, req.publish);

        } else {
          ROS_INFO("[CARTESIAN GRASPING] Not enough points in object");
          return true;
        }

        return true;
}


int main(int argc, char **argv) {

        ros::init(argc, argv, "extract_pca_node");
        nh_ = new ros::NodeHandle("~");
        tf_listener = new tf::TransformListener();

        ros::ServiceServer service = nh_->advertiseService(
                "/apc_grasping/extract_pca", extract_pca);

        ros::spin();

        return 0;
}
