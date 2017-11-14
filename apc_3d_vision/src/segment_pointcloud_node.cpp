#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <image_geometry/pinhole_camera_model.h>

#include <opencv2/highgui/highgui.hpp>

#include <apc_msgs/SegmentPointcloudFromLabels.h>
#include <apc_msgs/AlignedDimensions.h>

#include <apc_msgs/Camera_Info.h>

#include <boost/foreach.hpp>

#include <sstream>

#include <cmath>

/**
*Node for detecting 3D clusters and align them with corresponding 2D images
*/

using namespace message_filters;
using namespace std;
using namespace pcl;
using namespace cv;
using namespace ros;

typedef struct Proposal{
    cv::Rect bounding_box;
    std::vector<cv::Point2d> imagePoints;
    uint32_t label;
} proposal_t;

class ObjectSegment
{


public:

    ObjectSegment(ros::NodeHandle nh):
        nh_(nh),
        camera_initialized_(false),
        input_cloud_(new pcl::PointCloud<pcl::PointXYZRGB>),
        temp_cloud_(new pcl::PointCloud<pcl::PointXYZRGB>),
        output_cloud_(new pcl::PointCloud<pcl::PointXYZRGB>)
    {
        // Just do this once.
        std::string camera_info_topic_name = "/realsense_wrist/rgb/camera_info";
        sensor_msgs::CameraInfoConstPtr camera_info_ptr = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(camera_info_topic_name);
        onCameraInfo(camera_info_ptr);

        segmentPCSrv_ = nh_.advertiseService("segment_pointcloud_from_labels", &ObjectSegment::segmentPointcloudFromLabels, this);

        sor_.setMeanK(50);
        sor_.setStddevMulThresh(1.0);
    }

    /**
    * @brief onCameraInfo gets the intrinsic camara info
    * @param camara_info the info
    */
    void onCameraInfo(const sensor_msgs::CameraInfoConstPtr &camara_info){

        if (camera_initialized_) return;
        else{
            ROS_INFO_STREAM("Camera Initialised!");
            model_.fromCameraInfo(camara_info);
            camera_initialized_=true;
        }
    }

    bool segmentPointcloudFromLabels(apc_msgs::SegmentPointcloudFromLabels::Request &req,
                                     apc_msgs::SegmentPointcloudFromLabels::Response &res)
    {
        pcl::fromROSMsg(req.input_cloud, *input_cloud_);

        cv_bridge::CvImagePtr cv_ptr;
        cv::Mat labels;
        decltype(ros::Time::now()) t1, t2;
        t1 = ros::Time::now();
        try{
            cv_ptr = cv_bridge::toCvCopy(req.classification, sensor_msgs::image_encodings::MONO8);
            labels = cv_ptr->image;
        }
        catch (cv_bridge::Exception& e){
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return true;
        }
        t2 = ros::Time::now();
        ROS_INFO_STREAM("Getting image from message: " << (t2 - t1));

        t1 = ros::Time::now();
        double min_seg_d, max_seg_d;
        int max_seg;
        cv::minMaxIdx(labels, &min_seg_d, &max_seg_d);
        max_seg = static_cast<int>(max_seg_d);
        t2 = ros::Time::now();
        ROS_INFO_STREAM("Getting min/max from labels: " << (t2 - t1));

        ROS_INFO_STREAM("Number of segments is: " << max_seg << std::endl);

        // t1 = ros::Time::now();
        // sor_.setInputCloud(input_cloud_);
        // sor_.filter(*input_cloud_);
        // t2 = ros::Time::now();
        // ROS_INFO_STREAM("Setting SOR stuff: " << (t2 - t1));

        t1 = ros::Time::now();
        pc_segments_.resize(max_seg);
        for(int i = 0; i < max_seg; i++) {
            pc_segments_[i].resize(0);
            pc_segments_[i].reserve(input_cloud_->size());
        }
        t2 = ros::Time::now();
        ROS_INFO_STREAM("Adding PC segments: " << (t2 - t1));

        t1 = ros::Time::now();
        for(auto &point: *input_cloud_) {
            if(point.r == 0) {
                continue;
            }
            cv::Point3d xyz = cv::Point3d(point.x, point.y, point.z);
            cv::Point2d pt = model_.project3dToPixel(xyz);
            if (!std::isnan(pt.x) && !std::isnan(pt.y)){
                int label = (int)labels.at<uchar>((int)pt.y, (int)pt.x);
                if(label > 0) {
                    pc_segments_[label-1].push_back(point);
                }
            }
        }
        t2 = ros::Time::now();
        ROS_INFO_STREAM("Adding points to PC segments: " << (t2 - t1));

        t1 = ros::Time::now();
        res.segmented_pointclouds.reserve(max_seg);
        res.aligned_dimensions.reserve(max_seg);
        for(int i = 0; i < max_seg; i++) {
            // Push back the point clouds.
            sensor_msgs::PointCloud2 pc_msg;
            pcl::toROSMsg(pc_segments_[i], pc_msg);
            pc_msg.header = req.input_cloud.header;
            res.segmented_pointclouds.push_back(pc_msg);

            apc_msgs::AlignedDimensions dimensions_msg;

            // Compute PCA to create dimensions aligned along the 3 principal axes

            if(pc_segments_[i].size() < 10) {
                dimensions_msg.width.data = 0;
                dimensions_msg.height.data = 0;
                dimensions_msg.depth.data = 0;
                dimensions_msg.average_z.data = 0;
                dimensions_msg.min_z.data = 0;
                dimensions_msg.max_z.data = 0;
                res.aligned_dimensions.push_back(dimensions_msg);
                continue;
            }
            // Calculate the centroid
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(pc_segments_[i], centroid);
            pcl::demeanPointCloud<pcl::PointXYZRGB>(pc_segments_[i], centroid, *temp_cloud_);
            /// Compute PCA for the cloud
            pcl::PCA<pcl::PointXYZRGB> pca;
            pca.setInputCloud(temp_cloud_);
            Eigen::Matrix3d eigenvectors_temp = pca.getEigenVectors().cast<double>();
            Eigen::Matrix<double, 3, 3> eigenvectors = eigenvectors_temp;
            // Ensure z-axis direction satisfies right-hand rule
            eigenvectors.col(2) = eigenvectors.col(0).cross(eigenvectors.col(1));
            // Align the principal axes to the objects frame
            Eigen::Matrix<double, 4, 4> transformation(Eigen::Matrix<double, 4, 4>::Identity());
            transformation.template block<3, 3>(0, 0) = eigenvectors.transpose();
            pcl::transformPointCloud(*temp_cloud_, *output_cloud_, transformation);
            pcl::PointXYZRGB lowerBound;
            pcl::PointXYZRGB upperBound;
            pcl::getMinMax3D(*output_cloud_, lowerBound, upperBound);

            dimensions_msg.width.data = upperBound.x - lowerBound.x;
            dimensions_msg.height.data = upperBound.y - lowerBound.y;
            dimensions_msg.depth.data = upperBound.z - lowerBound.z;

            // Match our global reference frame.
            // Yaw is rotation around z.
            eigenvectors.col(0) = eigenvectors_temp.col(1);
            eigenvectors.col(1) = -1 * eigenvectors_temp.col(0);
            eigenvectors.col(2) = eigenvectors.col(0).cross(eigenvectors.col(1));

            eigenvectors = eigenvectors.normalized();
            tf::Matrix3x3 out;
            tf::matrixEigenToTF(eigenvectors, out);
            double roll, pitch, yaw;
            out.getRPY(roll, pitch, yaw);
            // align the yaw to a reachable direction.
            if(yaw > 1.57 || yaw < -1.57) {
                yaw = yaw + 3.14;
            }
            tf::Quaternion transformed_q = tf::createQuaternionFromRPY(0, 0, yaw);
            tf::quaternionTFToMsg(transformed_q, dimensions_msg.pca_rotation);

            // Calculate some other useful things
            double max_z = 0;
            double min_z = 10.0;
            double average_z = 0;
            for(auto &point: pc_segments_[i]) {
                if(point.z > max_z) {
                    max_z = point.z;
                }
                if(point.z < min_z) {
                    min_z = point.z;
                }
                average_z = average_z + point.z;
            }
            average_z = average_z/pc_segments_[i].size();

            dimensions_msg.average_z.data = average_z;
            dimensions_msg.min_z.data = min_z;
            dimensions_msg.max_z.data = max_z;

            res.aligned_dimensions.push_back(dimensions_msg);

        }
        t2 = ros::Time::now();
        ROS_INFO_STREAM("Everything else: " << (t2 - t1));
        return true;

    }

private:

    ros::NodeHandle nh_;
    image_geometry::PinholeCameraModel model_;
    bool camera_initialized_;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor_;

    std::vector<pcl::PointCloud<pcl::PointXYZRGB>> pc_segments_;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud_;

    //pub sub
    // ros::Subscriber camera_info_sub_;
    // boost::shared_ptr<message_filters::Synchronizer<sync_policies::ApproximateTime<sensor_msgs::PointCloud2,sensor_msgs::Image> > > syncPtr_;
    // ros::Publisher proposal_pub_;
    // ros::Publisher overlay_image_pub_;
    // ros::Publisher object_pub_;


    //services
    // ros::ServiceServer enableProposalPubSrv_;
    // ros::ServiceServer enableObjectPubSrv_;
    // ros::ServiceServer enableOverlayPubSrv_;
    // ros::ServiceServer doObjectProposalSrv_;
    // ros::ServiceServer changeCameraInfoSrv_;
    ros::ServiceServer segmentPCSrv_;

    // ros::ServiceClient cnn_client_;


};

int main(int argc, char** argv){

    ros::init(argc,argv,"segment_pointcloud_node");
    ros::NodeHandle nh("~");
    ObjectSegment proposal(nh);

    // cv::namedWindow("Display window", cv::WINDOW_AUTOSIZE );

    ros::spin();

    ROS_INFO("Terminating segment_pointcloud_node ...");

    //cv::destroyAllWindows();
    return 0;
}
