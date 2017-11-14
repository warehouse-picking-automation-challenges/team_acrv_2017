#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>

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

#include <apc_msgs/ImageToWorld.h>
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

class ImageToWorld {

public:

    ImageToWorld(ros::NodeHandle nh):
        nh_(nh),
        camera_initialized_(false)
    {
        // Just do this once.
        std::string camera_info_topic_name = "/realsense_wrist/rgb/camera_info";
        sensor_msgs::CameraInfoConstPtr camera_info_ptr = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(camera_info_topic_name);
        onCameraInfo(camera_info_ptr);

        image_to_world_service_handle = nh_.advertiseService("image_to_world", &ImageToWorld::image_to_world_service, this);

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
            camera_initialized_ = true;
        }
    }

    bool image_to_world_service(apc_msgs::ImageToWorld::Request &req,
                                     apc_msgs::ImageToWorld::Response &res)
    {
        pcl::PointCloud<pcl::PointXYZRGB> input_cloud;
        pcl::fromROSMsg(req.cloud, input_cloud);
        auto image_x = req.image_x.data;
        auto image_y = req.image_y.data;

        auto world_x = 0.0;
        auto world_y = 0.0;
        auto world_z = 0.0;
        auto best_distance = 10000000.0;

        auto num_points = 0;

        for(auto &point: input_cloud) {
            num_points++;
            if(point.r == 0) {
                // Sometimes the pointcloud has 100k points at the origin.
                // This remedies it
                continue;
            }
            cv::Point3d xyz = cv::Point3d(point.x, point.y, point.z);
            cv::Point2d pt = model_.project3dToPixel(xyz);
            if (!std::isnan(pt.x) && !std::isnan(pt.y)){
                auto x_dist = std::abs(pt.x - image_x);
                auto y_dist = std::abs(pt.y - image_y);

                auto distance = std::sqrt(x_dist*x_dist + y_dist*y_dist);

                if (distance < best_distance) {
                    world_x = xyz.x;
                    world_y = xyz.y;
                    world_z = xyz.z;
                    best_distance = distance;
                }
                if (distance < 1.0) {
                    break;
                }
            }
        }

        cv::Point2d pixel_point(image_x, image_y);
        cv::Point3d xyz_unit_vector = model_.projectPixelTo3dRay(pixel_point);

        cv::Point3d xyz;

        if (num_points > 1) {
            xyz.x = xyz_unit_vector.x * world_z/xyz_unit_vector.z;
            xyz.y = xyz_unit_vector.y * world_z/xyz_unit_vector.z;
            xyz.z = xyz_unit_vector.z * world_z/xyz_unit_vector.z;
            ROS_INFO_STREAM("World depth from rgb is: " << xyz.z);
        } else {
            xyz.x = xyz_unit_vector.x * 0.5/xyz_unit_vector.z;
            xyz.y = xyz_unit_vector.y * 0.5/xyz_unit_vector.z;
            xyz.z = xyz_unit_vector.z * 0.5/xyz_unit_vector.z;
            ROS_INFO_STREAM("No points in provided segment. Choosing world depth of 0.5m (camera to item)");
        }

        res.world_x.data = xyz.x;
        res.world_y.data = xyz.y;
        res.world_z.data = xyz.z;

        return true;
    }

private:

    ros::NodeHandle nh_;
    image_geometry::PinholeCameraModel model_;
    bool camera_initialized_;

    ros::ServiceServer image_to_world_service_handle;
};

int main(int argc, char** argv) {

    ros::init(argc,argv,"image_to_world_node");
    ros::NodeHandle nh("~");
    ImageToWorld proposal(nh);

    ros::spin();

    ROS_INFO("Terminating image_to_world_node ...");

    return 0;
}
