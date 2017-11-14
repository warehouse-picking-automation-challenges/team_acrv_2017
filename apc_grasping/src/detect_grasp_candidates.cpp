/*
   Copyright 2016 Australian Centre for Robotic Vision
 */
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <visualization_msgs/MarkerArray.h>
#include <string>

#include <apc_grasping/pcl_filters.h>

#include <apc_msgs/DetectGraspCandidates.h>

#include <pcl/common/geometry.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>

ros::Publisher grasp_pub, marker_array_pub, marker_pub, boundaries_pub,
               downsampled_pub;
ros::NodeHandle *nh_;
tf::TransformListener *tf_listener;

typedef pcl::PointXYZ InputPointType;
typedef pcl::PointNormal PointTypeNormal;
typedef pcl::PointXYZ PointTypeXYZ;
typedef pcl::PointCloud<InputPointType> InputPointCloud;
typedef pcl::PointCloud<PointTypeXYZ> PointCloudXYZ;
typedef pcl::PointCloud<PointTypeNormal> PointCloudNormal;

boost::shared_ptr<pcl::visualization::PCLVisualizer> visualizer;

float distance_to_closest_point(int K, pcl::PointXYZ searchPoint,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

        kdtree.setInputCloud(cloud);

        // K nearest neighbor search
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);

        if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch,
                                  pointNKNSquaredDistance) > 0) {
                // for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i)
                // std::cout << "    " << cloud->points[pointIdxNKNSearch[i]].x << " "
                //           << cloud->points[pointIdxNKNSearch[i]].y << " "
                //           << cloud->points[pointIdxNKNSearch[i]].z
                //           << " (squared distance: " << pointNKNSquaredDistance[i] <<
                //           ")"
                //           << std::endl;

                return sqrt(pointNKNSquaredDistance[0]);

        } else {
                return 0.0;
        }
}

std::vector<std::pair<geometry_msgs::Pose, double> >
sort_grasp_vectors(std::vector<geometry_msgs::Pose> &grasps,
                   std::vector<double> utilities) {
        // initialize original index locations
        std::vector<std::pair<geometry_msgs::Pose, double> > grasp_score_pairs;
        for (unsigned int i = 0; i < grasps.size(); i++) {
                std::pair<geometry_msgs::Pose, double> pair(grasps[i], utilities[i]);
                grasp_score_pairs.push_back(pair);
        }

        std::sort(
                grasp_score_pairs.begin(), grasp_score_pairs.end(),
                boost::bind(&std::pair<geometry_msgs::Pose, double>::second, _1) >
                boost::bind(&std::pair<geometry_msgs::Pose, double>::second, _2));

        return grasp_score_pairs;
}

geometry_msgs::PoseArray
compute_grasp_utility(geometry_msgs::PoseArray grasp_pose_pool,
                      std::vector<double> curvature,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr boundaryCloud,
                      std::vector<double> *utility, double curvature_threshold,
                      double curvature_weight, double boundary_threshold,
                      double boundary_weight) {
        geometry_msgs::PoseArray selected_grasps;

        std::vector<float> distance_to_boundary;
        std::vector<float> normalised_curvature;

        for (unsigned int i = 0; i < grasp_pose_pool.poses.size(); i++) {
                pcl::PointXYZ grasp_point(grasp_pose_pool.poses[i].position.x,
                                          grasp_pose_pool.poses[i].position.y,
                                          grasp_pose_pool.poses[i].position.z);

                float min_distance =
                        distance_to_closest_point(1, grasp_point, boundaryCloud);

                // Find max distance in order to normalise min_distance
                // pcl::getMinMax3D<pcl::PointXYZ>(*boundaryCloud, min_pt, max_pt);
                Eigen::Vector4f point_4f(grasp_point.x, grasp_point.y, grasp_point.z, 0.0);
                Eigen::Vector4f max_pt_4f;

                pcl::getMaxDistance(*boundaryCloud, point_4f, max_pt_4f);
                pcl::PointXYZ max_pt(max_pt_4f[0], max_pt_4f[1], max_pt_4f[2]);

                float max_distance =
                        pcl::geometry::distance<pcl::PointXYZ>(grasp_point, max_pt);

                float norm_min_distance = min_distance / max_distance;

                // compute normalised curvature using min and max values
                double maxCurvature = *std::max_element(curvature.begin(), curvature.end());
                double minCurvature = *std::min_element(curvature.begin(), curvature.end());
                normalised_curvature.push_back((curvature[i] - minCurvature) /
                                               (maxCurvature - minCurvature));

                if (min_distance > boundary_threshold) {
                        ROS_DEBUG_STREAM("Min Distance: " << min_distance << " Norm Min Distance: "
                                                         << norm_min_distance
                                                         << " Max Distance: " << max_distance
                                                         << " Normalised Curvature: "
                                                         << (1 - normalised_curvature[i]));
                        // ROS_DEBUG_STREAM("Found Grasp Pose at: " <<
                        // grasp_pose_pool.poses[i]);
                        // ROS_DEBUG_STREAM("Curvature: " << curvature[i]);
                        // ROS_DEBUG_STREAM("Distance from boundary: " << min_distance);

                        selected_grasps.poses.push_back(grasp_pose_pool.poses[i]);

                        distance_to_boundary.push_back(norm_min_distance);

                        utility->push_back((1 - normalised_curvature[i]) * curvature_weight +
                                           norm_min_distance * boundary_weight);
                }
        }

        return selected_grasps;
}

geometry_msgs::PoseArray
select_grasps(geometry_msgs::PoseArray grasp_pose_pool,
              std::vector<double> curvature,
              pcl::PointCloud<pcl::PointXYZ>::Ptr boundaryCloud,
              double curvature_threshold, double boundary_threshold) {
        geometry_msgs::PoseArray selected_grasps;

        for (unsigned int i = 0; i < grasp_pose_pool.poses.size(); i++) {
                pcl::PointXYZ grasp_point(grasp_pose_pool.poses[i].position.x,
                                          grasp_pose_pool.poses[i].position.y,
                                          grasp_pose_pool.poses[i].position.z);

                float min_distance =
                        distance_to_closest_point(1, grasp_point, boundaryCloud);

                if ((curvature[i] < curvature_threshold) &&
                    (min_distance > boundary_threshold)) {
                        // ROS_DEBUG_STREAM("Found Grasp Pose at: " <<
                        // grasp_pose_pool.poses[i]);
                        // ROS_DEBUG_STREAM("Curvature: " << curvature[i]);
                        // ROS_DEBUG_STREAM(
                        //     "Normalised Distance from boundary: " << norm_min_distance);
                        selected_grasps.poses.push_back(grasp_pose_pool.poses[i]);
                }
        }

        return selected_grasps;
}

geometry_msgs::Pose translate_pose(geometry_msgs::Pose input_pose,
                                   Eigen::Vector3d offset) {
        Eigen::Affine3d input_pose_eigen, input_pose_eigen_translated;
        geometry_msgs::Pose output_pose;
        tf::poseMsgToEigen(input_pose, input_pose_eigen);

        Eigen::Translation3d translation(offset);

        input_pose_eigen_translated = input_pose_eigen * translation;

        tf::poseEigenToMsg(input_pose_eigen_translated, output_pose);

        return output_pose;
}

void publishGraspMarkers(geometry_msgs::PoseArray grasp_poses,
                         std::vector<double> grasp_utility,
                         double marker_scale) {
        visualization_msgs::MarkerArray grasp_array_delete, grasp_array;

        for (unsigned int i = 0; i < 5000; i++) {
                visualization_msgs::Marker marker;
                marker.header.frame_id = grasp_poses.header.frame_id;
                marker.header.stamp = ros::Time::now();
                marker.ns = "grasp_poses";
                marker.id = i;
                // marker.type = visualization_msgs::Marker::ARROW;
                marker.action = visualization_msgs::Marker::DELETE;

                grasp_array_delete.markers.push_back(marker);
        }

        marker_array_pub.publish(grasp_array_delete);

        double max = grasp_utility[0];
        double min = grasp_utility[grasp_utility.size() - 1];

        for (unsigned int i = 0; i < grasp_poses.poses.size(); i++) {
                visualization_msgs::Marker marker;
                marker.header.frame_id = grasp_poses.header.frame_id;
                marker.header.stamp = ros::Time::now();
                marker.ns = "grasp_poses";
                marker.id = i;
                marker.type = visualization_msgs::Marker::ARROW;
                marker.action = visualization_msgs::Marker::ADD;

                // marker.scale.x = grasp_utility[i] * marker_scale;
                marker.scale.x = marker_scale / 4.0;
                marker.scale.y = 0.0035;
                marker.scale.z = 0.0035;
                marker.color.a = 1.0;
                marker.color.r = 1.0;
                marker.color.g = 0.0;
                marker.color.b = 0.0;
                marker.pose = translate_pose(grasp_poses.poses[i],
                                             Eigen::Vector3d(-marker.scale.x, 0, 0));
                grasp_array.markers.push_back(marker);
        }

        marker_array_pub.publish(grasp_array);
}

void graspPoseFromPointNormal(const pcl::PointNormal &pointNormal,
                              geometry_msgs::Pose *grasp_pose) {

        Eigen::Vector3f normal_vector(pointNormal.normal_x, pointNormal.normal_y, pointNormal.normal_z);
        Eigen::Quaternion<float> q = Eigen::Quaternion<float>::FromTwoVectors(Eigen::Vector3f::UnitZ(), normal_vector);

        grasp_pose->position.y = pointNormal.y;
        grasp_pose->position.x = pointNormal.x;
        grasp_pose->position.z = pointNormal.z;
        grasp_pose->orientation.x = q.x();
        grasp_pose->orientation.y = q.y();
        grasp_pose->orientation.z = q.z();
        grasp_pose->orientation.w = q.w();
}

void computePointNormalCloud(
        pcl::PointCloud<pcl::PointXYZ>::Ptr objectCloud_XYZ,
        pcl::PointCloud<pcl::PointXYZ>::Ptr objectCloudDownSampled_XYZ,
        PointCloudNormal::Ptr objectCloudPointNormal, double suction_cup_radius) {

        ROS_INFO("Computing normals on down sampled cloud");

        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*objectCloud_XYZ, centroid);

        Eigen::Vector3f view_point(centroid[0], centroid[1], centroid[2]);
        view_point.normalize();

        pcl::PointCloud<pcl::Normal>::Ptr cloudNormals(
                new pcl::PointCloud<pcl::Normal>);
        pcl_filters::compute_normals_down_sampled(
                objectCloud_XYZ, objectCloudDownSampled_XYZ, cloudNormals, view_point,
                suction_cup_radius);

        pcl::concatenateFields(*objectCloudDownSampled_XYZ, *cloudNormals,
                               *objectCloudPointNormal);
}

bool lookupAffineTransform(std::string target_frame, std::string source_frame,
                           Eigen::Affine3d *transform_eigen) {
        tf::Transform output_transform;
        tf::StampedTransform stampedtransform;
        bool tf_success = tf_listener->waitForTransform(
                target_frame, source_frame, ros::Time(0), ros::Duration(1.0));
        if (tf_success) {
                tf_listener->lookupTransform(target_frame, source_frame, ros::Time(0),
                                             stampedtransform);
                output_transform.setOrigin(stampedtransform.getOrigin());
                output_transform.setRotation(stampedtransform.getRotation());

                tf::transformTFToEigen(output_transform, *transform_eigen);

                return true;
        } else {
                ROS_INFO("Can't Find Transform from %s to %s!", source_frame.c_str(),
                         target_frame.c_str());
                return false;
        }
}

void transformPoseArray(const geometry_msgs::PoseArray &input_poses,
                        geometry_msgs::PoseArray *output_poses,
                        Eigen::Affine3d transform) {
        for (int i = 0; i < input_poses.poses.size(); i++) {
                geometry_msgs::Pose transformed_pose;
                Eigen::Affine3d input_pose_eigen, transformed_pose_eigen;
                tf::poseMsgToEigen(input_poses.poses[i], input_pose_eigen);
                transformed_pose_eigen = transform * input_pose_eigen;
                tf::poseEigenToMsg(transformed_pose_eigen, transformed_pose);
                output_poses->poses.push_back(transformed_pose);
        }
}

void graspPoseFromCentroid(pcl::PointCloud<pcl::PointXYZ>::Ptr objectCloud_XYZ,
                           geometry_msgs::PoseArray *output_grasps,
                           std::vector<double> *grasp_utility,
                           std::string target_frame, std::string source_frame) {

        Eigen::Affine3d transform_eigen;
        geometry_msgs::PoseArray grasps;

        pcl::PointCloud<pcl::PointXYZ>::Ptr objectClound_base(
                new pcl::PointCloud<pcl::PointXYZ>);

        int num_grasps;
        double square_width;

        nh_->param("num_grasps", num_grasps, 10);
        nh_->param("square_grid_size", square_width, 0.05);


        lookupAffineTransform(target_frame, source_frame, &transform_eigen);

        pcl::transformPointCloud(*objectCloud_XYZ, *objectClound_base,
                                 transform_eigen);

        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*objectClound_base, centroid);

        geometry_msgs::Pose centroid_grasp_up, centroid_grasp_front;
        centroid_grasp_up.position.x = centroid[0];
        centroid_grasp_up.position.y = centroid[1];
        centroid_grasp_up.position.z = centroid[2];

        centroid_grasp_front.position = centroid_grasp_up.position;

        centroid_grasp_up.orientation.x = 0;
        centroid_grasp_up.orientation.y = 0;
        centroid_grasp_up.orientation.z = 0;
        centroid_grasp_up.orientation.w = 1;

        centroid_grasp_front.orientation.x = 0;
        centroid_grasp_front.orientation.y = -0.707;
        centroid_grasp_front.orientation.z = 0;
        centroid_grasp_front.orientation.w = 0.707;

        grasps.poses.push_back(centroid_grasp_up);
        grasp_utility->push_back(1.0);
        grasps.poses.push_back(centroid_grasp_front);
        grasp_utility->push_back(0.99);

        geometry_msgs::Pose pose_up = centroid_grasp_up;
        geometry_msgs::Pose pose_front = centroid_grasp_front;

        for(double y = centroid[1] - square_width/2; y < centroid[1] + square_width/2; y += square_width/num_grasps ) {
                for(double z = centroid[2] - square_width/2; z < centroid[2] + square_width/2; z += square_width/num_grasps ) {
                        pose_up.position.y = y;
                        pose_up.position.z = z;
                        grasps.poses.push_back(pose_up);
                        double dist = sqrt(pow(centroid[1]-y,2) + pow(centroid[2]-z,2));
                        double max_dist = sqrt(2.0)*square_width/2;
                        grasp_utility->push_back((max_dist - dist) / max_dist);
                }
        }

        for(double x = centroid[0] - square_width/2; x < centroid[0] + square_width/2; x += square_width/num_grasps ) {
                for(double y = centroid[1] - square_width/2; y < centroid[1] + square_width/2; y += square_width/num_grasps ) {
                        pose_front.position.x = x;
                        pose_front.position.y = y;
                        grasps.poses.push_back(pose_front);
                        double dist = sqrt(pow(centroid[0]-x,2) + pow(centroid[1]-y,2));
                        double max_dist = sqrt(2.0)*square_width/2;
                        grasp_utility->push_back((max_dist - dist) / max_dist);
                }
        }


        // for (int i = 1; i < num_grasps; i++) {
        //
        //         for (int j = 0; j < 4; j++) {
        //                 geometry_msgs::Pose pose_up = centroid_grasp_up;
        //                 geometry_msgs::Pose pose_front = centroid_grasp_front;
        //                 switch (j) {
        //                 case 0:
        //                         pose_up.position.y += i * grid_size;
        //                         pose_up.position.z += i * grid_size;
        //                         pose_front.position.x += i * grid_size;
        //                         pose_front.position.y += i * grid_size;
        //                         break;
        //                 case 1:
        //                         pose_up.position.y -= i * grid_size;
        //                         pose_up.position.z += i * grid_size;
        //                         pose_front.position.x -= i * grid_size;
        //                         pose_front.position.y += i * grid_size;
        //                         break;
        //                 case 2:
        //                         pose_up.position.y += i * grid_size;
        //                         pose_up.position.z -= i * grid_size;
        //                         pose_front.position.x += i * grid_size;
        //                         pose_front.position.y -= i * grid_size;
        //                         break;
        //                 case 3:
        //                         pose_up.position.y -= i * grid_size;
        //                         pose_up.position.z -= i * grid_size;
        //                         pose_front.position.x -= i * grid_size;
        //                         pose_front.position.y -= i * grid_size;
        //                         break;
        //                 default:
        //                         break;
        //                 }
        //                 grasps.poses.push_back(pose_up);
        //                 grasps.poses.push_back(pose_front);
        //                 grasp_utility->push_back(1.0 - ((double)i / (double)num_grasps));
        //         }
        // }

        transformPoseArray(grasps, output_grasps, transform_eigen.inverse());
}

bool graspPosesFromSurface(
        pcl::PointCloud<pcl::PointXYZ>::Ptr objectCloud_XYZ,
        pcl::PointCloud<pcl::PointXYZ>::Ptr objectCloudDownSampled_XYZ,
        pcl::PointCloud<pcl::PointXYZ>::Ptr boundaryCloudPCL,
        geometry_msgs::PoseArray *selected_grasp_poses,
        std::vector<double> *grasp_utility, std::string input_frame) {

        geometry_msgs::PoseArray grasp_poses;
        PointCloudNormal::Ptr objectCloudPointNormal(new PointCloudNormal);
        pcl::PointCloud<pcl::Boundary>::Ptr boundaries(
                new pcl::PointCloud<pcl::Boundary>);
        PointCloudNormal::Ptr graspPointNormalsDownsampled;

        double suction_cup_radius, grasp_sample_radius;
        double boundary_detector_radius, boundary_detector_angle, boundary_threshold,
               curvature_threshold, curvature_weight, boundary_weight;

        int boundary_detector_k;
        // Grab parameters from rosparam server
        nh_->param("suction_cup_radius", suction_cup_radius, 25.0 / 1000.0);
        nh_->param("grasp_sample_radius", grasp_sample_radius, 25.0 / 1000.0);

        nh_->param("curvature_weight", curvature_weight, 0.5);
        nh_->param("boundary_weight", boundary_weight, 0.5);

        nh_->param("curvature_threshold", curvature_threshold, 0.1);
        nh_->param("boundary_threshold", boundary_threshold, 0.01);
        nh_->param("boundary_detector_k", boundary_detector_k, 0);
        nh_->param("boundary_detector_radius", boundary_detector_radius, 0.03);
        nh_->param("boundary_detector_angle", boundary_detector_angle, M_PI / 2.0);

        computePointNormalCloud(objectCloud_XYZ, objectCloudDownSampled_XYZ,
                                objectCloudPointNormal, suction_cup_radius);

        ROS_INFO("Estimating Object Boundary");
        boundaries = pcl_filters::boundaryEstimation(
                objectCloudPointNormal, boundary_detector_k, boundary_detector_radius,
                boundary_detector_angle);

        ROS_INFO("Downsampling Grasp Normal Cloud");
        graspPointNormalsDownsampled = pcl_filters::downSample<pcl::PointNormal>(
                objectCloudPointNormal, grasp_sample_radius);

        ROS_INFO("Constructing Grasp Poses from cloud");
        if (graspPointNormalsDownsampled->size() > 0) {
                std::vector<double> curvatures;

                grasp_poses.header.frame_id = input_frame;
                grasp_poses.header.stamp = ros::Time::now();

                for (unsigned int i = 0; i < graspPointNormalsDownsampled->width; i++) {

                        // construct grasp poses from point normals
                        geometry_msgs::Pose grasp_pose;
                        graspPoseFromPointNormal(graspPointNormalsDownsampled->points[i],
                                                 &grasp_pose);

                        if( pcl_isnan(grasp_pose.orientation.x) ||
                            pcl_isnan(grasp_pose.orientation.y) ||
                            pcl_isnan(grasp_pose.orientation.z) ||
                            pcl_isnan(grasp_pose.orientation.w) ) {
                              continue;
                        }

                        grasp_poses.poses.push_back(grasp_pose);
                        curvatures.push_back(graspPointNormalsDownsampled->points[i].curvature);
                }

                // create point cloud of boundary points
                ROS_INFO("Creating Boundary Cloud for visualisation");
                for (unsigned int i = 0; i < objectCloudPointNormal->size(); i++) {
                        if (boundaries->points[i].boundary_point == 1) {
                                pcl::PointXYZ point;
                                point.x = objectCloudPointNormal->points[i].x;
                                point.y = objectCloudPointNormal->points[i].y;
                                point.z = objectCloudPointNormal->points[i].z;
                                boundaryCloudPCL->push_back(point);
                        }
                }

                // compute a grasp utility
                ROS_INFO("Computing Grasp utilities on grasp candidats");
                *selected_grasp_poses = compute_grasp_utility(
                        grasp_poses, curvatures, boundaryCloudPCL, grasp_utility,
                        curvature_threshold, curvature_weight, boundary_threshold,
                        boundary_weight);

        } else {
                ROS_INFO("Not enough points in down sampled object");
                return false;
        }

        return true;
}

void publishClouds(
        pcl::PointCloud<pcl::PointXYZ>::Ptr boundaryCloudPCL,
        pcl::PointCloud<pcl::PointXYZ>::Ptr objectCloudDownSampled_XYZ,
        std::string input_frame) {

        // Publish object clouds and grasp markers
        ROS_INFO("Publishing Boundary and Object Clouds");
        sensor_msgs::PointCloud2 boundaryCloud, objectDownSampled;
        pcl::toROSMsg(*boundaryCloudPCL, boundaryCloud);
        pcl::toROSMsg(*objectCloudDownSampled_XYZ, objectDownSampled);

        boundaryCloud.header.frame_id = input_frame;
        boundaryCloud.header.stamp = ros::Time::now();
        boundaries_pub.publish(boundaryCloud);

        objectDownSampled.header.frame_id = input_frame;
        objectDownSampled.header.stamp = ros::Time::now();
        downsampled_pub.publish(objectDownSampled);
}

// void object_cloud_callback(const sensor_msgs::PointCloud2ConstPtr msg){
bool grasp_candidate_detection(apc_msgs::DetectGraspCandidates::Request &req,
                               apc_msgs::DetectGraspCandidates::Response &res) {
        ROS_INFO("Got Object Cloud estimating surface properties");
        decltype(ros::Time::now()) t1, t2;

        InputPointCloud::Ptr objectCloud(new InputPointCloud);
        pcl::PointCloud<pcl::PointXYZL>::Ptr objectCloudlabelled(
                new pcl::PointCloud<pcl::PointXYZL>);
        InputPointCloud::Ptr objectCloudDownSampled(new InputPointCloud);
        // pcl::PointCloud<pcl::Normal>::Ptr objectCloudNormal(
        //     new pcl::PointCloud<pcl::Normal>);
        PointCloudNormal::Ptr objectCloudPointNormal(
                new PointCloudNormal);
        pcl::PointCloud<pcl::PointXYZ>::Ptr boundaryCloudPCL(
                new pcl::PointCloud<pcl::PointXYZ>);

        geometry_msgs::PoseArray selected_grasp_poses;
        std::vector<double> grasp_utility;

        double utility_scale_to_marker_length, down_sample_radius;

        nh_->param("utility_scale_to_marker_length", utility_scale_to_marker_length,
                   0.1);
        nh_->param("down_sample_radius", down_sample_radius, 0.01);

        std::string input_frame = req.cloud.header.frame_id;
        pcl::fromROSMsg(req.cloud, *objectCloudlabelled);
        pcl::copyPointCloud(*objectCloudlabelled, *objectCloud);

        pcl::StatisticalOutlierRemoval<InputPointType> sor;
        sor.setMeanK(25);
        sor.setStddevMulThresh(1.0);
        sor.setInputCloud(objectCloud);
        sor.filter(*objectCloud);
        t2 = ros::Time::now();

        // Downsample object cloud
        objectCloudDownSampled =
                pcl_filters::downSample<InputPointType>(objectCloud, down_sample_radius);

        // Filter downsampled point cloud
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*objectCloudDownSampled, *objectCloudDownSampled,
                                     indices);

        // Check downsampled cloud has points
        if (objectCloudDownSampled->width > 0) {
                // ensure point cloud is in XYZ format
                PointCloudXYZ::Ptr objectCloudDownSampled_XYZ(new PointCloudXYZ);
                PointCloudXYZ::Ptr objectCloud_XYZ(new PointCloudXYZ);

                pcl::copyPointCloud(*objectCloudDownSampled, *objectCloudDownSampled_XYZ);
                pcl::copyPointCloud(*objectCloud, *objectCloud_XYZ);

                if (req.grasp_from_centroid) {
                        ROS_INFO("Detecting Grasp Candidates from Centroid");
                        graspPoseFromCentroid(objectCloud_XYZ, &selected_grasp_poses,
                                              &grasp_utility, input_frame, input_frame);
                } else {
                        ROS_INFO("Detecting Grasp Candidates from Surface");
                        graspPosesFromSurface(objectCloud_XYZ, objectCloudDownSampled_XYZ,
                                              boundaryCloudPCL, &selected_grasp_poses,
                                              &grasp_utility, input_frame);
                }

                if(selected_grasp_poses.poses.size() > 0){
                        selected_grasp_poses.header.frame_id = input_frame;
                        selected_grasp_poses.header.stamp = ros::Time::now();


                        publishClouds(boundaryCloudPCL, objectCloudDownSampled_XYZ, input_frame);
                        grasp_pub.publish(selected_grasp_poses);

                        std::vector<std::pair<geometry_msgs::Pose, double> > sorted_grasp_pairs =
                                sort_grasp_vectors(selected_grasp_poses.poses, grasp_utility);

                        res.grasp_candidates.grasp_poses.header.frame_id = input_frame;
                        res.grasp_candidates.grasp_poses.header.stamp = ros::Time::now();

                        for (unsigned int i = 0; i < sorted_grasp_pairs.size(); i++) {
                                res.grasp_candidates.grasp_poses.poses.push_back(
                                        sorted_grasp_pairs[i].first);
                                res.grasp_candidates.grasp_utilities.push_back(
                                        sorted_grasp_pairs[i].second);
                        }

                        publishGraspMarkers(res.grasp_candidates.grasp_poses, res.grasp_candidates.grasp_utilities,
                                    utility_scale_to_marker_length);
                }else{
                        ROS_INFO("No grasp candidates found");
                        return true;
                }


        } else {
                ROS_INFO("Not enough points in object");
                return true;
        }

        ROS_DEBUG_STREAM("[DetectGraspCandidates] Finished! Found "
                        << res.grasp_candidates.grasp_poses.poses.size()
                        << " Candidate Grasps");
        return true;
}

/////////////////////////////////////////////

int main(int argc, char **argv) {
        std::string topicObjectCloud, graspPosesTopic, boundariesTopic,
                    marker_array_topic;

        ros::init(argc, argv, "local_patch_grasp_node");

        nh_ = new ros::NodeHandle("~");

        tf_listener = new tf::TransformListener();

        nh_->param("object_cloud_topic", topicObjectCloud,
                   std::string("/grasp/points"));
        nh_->param("grasp_poses_topic", graspPosesTopic, std::string("/grasp_poses"));
        nh_->param("object_boundaries_topic", boundariesTopic,
                   std::string("/object_boundaries"));
        nh_->param("marker_array_topic", marker_array_topic,
                   std::string("/grasp_candidate_markers"));

        //      ros::Subscriber object_cloud_subscriber =
        //      nh_->subscribe(topicObjectCloud,
        //                                                               10,
        //    object_cloud_callback);

        ros::ServiceServer service = nh_->advertiseService(
                "/apc_grasping/detect_grasp_candidates", grasp_candidate_detection);

        //      ros::ServiceServer grasp_service =
        //          nh_.advertiseService("/apc_grasping/local_grasp_estimate",
        //          &grasp_estimation);

        grasp_pub = nh_->advertise<geometry_msgs::PoseArray>(graspPosesTopic, 0);

        boundaries_pub = nh_->advertise<sensor_msgs::PointCloud2>(boundariesTopic, 0);

        downsampled_pub =
                nh_->advertise<sensor_msgs::PointCloud2>("/object_downsampled", 0);

        marker_array_pub =
                nh_->advertise<visualization_msgs::MarkerArray>(marker_array_topic, 0);

        marker_pub =
                nh_->advertise<visualization_msgs::Marker>("/marker_array/delete", 0);

        ros::spin();

        return 0;
}
