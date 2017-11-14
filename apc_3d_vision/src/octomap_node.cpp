#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>

#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/octree/octree_pointcloud_occupancy.h>
#include <pcl_conversions/pcl_conversions.h>
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

#include <pcl_ros/point_cloud.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

/* Messages */
#include "apc_msgs/BoundingBoxDepth.h"

/* Services */
#include "apc_msgs/ObjectPlacementPoseFromCloud.h"

/*
NOTE
Custom service definitions should have the same name as the service that employs
them. I.E. the find_free_space service should have a custom service definition
called FindFreeSpace.
*/

// TODO convert to non-colour pointcloud at start
// TODO update draw_object function to operate on position and angle


bool does_object_intersect(pcl::octree::OctreePointCloudOccupancy<pcl::PointXYZRGB> &tree,
                           pcl::PointXYZ search_origin, pcl::PointXYZ object_dimensions,
                           double resolution) {
   for (double z=search_origin.z; z>search_origin.z-object_dimensions.z; z-=resolution) {
       for (double x=search_origin.x; x<search_origin.x+object_dimensions.x; x+=resolution) {
           for (double y=search_origin.y; y<search_origin.y+object_dimensions.y; y+=resolution) {
               if (tree.isVoxelOccupiedAtPoint(x, y, z)) {
                   return true;
               }
           }
       }
   }
   return false;
}

int count_vacancies_under_object(pcl::octree::OctreePointCloudOccupancy<pcl::PointXYZRGB> &tree,
                                 pcl::PointXYZRGB max_pt,
                                 pcl::PointXYZ object_origin,
                                 pcl::PointXYZ object_dimensions,
                                 double resolution) {
    int num_vacancies = 0;
    for (double z=max_pt.z; z>object_origin.z; z-=resolution) {
        for (double x=object_origin.x; x<object_origin.x+object_dimensions.x; x+=resolution) {
            for (double y=object_origin.y; y<object_origin.y+object_dimensions.y; y+=resolution) {
                if (!tree.isVoxelOccupiedAtPoint(x, y, z)) {
                    num_vacancies++;
                }
            }
        }
    }
    return num_vacancies;
}

bool get_valid_object_position(pcl::octree::OctreePointCloudOccupancy<pcl::PointXYZRGB> &tree,
                               pcl::PointXYZ object_dimensions,
                               pcl::PointXYZRGB min_pt,
                               pcl::PointXYZRGB max_pt,
                               double resolution,
                               pcl::PointXYZ &ret_valid_object_position) {
    // Do a bottom of tote up search for a position where the whole object fits
    pcl::PointXYZ search_origin;
    bool is_finished = false;
    for (double z=max_pt.z; z>min_pt.z+object_dimensions.z; z-=resolution) {
    // for (double z=min_pt.z; z<max_pt.z-object_dimensions.z; z+=resolution) {
        for (double x=min_pt.x; x<max_pt.x-object_dimensions.x; x+=resolution) {
            for (double y=min_pt.y; y<max_pt.y-object_dimensions.y; y+=resolution) {
                if (!tree.isVoxelOccupiedAtPoint(x, y, z)) {
                    search_origin.x = x;
                    search_origin.y = y;
                    search_origin.z = z;
                    if (!does_object_intersect(tree, search_origin, object_dimensions, resolution)) {
                        ROS_INFO_STREAM("Place origin of object at: " << search_origin.x << ", " << search_origin.y << ", " << search_origin.z);
                        is_finished = true;
                    }
                    // else {
                    //     ROS_INFO_STREAM("Object doesn't fit here. Continuing search...");
                    // }
                }
                if (is_finished) {break;}
            }
            if (is_finished) {break;}
        }
        if (is_finished) {break;}
    }

    if (!is_finished) {
        return false;
    }

    pcl::PointXYZ object_origin = search_origin;
    pcl::PointXYZ best_object_origin;

    // Trial all x,y positions at found depth to choose the spot with the least number of vacancies below
    // This was added to promote placing items on other items vs. over vacancies where smaller items might fit better
    int num_vacancies = 0;
    int min_vacancies = 1000000;
    for (double x=min_pt.x; x<max_pt.x-object_dimensions.x; x+=resolution) {
        for (double y=min_pt.y; y<max_pt.y-object_dimensions.y; y+=resolution) {
            object_origin.x = x;
            object_origin.y = y;
            if (!does_object_intersect(tree, object_origin, object_dimensions, resolution)) {
                num_vacancies = count_vacancies_under_object(tree, max_pt, object_origin, object_dimensions, resolution);

                if (num_vacancies < min_vacancies) {
                    min_vacancies = num_vacancies;
                    best_object_origin.x = x;
                    best_object_origin.y = y;
                    best_object_origin.z = object_origin.z;
                }
            }
        }
    }

    ret_valid_object_position = best_object_origin;

    return true;
}

void fill_under_objects(pcl::octree::OctreePointCloudOccupancy<pcl::PointXYZRGB> &tree,
                        pcl::PointXYZRGB min_pt, pcl::PointXYZRGB max_pt,
                        double resolution) {
    pcl::octree::OctreePointCloud<pcl::PointXYZRGB>::AlignedPointTVector occupiedCells;
    tree.getOccupiedVoxelCenters(occupiedCells);
    pcl::octree::OctreePointCloud<pcl::PointXYZRGB>::AlignedPointTVector::iterator it;

    for (it = occupiedCells.begin(); it != occupiedCells.end(); ++it) {
        for (double z=max_pt.z; z>it->z; z-=resolution) {
            pcl::PointXYZRGB point;
            point.x = it->x;
            point.y = it->y;
            point.z = z;
            tree.setOccupiedVoxelAtPoint(point);
        }
    }

    // Fill roof too to stop items being placed above
    double z = max_pt.z - 0.23;  // Storage is about 20cm high
    for (double x=min_pt.x; x<max_pt.x; x+=resolution) {
        for (double y=min_pt.y; y<max_pt.y; y+=resolution) {
            pcl::PointXYZRGB point;
            point.x = x;
            point.y = y;
            point.z = z;
            tree.setOccupiedVoxelAtPoint(point);
        }
    }

    // Fill ~2cm of walls too
    // tl, tr, bl, br
    double top_left_x = max_pt.x;
    double top_left_y = min_pt.y;
    double top_right_x = max_pt.x;
    double top_right_y = max_pt.y;
    double bottom_left_x = min_pt.x;
    double bottom_left_y = min_pt.y;
    double bottom_right_x = min_pt.x;
    double bottom_right_y = max_pt.y;
    double bottom_z = max_pt.z;
    double top_z = bottom_z - 0.23;
    double wall_width = 0.03;

    // ROS_INFO_STREAM("top_left_x = " << top_left_x);
    // ROS_INFO_STREAM("top_left_y = " << top_left_y);
    // ROS_INFO_STREAM("top_right_x = " << top_right_x);
    // ROS_INFO_STREAM("top_right_y = " << top_right_y);
    // ROS_INFO_STREAM("bottom_left_x = " << bottom_left_x);
    // ROS_INFO_STREAM("bottom_left_y = " << bottom_left_y);
    // ROS_INFO_STREAM("bottom_right_x = " << bottom_right_x);
    // ROS_INFO_STREAM("bottom_right_y = " << bottom_right_y);
    // ROS_INFO_STREAM("bottom_z = " << bottom_z);
    // ROS_INFO_STREAM("top_z = " << top_z);

    for (double z=bottom_z; z>top_z; z-=resolution) {
        // Fill top row
        for (double y=top_left_y; y<top_right_y; y+=resolution) {
            for (double x=top_left_x; x>top_left_x-wall_width; x-=resolution) {
                // ROS_INFO_STREAM("z = " << z << ", x = " << x << ", y = " << y);
                pcl::PointXYZRGB point;
                point.x = x;
                point.y = y;
                point.z = z;
                tree.setOccupiedVoxelAtPoint(point);
            }
        }
        // Fill bottom row
        for (double y=bottom_left_y; y<bottom_right_y; y+=resolution) {
            for (double x=bottom_left_x; x<bottom_left_x+wall_width; x+=resolution) {
                // ROS_INFO_STREAM("z = " << z << ", x = " << x << ", y = " << y);
                pcl::PointXYZRGB point;
                point.x = x;
                point.y = y;
                point.z = z;
                tree.setOccupiedVoxelAtPoint(point);
            }
        }
        // Fill left column
        for (double x=bottom_left_x; x<top_left_x; x+=resolution) {
            for (double y=bottom_left_y; y<bottom_left_y+wall_width; y+=resolution) {
                // ROS_INFO_STREAM("z = " << z << ", x = " << x << ", y = " << y);
                pcl::PointXYZRGB point;
                point.x = x;
                point.y = y;
                point.z = z;
                tree.setOccupiedVoxelAtPoint(point);
            }
        }
        // Fill right column
        for (double x=bottom_right_x; x<top_right_x; x+=resolution) {
            for (double y=bottom_right_y; y>bottom_right_y-wall_width; y-=resolution) {
                // ROS_INFO_STREAM("z = " << z << ", x = " << x << ", y = " << y);
                pcl::PointXYZRGB point;
                point.x = x;
                point.y = y;
                point.z = z;
                tree.setOccupiedVoxelAtPoint(point);
            }
        }
    }
}

void draw_object(pcl::octree::OctreePointCloudOccupancy<pcl::PointXYZRGB> &tree,
                 pcl::PointXYZ object_origin, pcl::PointXYZ object_dimensions,
                 double resolution) {
   for (double z=object_origin.z; z>object_origin.z-object_dimensions.z; z-=resolution) {
       for (double x=object_origin.x; x<object_origin.x+object_dimensions.x; x+=resolution) {
           for (double y=object_origin.y; y<object_origin.y+object_dimensions.y; y+=resolution) {
               pcl::PointXYZRGB point;
               point.x = x;
               point.y = y;
               point.z = z;
               tree.setOccupiedVoxelAtPoint(point);
           }
       }
   }
}

void save_pointcloud_version_of_octree(pcl::octree::OctreePointCloudOccupancy<pcl::PointXYZRGB> &tree,
                                       std::string filename) {
    //how many occupied cells do we have in the tree?
    pcl::octree::OctreePointCloud<pcl::PointXYZRGB>::AlignedPointTVector occupiedCells;
    tree.getOccupiedVoxelCenters(occupiedCells);

    //cloud to store the points
    pcl::PointCloud<pcl::PointXYZ> cloud;
    ROS_INFO_STREAM("num_occupied_cells = " << occupiedCells.size());
    // cloud.points.resize(occupiedCells.size());
    // cloud.width = 256426;
    cloud.width = occupiedCells.size();
    cloud.height = 1;
    cloud.points.resize(cloud.width * cloud.height);
    // ROS_INFO_STREAM("cloud width = " << cloud.width);
    // ROS_INFO_STREAM("cloud height = " << cloud.height);

    pcl::octree::OctreePointCloud<pcl::PointXYZRGB>::AlignedPointTVector::iterator it;
    int i=0;
    for (it = occupiedCells.begin(); it != occupiedCells.end(); ++it, i++)
    {
        //add point in point cloud
        cloud.points[i].x = it->x;
        cloud.points[i].y = it->y;
        cloud.points[i].z = it->z;
    }
    //save cloud
    pcl::io::savePCDFileASCII(filename, cloud);
}

bool get_valid_object_position_and_orientation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                                               double object_longest_side_length,
                                               double object_middlest_side_length,
                                               double object_shortest_side_length,
                                               double resolution,
                                               bool verbose,
                                               pcl::PointXYZ &ret_valid_object_position,
                                               double &ret_valid_object_orientation) {
    pcl::octree::OctreePointCloudOccupancy<pcl::PointXYZRGB> pcl_tree (resolution);

    pcl_tree.setOccupiedVoxelsAtPointsFromCloud(cloud);

    pcl::octree::OctreePointCloud<pcl::PointXYZRGB>::AlignedPointTVector occupiedCells;
    pcl_tree.getOccupiedVoxelCenters(occupiedCells);
    pcl::octree::OctreePointCloud<pcl::PointXYZRGB>::AlignedPointTVector::iterator it;

    std::vector<int> index;
    cloud->is_dense = false;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, index);

    pcl::PointXYZRGB min_pt, max_pt;
    pcl::getMinMax3D(*cloud, min_pt, max_pt);

    ROS_INFO_STREAM("min_x = " << min_pt.x << ", min_y" << min_pt.y << ", min_z" << min_pt.z);
    ROS_INFO_STREAM("max_x = " << max_pt.x << ", max_y" << max_pt.y << ", max_z" << max_pt.z);

    fill_under_objects(pcl_tree, min_pt, max_pt, resolution);

    if (verbose) {
        save_pointcloud_version_of_octree(pcl_tree, "octree_without_object.pcd");
    }

    pcl::PointXYZ object_dimensions_0;
    double degrees_to_global_y_0 = 90;
    object_dimensions_0.x = object_longest_side_length;
    object_dimensions_0.y = object_middlest_side_length;
    object_dimensions_0.z = object_shortest_side_length;
    pcl::PointXYZ object_dimensions_1;  // flipped width and height
    double degrees_to_global_y_1 = 0;
    object_dimensions_1.x = object_middlest_side_length;
    object_dimensions_1.y = object_longest_side_length;
    object_dimensions_1.z = object_shortest_side_length;

    pcl::PointXYZ object_dimensions;

    // Choose which z position is largest (i.e. furthest from camera / closest to floor)
    pcl::PointXYZ valid_position, valid_position_0, valid_position_1;
    bool success_0 = get_valid_object_position(pcl_tree, object_dimensions_0, min_pt, max_pt, resolution, valid_position_0);
    bool success_1 = get_valid_object_position(pcl_tree, object_dimensions_1, min_pt, max_pt, resolution, valid_position_1);

    if (success_0 && success_1) {
        // Choose which z position is largest (i.e. furthest from camera / closest to floor)
        if (valid_position_0.z > valid_position_1.z) {
            valid_position = valid_position_0;
            object_dimensions = object_dimensions_0;

            ret_valid_object_position.x = valid_position.x + object_longest_side_length/2.0;
            ret_valid_object_position.y = valid_position.y + object_middlest_side_length/2.0;
            ret_valid_object_position.z = valid_position.z - object_shortest_side_length;
            ret_valid_object_orientation = degrees_to_global_y_0;
        } else {
            valid_position = valid_position_1;
            object_dimensions = object_dimensions_1;

            ret_valid_object_position.x = valid_position.x + object_middlest_side_length/2.0;
            ret_valid_object_position.y = valid_position.y + object_longest_side_length/2.0;
            ret_valid_object_position.z = valid_position.z - object_shortest_side_length;
            ret_valid_object_orientation = degrees_to_global_y_1;
        }
    } else if (success_0) {
        valid_position = valid_position_0;
        object_dimensions = object_dimensions_0;

        ret_valid_object_position.x = valid_position.x + object_longest_side_length/2.0;
        ret_valid_object_position.y = valid_position.y + object_middlest_side_length/2.0;
        ret_valid_object_position.z = valid_position.z - object_shortest_side_length;
        ret_valid_object_orientation = degrees_to_global_y_0;
    } else if (success_1) {
        valid_position = valid_position_1;
        object_dimensions = object_dimensions_1;

        ret_valid_object_position.x = valid_position.x + object_middlest_side_length/2.0;
        ret_valid_object_position.y = valid_position.y + object_longest_side_length/2.0;
        ret_valid_object_position.z = valid_position.z - object_shortest_side_length;
        ret_valid_object_orientation = degrees_to_global_y_1;
    } else {
        return false;
    }

    if (verbose) {
        pcl::octree::OctreePointCloudOccupancy<pcl::PointXYZRGB> pcl_tree_empty (resolution);
        draw_object(pcl_tree_empty, valid_position, object_dimensions, resolution);
        save_pointcloud_version_of_octree(pcl_tree_empty, "object_in_valid_position.pcd");
    }

    return true;
}

bool object_placement_pose_from_cloud_service_callback(apc_msgs::ObjectPlacementPoseFromCloud::Request &req,
                                                       apc_msgs::ObjectPlacementPoseFromCloud::Response &res) {
   boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud;
   cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

   double object_longest_side_length = req.object_longest_side_length.data;
   double object_middlest_side_length = req.object_middlest_side_length.data;
   double object_shortest_side_length = req.object_shortest_side_length.data;
   double resolution = req.resolution.data;

   Eigen::Affine3d camera_to_world_tf_eigen;
   tf::poseMsgToEigen(req.camera_to_world, camera_to_world_tf_eigen);

   pcl::fromROSMsg(req.cropped_cloud, *cloud);

   pcl::transformPointCloud(*cloud, *cloud, camera_to_world_tf_eigen);

   pcl::PointXYZ valid_object_position;
   double valid_object_orientation;
   bool verbose = true;

   bool success = get_valid_object_position_and_orientation(cloud,
                                                            object_longest_side_length,
                                                            object_middlest_side_length,
                                                            object_shortest_side_length,
                                                            resolution,
                                                            verbose,
                                                            valid_object_position,
                                                            valid_object_orientation);

   res.x.data = valid_object_position.x;
   res.y.data = valid_object_position.y;
   res.z.data = valid_object_position.z;
   res.degrees_to_global_y.data = valid_object_orientation;
   res.success.data = success;

   ROS_INFO_STREAM("Valid position = x: " << valid_object_position.x << ", y: " << valid_object_position.y << ", z: " << valid_object_position.z);
   ROS_INFO_STREAM("Valid orientation to global y = " << valid_object_orientation << " degrees");

   return true;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "object_placement_pose_from_cloud_node");
    ros::NodeHandle n;

    ros::ServiceServer service_handle = n.advertiseService("object_placement_pose_from_cloud", object_placement_pose_from_cloud_service_callback);

    ros::spin();

    return 0;
}
