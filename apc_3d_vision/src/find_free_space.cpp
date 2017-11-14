#include "ros/ros.h"
#include "apc_msgs/BoundingBoxDepth.h"
#include "apc_msgs/ReturnFreeCentroids.h"
#include <cmath>
#include <vector>

//
// #include <iostream>
// //PCL Libraries - SOMETHING WRONG WITH THE INCLUDES HERE
#include <pcl/common/io.h>  // pcl::copyPointCloud
#include <pcl/common/geometry.h>
#include <pcl/common/pca.h>
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
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>


#include <pcl/surface/mls.h>


std::vector<std::array<float, 6>> find_free_space(pcl::PointCloud<pcl::PointXYZRGB> &pc,
                                                  float top, float bottom, float left, float right,
                                                  float z_threshold, int depth)
{
  int max_depth = 3;
  std::vector<std::array<float, 6>> free_space;

  float avg_z = 0.0;
  float min_z = 100;
  int pts_abv_threshold = 0;
  int point_count = 0;

  for(auto &p: pc) {
    // if(p.r == 0) {
    //   continue;
    // }
    if((p.y < top) && (p.y > bottom) && (p.x < left) && (p.x > right)) {
      avg_z += p.z;
      if(p.z < min_z){
        min_z = p.z;
      }
      point_count ++;
      if(p.z < z_threshold) {
        pts_abv_threshold++;
      }
    }
  }
  avg_z /= point_count;
  std::cout << "Running at depth " << depth << std::endl;
  std::cout << point_count << ", " << avg_z << ", " << z_threshold << std::endl;

  if(point_count == 0) {
    return free_space;
  }

  if(pts_abv_threshold < 10) {
    std::cout << "Less than threshold, returning" << std::endl;
    std::array<float, 6> f = {top, bottom, left, right, avg_z, min_z};
    free_space.emplace_back(f);
  } else if(depth < max_depth) {
    std::cout << "GT Threshold, breaking into small parts" << std::endl;
    float y_mid = bottom + (top-bottom)/2;
    float x_mid = right + (left-right)/2;
    std::cout << top << ", " << y_mid << ", " << bottom << ", " << left << ", " << x_mid << ", " << right << std::endl;

    auto a = find_free_space(pc, top, y_mid, left, x_mid, z_threshold, depth + 1);
    free_space.insert(free_space.end(), a.begin(), a.end());

    auto b = find_free_space(pc, top, y_mid, x_mid, right, z_threshold, depth + 1);
    free_space.insert(free_space.end(), b.begin(), b.end());

    auto c = find_free_space(pc, y_mid, bottom, left, x_mid, z_threshold, depth + 1);
    free_space.insert(free_space.end(), c.begin(), c.end());

    auto d = find_free_space(pc, y_mid, bottom, x_mid, right, z_threshold, depth + 1);
    free_space.insert(free_space.end(), d.begin(), d.end());

  }

  return free_space;

}

// Function that provides the service for adding two ints, takes request and response
// type defined in the srv file and returns a boolean
bool add(apc_msgs::ReturnFreeCentroids::Request &req,
        apc_msgs::ReturnFreeCentroidsResponse &res)
{
  // Load Message (does it need to be XYZRGB or just XYZ)
  pcl::PointCloud<pcl::PointXYZRGB> tempCloud;
  pcl::PointCloud<pcl::PointXYZRGB> storageCloud;
  pcl::fromROSMsg(req.cloudSearch, tempCloud);
  std::vector<int> index;   // The mapping (ordered): cloud_out.points[i] = cloud_in.points[index[i]]
  tempCloud.is_dense = false;
  pcl::removeNaNFromPointCloud(tempCloud, tempCloud, index);

  pcl::PointXYZRGB lowerBound;
  pcl::PointXYZRGB upperBound;
  pcl::getMinMax3D(tempCloud, lowerBound, upperBound);

  // Cut a couple of cm off the edges.
  float border = 0.03;
  for(auto &p: tempCloud) {
    if( p.x < (upperBound.x - border) && p.x > (lowerBound.x + border) &&
        p.y < (upperBound.y - border) && p.y > (lowerBound.y + border)
      ) {
        storageCloud.push_back(p);
      }
  }

  pcl::getMinMax3D(storageCloud, lowerBound, upperBound);

  float storage_bottom = 0.8;
  float storage_top = storage_bottom - 0.19;
  float z_threshold = storage_top + 0.10;

  auto free_space = find_free_space(storageCloud, upperBound.y, lowerBound.y, upperBound.x, lowerBound.x, z_threshold, 0);

  res.success.data = true;
  apc_msgs::BoundingBoxDepth bbmsg;
  for (int i = 0; i < free_space.size(); i++){
    bbmsg.top_left.x = free_space[i][2];
    bbmsg.top_left.y = free_space[i][0];
    bbmsg.bottom_right.x = free_space[i][3];
    bbmsg.bottom_right.y = free_space[i][1];
    bbmsg.z_avg.data = free_space[i][4];
    bbmsg.z_min.data = free_space[i][5];

    res.bounding_boxes.push_back(bbmsg);
  }

  // int numPoints = storageCloud.size();
  // float gridSize = 0.05;                         //Length of grid sides
  // int numGridsX =  floor( xSizeABS / gridSize);
  // int numGridsy =  floor( ySizeABS / gridSize);
  // int numGrids = numGridsX * numGridsy;
  // std::cout << "NuMGrids: " << numGrids << std::endl;
  // int pointsPerGrid = numPoints / numGrids;
  // //int pointsPerGridMin = pointsPerGrid * 0.1;
  // int pointsPerGridMin = (gridSize/euclidDist)*(gridSize/euclidDist)*0.65;
  //
  // std::cout << numPoints << ", " << pointsPerGrid << ", " << pointsPerGridMin << std::endl;
  //
  // // Create vector of x-y locations
  // // std::vector<std::array<float, 2>> centroidLocationsVector;
  // std::vector<std::vector<float>> centroidLocationsVector;
  // float halfSize = gridSize * 0.5;
  // for(int i = 0; i < numGridsX; i++){
  //   for(int j = 0; j < numGridsX; j++){
  //     centroidLocationsVector.push_back({lowerBound.x + gridSize * i + halfSize, lowerBound.y + gridSize * j + halfSize});
  //   }
  // }
  //
  // //Search for valid locations
  // std::vector<std::vector<float>> validCentroidLocationsVector;
  // int pointCount;
  // for(int i = 0; i < centroidLocationsVector.size(); i++){
  //   pointCount = 0;
  //   for(int j = 0; j < numPoints; j++){
  //     if(storageCloud.points[index[j]].x > (centroidLocationsVector[i][0] - halfSize) &&
  //        storageCloud.points[index[j]].x < (centroidLocationsVector[i][0] + halfSize) &&
  //        storageCloud.points[index[j]].y > (centroidLocationsVector[i][1] - halfSize) &&
  //        storageCloud.points[index[j]].y < (centroidLocationsVector[i][1] + halfSize) &&
  //        storageCloud.points[index[j]].r > 0)
  //     {
  //       pointCount++;
  //     }
  //   }
  //   std::cout << centroidLocationsVector[i][0] << ", " << centroidLocationsVector[i][1] << std::endl;
  //   std::cout << pointCount << std::endl;
  //   if(pointCount >= pointsPerGridMin){
  //     validCentroidLocationsVector.push_back(centroidLocationsVector[i]); //HOW DO I REFERENCE THIS BETTER?
  //   }
  // }
  //
  // //Push valid locations into message
  // geometry_msgs::Vector3 storageVector;
  // res.success.data = true;
  // for (int i = 0; i < validCentroidLocationsVector.size(); i++){
  //   storageVector.x = validCentroidLocationsVector[i][0];
  //   storageVector.y = validCentroidLocationsVector[i][1];
  //   storageVector.z = 0;
  //   res.validCentroids.push_back(storageVector);
  // }

  return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "find_free_space");
  ros::NodeHandle n;

  // Service created and advertised over ROS
  ros::ServiceServer service = n.advertiseService("/apc_3d_vision/find_free_space", add);
  ROS_INFO("Ready to accept point cloud.");
  ros::spin();

  return 0;
}
