/*
   Copyright 2016 Australian Centre for Robotic Vision
 */
#include <tf/transform_listener.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_lib/move_robot_named.h>
#include <moveit_msgs/PlanningScene.h>

#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

#include <math.h>
#include <algorithm>

// #include <apc_grasping/Item.h>
#include <apc_msgs/GraspPose.h>
#include <apc_msgs/SelectGraspFromCandidates.h>
#include <apc_msgs/SelectGraspFromModel.h>
#include <moveit_lib/move_robot_pose.h>

#define TIMING
#ifdef TIMING
#include <ctime>
#define TICK(n) clock_t __start_##n = clock()
#define TOCK(n)                                                                \
  do {                                                                         \
    clock_t __end_##n = clock();                                               \
    double duration =                                                          \
        1000.0 * (double)(__end_##n - __start_##n) / (double)CLOCKS_PER_SEC;   \
    static double min = duration;                                              \
    if (duration < min)                                                        \
      min = duration;                                                          \
    static double max = duration;                                              \
    if (duration > max)                                                        \
      max = duration;                                                          \
    static double sum = 0;                                                     \
    sum += duration;                                                           \
    static double count = 0;                                                   \
    count += 1.0;                                                              \
    std::cout << "Timer " #n << " [" << duration << "," << min << ","          \
              << sum / count << "," << max << "] ms [meas,min,avg,max]"        \
              << std::endl;                                                    \
  } while (0)
#else
#define TICK(n)
#define TOCK(n)
#endif

typedef enum indexes {
  UNIT_X_IDX = 0,
  UNIT_Y_IDX = 1,
  UNIT_Z_IDX = 2,
} UNIT_AXIS_INDEX;

typedef struct grasps_info {
  geometry_msgs::Pose pose;
  std::string move_group_name;
} grasp_info;

planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
robot_model::RobotModelPtr kinematic_model;
robot_state::RobotStatePtr robot_state_;

std::string bin_name;

std::string which_arm;

tf::Transform bin_to_base_transform, grasp_transform, base_to_bin_transform;

ros::Publisher grasp_pub, marker_array_pub, marker_pub, pose_array_pub;

tf::TransformListener *tf_listener;
ros::NodeHandle *nh;

bool vertical_only;

bool transformPose(const geometry_msgs::PoseStamped &input_pose,
                   std::string target_frame_id,
                   geometry_msgs::PoseStamped *transformed_pose) {
  bool tf_success =
      tf_listener->waitForTransform(target_frame_id, input_pose.header.frame_id,
                                    ros::Time(0), ros::Duration(1.0));
  if (tf_success) {
    tf_listener->getLatestCommonTime(input_pose.header.frame_id,
                                     target_frame_id,
                                     transformed_pose->header.stamp, NULL);

    tf_listener->transformPose(target_frame_id, input_pose, *transformed_pose);
    return true;
  } else {
    ROS_INFO("Can't Find Transform from %s to %s!",
             input_pose.header.frame_id.c_str(), target_frame_id.c_str());
    return false;
  }
}

void transformPoseNoLookup(const geometry_msgs::Pose &input_pose,
                           const tf::Transform &transform,
                           geometry_msgs::Pose *transformed_pose) {
  Eigen::Affine3d input_pose_eigen, transform_eigen, transformed_pose_eigen;

  tf::transformTFToEigen(transform, transform_eigen);
  tf::poseMsgToEigen(input_pose, input_pose_eigen);

  transformed_pose_eigen = transform_eigen * input_pose_eigen;

  tf::poseEigenToMsg(transformed_pose_eigen, *transformed_pose);

  // THIS IS A HACK THAT WE DON'T KNOW HOW TO FIX
  // transformed_pose->orientation.x = -1 * transformed_pose->orientation.x;
  // transformed_pose->orientation.y = -1 * transformed_pose->orientation.y;
  // transformed_pose->orientation.z = -1 * transformed_pose->orientation.z;
  // transformed_pose->orientation.w = -1 * transformed_pose->orientation.w;
}

// void transformPoseNoLookup(const std::string &target_frame,
//                            const tf::Stamped<tf::Pose> &stamped_in,
//                            tf::Stamped<tf::Pose> *stamped_out,
//                            const tf::StampedTransform &transform) {
//     stamped_out->setData(transform * stamped_in);
//     stamped_out->header.stamp_ = transform.header.stamp_;
//     stamped_out->header.frame_id = target_frame;
// }

bool lookupTransform(std::string target_frame, std::string source_frame,
                     tf::Transform *output_transform) {
  // tf::Transform transform;
  tf::StampedTransform stampedtransform;
  bool tf_success = tf_listener->waitForTransform(
      target_frame, source_frame, ros::Time(0), ros::Duration(1.0));
  if (tf_success) {
    tf_listener->lookupTransform(target_frame, source_frame, ros::Time(0),
                                 stampedtransform);
    output_transform->setOrigin(stampedtransform.getOrigin());
    output_transform->setRotation(stampedtransform.getRotation());

    return true;
  } else {
    ROS_INFO("Can't Find Transform from %s to %s!", source_frame.c_str(),
             target_frame.c_str());
    return false;
  }
}

std::vector<std::pair<grasp_info, double>>
sort_grasp_vectors(std::vector<grasp_info> &grasps,
                   std::vector<double> &utilities) {
  // initialize original index locations
  std::vector<std::pair<grasp_info, double>> grasp_score_pairs;
  for (unsigned int i = 0; i < grasps.size(); i++) {
    std::pair<grasp_info, double> pair(grasps[i], utilities[i]);
    grasp_score_pairs.push_back(pair);
  }

  std::sort(grasp_score_pairs.begin(), grasp_score_pairs.end(),
            boost::bind(&std::pair<grasp_info, double>::second, _1) >
                boost::bind(&std::pair<grasp_info, double>::second, _2));

  return grasp_score_pairs;
}

std::vector<std::pair<apc_msgs::GraspPose, double>>
sort_grasp_msg_vectors(std::vector<apc_msgs::GraspPose> &grasps,
                       std::vector<double> &utilities) {
  // initialize original index locations
  std::vector<std::pair<apc_msgs::GraspPose, double>> grasp_score_pairs;
  for (unsigned int i = 0; i < grasps.size(); i++) {
    std::pair<apc_msgs::GraspPose, double> pair(grasps[i], utilities[i]);
    grasp_score_pairs.push_back(pair);
  }

  std::sort(
      grasp_score_pairs.begin(), grasp_score_pairs.end(),
      boost::bind(&std::pair<apc_msgs::GraspPose, double>::second, _1) >
          boost::bind(&std::pair<apc_msgs::GraspPose, double>::second, _2));

  return grasp_score_pairs;
}

double getPositionalWeighting(const geometry_msgs::Pose &current_grasp,
                              std::string bin_name) {
  // ros::NodeHandle nh;

  std::string planning_frame;
  tf::StampedTransform current_grasp_transform;
  geometry_msgs::Pose current_grasp_bin;
  double bin_width, bin_height;
  double grasp_x_position, grasp_y_position, x_distance_from_center,
      y_distance_from_center, min_distance_shelf, norm_min_distance_shelf;
  double center_x_position, center_y_position;

  nh->param("/shelf_layout/" + bin_name + "/bin_width", bin_width, 0.0);
  nh->param("/shelf_layout/" + bin_name + "/bin_height", bin_height, 0.0);
  nh->param("planning_frame", planning_frame, std::string("base"));

  ROS_INFO_STREAM("Bin width and height = " << bin_width << ", " << bin_height);

  center_x_position = bin_height / 2.0;
  // y axis is negative (points left)
  center_y_position = -bin_width / 2.0;

  transformPoseNoLookup(current_grasp, base_to_bin_transform,
                        &current_grasp_bin);

  grasp_x_position = current_grasp_bin.position.x;
  grasp_y_position = current_grasp_bin.position.y;

  std::cout << "GraspVariables: x: " << grasp_x_position << " y "
            << grasp_y_position << std::endl;

  x_distance_from_center = std::abs(grasp_x_position - center_x_position);
  y_distance_from_center = std::abs(grasp_y_position - center_y_position);

  std::cout << "x_distance_from_center:" << x_distance_from_center << std::endl;
  std::cout << "y_distance_from_center:" << y_distance_from_center << std::endl;

  // max from the centre is the minimum of shelf
  // TODO maybe do this per axis?
  min_distance_shelf = std::max(x_distance_from_center, y_distance_from_center);

  std::cout << "min_distance_shelf:" << min_distance_shelf << ".."
            << std::max(x_distance_from_center, y_distance_from_center)
            << std::endl;
  std::cout << "half max shelf:" << (std::max(bin_width, bin_height) / 2)
            << std::endl;

  // Normalise
  norm_min_distance_shelf =
      min_distance_shelf / (std::max(bin_width, bin_height) / 2);

  std::cout << "norm min shelf:" << norm_min_distance_shelf << std::endl;

  // distance_from_center =
  //   sqrt(pow(x_distance_from_center, 2) + pow(y_distance_from_center,
  //   2));

  // Normalise
  // distance_from_center = distance_from_center /
  // (sqrt(pow(center_x_position, 2) + pow(center_y_position, 2)));

  // tf_listener.lookupTransform(bin_name, current_grasp.frame_id,
  //   ros::Time::now(), current_grasp_transform);
  // }
  return (1.0 - norm_min_distance_shelf);
}

geometry_msgs::Pose rotate_pose(geometry_msgs::Pose input_pose,
                                double angle_offset,
                                Eigen::Vector3d unit_axis) {
  Eigen::Affine3d input_pose_eigen, input_pose_eigen_rotated;
  geometry_msgs::Pose output_pose;
  tf::poseMsgToEigen(input_pose, input_pose_eigen);

  Eigen::Affine3d rotation(Eigen::AngleAxisd(angle_offset, unit_axis));

  input_pose_eigen_rotated = input_pose_eigen * rotation;

  tf::poseEigenToMsg(input_pose_eigen_rotated, output_pose);

  return output_pose;
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


// Better way to do it woould be to find the angle between the grasp pose and
// the optimal grasp vectors
double angle_between_pose(geometry_msgs::Pose grasp_pose,
                          uint8_t grasp_axis_idx, uint8_t base_axis_idx) {
  Eigen::Affine3d grasp_pose_eigen;
  tf::poseMsgToEigen(grasp_pose, grasp_pose_eigen);

  Eigen::Vector3d grasp_unit_axis =
      grasp_pose_eigen.rotation().col(grasp_axis_idx);

  // angle = acos (u.v / norm(u).norm(v) )
  // dot product of vector and unit vector is just the index of the vector
  // u.x = u[0], u.y = u[1], u.z = u[2]
  // norm(u).norm(z) = norm(u)*1 = norm(u)
  return acos(grasp_unit_axis[base_axis_idx] / grasp_unit_axis.norm());
}
//
// void flipOrientationTowardsShelf(geometry_msgs::Pose grasp_pose, ) {
//     geometry_msgs::Pose shelf_grasp_pose;
//     //
//     transformPoseNoLookup(grasp_pose, bin_to_base_transform,
//     &shelf_grasp_pose);
//     if (angle_between_pose(shelf_grasp_pose, UNIT_X_IDX, UNIT_Z_IDX) >  {
//     }
// }

// ROS_INFO_STREAM("Found Reachable Pose at: " << rotated_pose);
bool checkCollision(robot_state::RobotStatePtr kinematic_state,
                    planning_scene::PlanningScenePtr planning_scene,
                    std::string group_name) {
  collision_detection::CollisionRequest collision_request;
  collision_request.group_name = group_name;
  collision_request.contacts = true;
  collision_detection::CollisionResult collision_result;
  collision_result.clear();

  planning_scene->checkCollision(collision_request, collision_result,
                                 *kinematic_state);
  return collision_result.collision;
}

bool isStateValid(
    const planning_scene::PlanningScene *planning_scene,
    const kinematic_constraints::KinematicConstraintSet *constraint_set,
    robot_state::RobotState *state, const robot_state::JointModelGroup *group,
    const double *ik_solution) {
  state->setJointGroupPositions(group, ik_solution);
  state->update();
  return (!planning_scene ||
          !planning_scene->isStateColliding(*state, group->getName())) &&
         (!constraint_set || constraint_set->decide(*state).satisfied);
}

bool isStateValidSimple(const planning_scene::PlanningScene *planning_scene,
                        robot_state::RobotState *state,
                        const robot_state::JointModelGroup *group,
                        const double *ik_solution) {
  state->setJointGroupPositions(group, ik_solution);
  state->update();
  // return (!planning_scene ||
  //         !planning_scene->isStateColliding(*state, group->getName()));
  return !planning_scene->isStateColliding(*state, group->getName());
}

bool checkIKandCollision(const robot_state::JointModelGroup *jmg,
                         robot_state::RobotStatePtr state,
                         geometry_msgs::Pose *pose, int attempts) {
  kinematics::KinematicsQueryOptions options;

  boost::scoped_ptr<planning_scene_monitor::LockedPlanningSceneRO> ls;
  ls.reset(new planning_scene_monitor::LockedPlanningSceneRO(
      planning_scene_monitor_));

  std::string link_name = jmg->getLinkModelNames().back();

  const moveit::core::GroupStateValidityCallbackFn validity_fn = boost::bind(
      &isStateValidSimple,
      static_cast<const planning_scene::PlanningSceneConstPtr &>(*ls).get(), _1,
      _2, _3);

  // Can add tip link frame using RobotState.getLinkModel()
  bool found_ik =
      state->setFromIK(jmg, *pose, link_name, attempts, 0.0, validity_fn);

  // ROS_INFO_STREAM("Link Name = " << link_name);
  const Eigen::Affine3d &end_effector_pose =
      state->getGlobalLinkTransform(link_name);

  tf::poseEigenToMsg(end_effector_pose, *pose);

  return found_ik;
}

bool checkIK(const robot_state::JointModelGroup *jmg,
             robot_state::RobotStatePtr state, geometry_msgs::Pose pose,
             int attempts) {
  // Can add tip link frame using RobotState.getLinkModel()
  // std::string link_name = jmg->getLinkModelNames().back();
  bool ik = state->setFromIK(jmg, pose, attempts, 0.1);
  return (ik);
}

double updateUtility(geometry_msgs::Pose pose, double grasp_utility,
                     std::string group_name) {
  double positional_score, rotational_score, move_group_score;
  double angle_between_x_and_z, angle_between_x_and_y;

  double move_group_weighting, positional_weighting, rotational_weighting,
      gravity_weighting;

  if (!nh->getParam("move_group_weighting", move_group_weighting))
    move_group_weighting = 0.3;
  if (!nh->getParam("rotational_weighting", rotational_weighting))
    rotational_weighting = 0.2;
  if (!nh->getParam("positional_weighting", positional_weighting))
    positional_weighting = 0.1;
  if (!nh->getParam("gravity_weighting", gravity_weighting))
    gravity_weighting = 0.5;
  // nh->param("rotational_weighting", rotational_weighting, 0.33);
  // nh->param("positional_weighting", positional_weighting, 0.1);

  // Determine score based on position within shelf
  if (vertical_only == false) {
      positional_score = getPositionalWeighting(pose, bin_name);
  } else {
      positional_score = 0.0;
      rotational_weighting += positional_weighting;
  }

  // Determine the angle between the currents grasps x axis and
  // the worlds z axis
  angle_between_x_and_z = angle_between_pose(pose, UNIT_X_IDX, UNIT_Z_IDX);

  // Determine the angle between the currents grasps x axis and
  // the bin's
  // geometry_msgs::Pose pose_bin_frame;
  // transformPoseNoLookup(pose, base_to_bin_transform,
  //                       &pose_bin_frame);
  // angle_between_x_and_y = angle_between_pose(pose, UNIT_X_IDX, UNIT_Y_IDX);
  //
  // ROS_INFO_STREAM("Angle between x and z:  " << angle_between_x_and_z);
  // ROS_INFO_STREAM("Angle between x and y:  " << angle_between_x_and_y);
  // angle_between_x_and_y += (45 * M_PI / 180);
  // Determine the rotational and move group scores

  if (vertical_only == false) {
      if (group_name == "left_arm" || group_name == "right_arm") {
        rotational_score = sin(angle_between_x_and_z);
        // rotational_score = (sin(angle_between_x_and_z) + sin(angle_between_x_and_y)) / 2.0;
        // ROS_INFO_STREAM("Score between x and y:  " << sin(angle_between_x_and_y));
        move_group_score = 0.5;
      } else {
        rotational_score = -cos(angle_between_x_and_z);
        // rotational_score = std::max(-cos(angle_between_x_and_z), std::abs(cos(angle_between_x_and_y) * 0.5));
        // ROS_INFO_STREAM("Score between x and y:  " << std::abs(cos(angle_between_x_and_y)));
        move_group_score = 1;
        // Possible TODO (per item switch could be implemented)
      }
  } else {
      if (group_name == "left_arm" || group_name == "right_arm") {
        rotational_score = -cos(angle_between_x_and_z);
        move_group_score = 1.0;
      } else {
        rotational_score = sin(angle_between_x_and_z);
        move_group_score = 0.5;
        // Possible TODO (per item switch could be implemented)
      }
  }
  ROS_INFO("Score final = %f + %f x %f + %f x %f + %f x %f", grasp_utility, positional_score, positional_weighting, rotational_score, rotational_weighting, move_group_score, move_group_weighting);

  grasp_utility += positional_score * positional_weighting +
                   rotational_score * rotational_weighting +
                   move_group_score * move_group_weighting;
  ROS_INFO_STREAM("            = " << grasp_utility);

  return grasp_utility;
}

void publishGraspMarkers(geometry_msgs::PoseArray grasp_poses,
                         std::vector<double> grasp_utility, double marker_scale,
                         std::vector<std::string> move_group_names) {
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

  for (unsigned int i = 0; i < grasp_poses.poses.size(); i++) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = grasp_poses.header.frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = "grasp_poses";
    marker.id = i;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = std::abs(marker_scale);
    marker.scale.y = 0.0035;
    marker.scale.z = 0.0035;
    marker.color.a = 1.0;

    //        double max = *std::max_element(grasp_utility.begin(),
    //        grasp_utility.end());
    double max = grasp_utility[0];
    double min = grasp_utility[grasp_utility.size() - 1];
    if ((move_group_names[i] == "left_arm") || (move_group_names[i] == "right_arm")) {
        if (i == 0) {
          marker.color.r = 0.0;
          marker.color.g = 0.0;
          marker.color.b = 1.0;
        } else {
          marker.color.r = (grasp_utility[i] - min) / (max - min);
          marker.color.g = 0.0;
          marker.color.b = 0.0;
        }
    } else {
        if (i == 0) {
          marker.color.r = 0.3;
          marker.color.g = 0.0;
          marker.color.b = 0.7;
        } else {
          marker.color.r = 0.0;
          marker.color.g = (grasp_utility[i] - min) / (max - min);
          marker.color.b = 0.0;
        }
    }

    //        marker.pose = grasp_poses.poses[i];
    marker.pose = translate_pose(grasp_poses.poses[i],
                                 Eigen::Vector3d(-marker.scale.x, 0, 0));
    grasp_array.markers.push_back(marker);
  }

  marker_array_pub.publish(grasp_array);
}

bool selectGraspPoseFromCandidates(
    apc_msgs::SelectGraspFromCandidates::Request &req,
    apc_msgs::SelectGraspFromCandidates::Response &res) {
  // Client to call moveit_lib service
  // ros::NodeHandle nh;
  std::vector<robot_state::JointModelGroup *> joint_model_groups;
  std::vector<grasp_info> selected_grasps, scored_grasps, pre_grasps;
  std::vector<apc_msgs::GraspPose> scored_grasp_msgs;
  std::vector<std::string> grasp_move_group;
  std::vector<double> grasp_utilities_unsorted;

  geometry_msgs::PoseStamped current_grasp;
  geometry_msgs::PoseStamped current_grasp_bin;

  std::string planning_frame;

  int num_IK_attempts;
  int max_grasps_selected = 100;
  int nSelected = 0;

  geometry_msgs::PoseArray marker_grasp_array;
  std::vector<std::string> marker_name_array;
  std::vector<double> marker_utilities;
  std::vector<double> selected_grasp_utility;
  double pre_grasp_offset, grasp_offset, grasp_90_offset, vertical_range;

  ROS_INFO("Starting Grasp Selection Service");

  nh->param("num_IK_attempts", num_IK_attempts, 1);
  nh->param("pre_grasp_offset", pre_grasp_offset, 0.05);
  nh->param("planning_frame", planning_frame, std::string("base"));
  nh->param("grasp_offset", grasp_offset, 0.0);
  nh->param("grasp_90_offset", grasp_90_offset, 0.0);
  nh->param("vertical_range", vertical_range, M_PI / 8); // 20 degrees
  // nh->param("max_grasps_selected", max_grasps_selected, 0);

  // Find the pose array
  geometry_msgs::PoseArray grasp_candidates = req.grasp_candidates.grasp_poses;

  // Determine whether baxter is picking or storing items
  vertical_only = req.vertical_only;

  // Find the respective pose scores
  std::vector<double> grasp_utilities = req.grasp_candidates.grasp_utilities;
  // Find which bin is being used in the shelf
  bin_name = req.bin_name;
  ROS_INFO_STREAM("Selecting Grasps in Bin: " << bin_name);
  std::vector<std::string> move_group_names = req.move_group_names_to_check;

  current_grasp_bin.header.frame_id = bin_name;
  res.success.data = false;

  // If max grasps uninitialised use length of grasp array
  // if (max_grasps_selected == 0) {
  //     ROS_INFO("Max grasps not set will return all selected grasps");
  //     max_grasps_selected = grasp_utilities.size();
  // }

  printf("\n\n\n\n\n\n\nWHICH_ARM: %s\n\n\n\n\n\n\n", which_arm.c_str());

  // Inverse kinematics

  planning_scene::PlanningScenePtr planning_scene =
      planning_scene_monitor_->getPlanningScene();

  int len = move_group_names.size();
  ROS_INFO("Starting loop... (with %d move_groups)", len);

  lookupTransform(bin_name, planning_frame, &base_to_bin_transform);
  lookupTransform(planning_frame, bin_name, &bin_to_base_transform);
  lookupTransform(planning_frame, grasp_candidates.header.frame_id,
                  &grasp_transform);

  current_grasp.header.frame_id = planning_frame;

  // double roll, pitch, yaw;
  // geometry_msgs::Quaternion q_msg;
  // tf::Quaternion q;

  for (unsigned int i = 0; i < move_group_names.size(); i++) {
    std::string group_name = move_group_names[i];
    TICK(100);
    for (unsigned int j = 0; j < grasp_candidates.poses.size(); j++) {
      apc_msgs::GraspPose current_grasp_msg;

      transformPoseNoLookup(grasp_candidates.poses[j], grasp_transform,
                            &(current_grasp_msg.grasp_pose));

      // Change every pose to straight on, ie. set z orientation to 0
      // transformPoseNoLookup(current_grasp.pose, base_to_bin_transform,
      //                       &current_grasp_bin.pose);
      // q_msg = current_grasp_bin.pose.orientation;
      // tf::quaternionMsgToTF(q_msg, q);
      // tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
      // roll = 0;
      //
      // current_grasp_bin.pose.orientation =
      //     tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
      // transformPoseNoLookup(current_grasp_bin.pose,
      // bin_to_base_transform,
      //                       &current_grasp.pose);

      // Offset the grasp pose
      if(group_name == "left_arm" || group_name == "right_arm"){
          current_grasp_msg.grasp_pose = translate_pose(
          current_grasp_msg.grasp_pose, Eigen::Vector3d(grasp_offset, 0, 0));
      }else{
          current_grasp_msg.grasp_pose = translate_pose(
          current_grasp_msg.grasp_pose, Eigen::Vector3d(grasp_90_offset, 0, 0));
      }

      current_grasp_msg.pre_grasp_pose =
          translate_pose(current_grasp_msg.grasp_pose,
                         Eigen::Vector3d(-pre_grasp_offset, 0, 0));

      // Check IK and Collisions for current pose
      double angle = angle_between_pose(current_grasp_msg.grasp_pose,
                                        UNIT_X_IDX, UNIT_Z_IDX);
      double upper_bound = M_PI + vertical_range;
      double lower_bound = M_PI - vertical_range;

    //   if (!(req.vertical_only && (lower_bound < angle) &&
    //         (angle > upper_bound))) {

        if (checkIKandCollision(kinematic_model->getJointModelGroup(group_name),
                                robot_state_, &current_grasp_msg.grasp_pose,
                                1) &&
            checkIKandCollision(kinematic_model->getJointModelGroup(group_name),
                                robot_state_, &current_grasp_msg.pre_grasp_pose,
                                1)) {
          // Without Rotation
          ROS_INFO("Found Collision Free IK for grasp and pre grasp pose");
          // grasp_info current_grasp_info;
          // current_grasp_info.pose = current_grasp_msg.grasp_pose;
          // current_grasp_info.move_group_name = group_name;

          current_grasp_msg.move_group_name = group_name;

          scored_grasp_msgs.push_back(current_grasp_msg);
          // scored_grasps.push_back(current_grasp_info);
        //   if (!req.vertical_only) {
            grasp_utilities_unsorted.push_back(updateUtility(
                current_grasp_msg.grasp_pose, grasp_utilities[j], group_name));
        //   }else{
            // grasp_utilities_unsorted.push_back(grasp_utilities[j]);
        //   }
        } else {
          // printf("\n\n\n\n\n\n\nGroup Name: %s\n\n\n\n\n\n\n", group_name.c_str());
          ROS_INFO("Found no solution for grasp pose");
        }
    //   } else {
    //     ROS_INFO("Vertical only pose was not within range");
    //   }
    }
    TOCK(100);
    std::cout << "this is the grasp candidate loop finished" << std::endl;
  }

  if (scored_grasp_msgs.size() > 0) {
    ROS_INFO("Finished Scoring Grasps, Sorting and Validating IK & Collision");

    std::cout << "lengths: " << scored_grasp_msgs.size() << ", "
              << grasp_utilities_unsorted.size() << std::endl;
    assert(scored_grasp_msgs.size() == grasp_utilities_unsorted.size());

    // Sort selected grasps using updated utilities
    // std::vector<std::pair<grasp_info, double>> sorted_grasp_pairs =
    //     sort_grasp_vectors(scored_grasps, grasp_utilities_unsorted);

    std::vector<std::pair<apc_msgs::GraspPose, double>> sorted_grasp_pairs =
        sort_grasp_msg_vectors(scored_grasp_msgs, grasp_utilities_unsorted);

    double marker_scale;
    nh->param("marker_scale", marker_scale, 0.1);

    // for (unsigned int i = 0;
    //      i < max_grasps_selected && i < sorted_grasp_pairs.size(); i++) {
    //     geometry_msgs::Pose pose = sorted_grasp_pairs[i].first.pose;
    //
    //     res.selected_grasps.poses.push_back(pose);
    //     res.selected_pre_grasps.poses.push_back(
    //         translate_pose(pose, Eigen::Vector3d(-pre_grasp_offset, 0,
    //         0)));
    //     res.grasp_move_group.push_back(
    //         sorted_grasp_pairs[i].first.move_group_name);
    //     selected_grasp_utility.push_back(sorted_grasp_pairs[i].second);
    // }

    // Using apc_msgs::GraspPose
    for (unsigned int i = 0;
         i < max_grasps_selected && i < sorted_grasp_pairs.size(); i++) {
      res.selected_grasps.poses.push_back(
          sorted_grasp_pairs[i].first.grasp_pose);
      res.selected_pre_grasps.poses.push_back(
          sorted_grasp_pairs[i].first.pre_grasp_pose);
      res.grasp_move_group.push_back(
          sorted_grasp_pairs[i].first.move_group_name);
      selected_grasp_utility.push_back(sorted_grasp_pairs[i].second);
    }

    res.success.data = true;

    // double marker_scale;
    // nh->param("marker_scale", marker_scale, 1.0);
    res.selected_grasps.header.frame_id = planning_frame;
    res.selected_pre_grasps.header.frame_id = planning_frame;

    pose_array_pub.publish(res.selected_grasps);

    publishGraspMarkers(res.selected_grasps, selected_grasp_utility,
                        marker_scale, res.grasp_move_group);

  } else {
    ROS_INFO("Did not find any successful grasp points!");
    res.success.data = false;
  }
  ROS_INFO("Finished Selecting Grasps");

  return true;
}

bool selectGraspPoseFromModel(apc_msgs::SelectGraspFromModel::Request &req,
                              apc_msgs::SelectGraspFromModel::Response &res) {
  // ros::AsyncSpinner spinner(2);
  // spinner.start();

  // Client to call moveit_lib service
  // ros::NodeHandle nh;
  ros::ServiceClient move_robot_pose_client =
      nh->serviceClient<moveit_lib::move_robot_pose>(
          "/moveit_lib/move_robot_pose");
  moveit_lib::move_robot_pose move_robot_pose_srv;

  // Get the item id
  std::string item_TF_id(req.item_id.data.c_str());

  // // Get the transform between the two end effectors

  // Remove item from allowed collision matrix
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScene planning_scene(kinematic_model);
  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  collision_detection::AllowedCollisionMatrix acm =
      planning_scene.getAllowedCollisionMatrix();
  robot_state::RobotState copied_state = planning_scene.getCurrentState();

  // acm.setEntry("left_gripper", item_TF_id, true);

  // Inverse kinematics
  robot_state::RobotStatePtr kinematic_state(
      new robot_state::RobotState(kinematic_model));

  const robot_state::JointModelGroup *joint_model_group =
      kinematic_model->getJointModelGroup(which_arm);

  tf::Transform tfItemGoal;
  tf::StampedTransform stampedTfItem;
  tf::Transformer box2Goal;
  tf::Transformer base2Goal;
  Eigen::Affine3d box2GoalTFMatrix;

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  geometry_msgs::PoseStamped goal_pose;
  geometry_msgs::PoseStamped goal_pose_in_base;
  // geometry_msgs::PoseStamped goalPoseBase_90;

  int pose_amount;
  nh->param<int>(item_TF_id + "/poses/amount", pose_amount, 0);

  std::vector<double> pose_vector;

  // Wait for transform
  // tf_listener->waitForTransform("base", item_TF_id, ros::Time::now(),
  //   ros::Duration(5.0));
  for (int i = 1; i <= pose_amount; i++) {
    nh->getParam(item_TF_id + "/poses/pose" + std::to_string(i), pose_vector);

    // Set the posiiton of the goal pose
    goal_pose.pose.position.x = pose_vector[0];
    goal_pose.pose.position.y = pose_vector[1];
    goal_pose.pose.position.z = pose_vector[2];

    // Set the orientation of the goal pose
    geometry_msgs::Quaternion q = tf::createQuaternionMsgFromRollPitchYaw(
        pose_vector[3] * (M_PI / 180), pose_vector[4] * (M_PI / 180),
        pose_vector[5] * (M_PI / 180));
    goal_pose.pose.orientation.x = q.x;
    goal_pose.pose.orientation.y = q.y;
    goal_pose.pose.orientation.z = q.z;
    goal_pose.pose.orientation.w = q.w;

    goal_pose.header.frame_id = item_TF_id;

    // Find goal pose in the base frame for the gripper
    // tf_listener->
    transformPose(goal_pose, "base", &goal_pose_in_base);

    // Find goal pose in the base frame for the 90 degree gripper

    // Determine whether pose is possible to reach
    bool found_ik = kinematic_state->setFromIK(joint_model_group,
                                               goal_pose_in_base.pose, 10, 0.1);

    // Determine whether pose in in collision
    collision_result.clear();
    planning_scene.checkCollision(collision_request, collision_result,
                                  copied_state, acm);

    ROS_INFO_STREAM("Found IK? : " << found_ik);
    ROS_INFO_STREAM("In Collision? : " << collision_result.collision);
    ROS_INFO_STREAM("Pose:" << goal_pose_in_base.pose);

    // Set the request parameters for the moveit_lib service
    move_robot_pose_srv.request.move_group.data = which_arm;
    printf("\n\n\n\n\n\n\nWHICH_ARM: %s\n\n\n\n\n\n\n", which_arm.c_str());
    move_robot_pose_srv.request.target_pose = goal_pose_in_base;
    if (found_ik) {
      // success = left_arm.plan(left_plan);
      if (move_robot_pose_client.call(move_robot_pose_srv)) {
        if (move_robot_pose_srv.response.success.data == true) {
          ROS_INFO_STREAM("Successfully moved to goal.");
          break;
        } else {
          ROS_INFO_STREAM("Could not plan to target. Trying another pose");
        }
      } else {
        ROS_WARN_STREAM("Failed to call movelib service.");
      }
    }
  }

  return 1;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "grasp_selection_service_node");

  nh = new ros::NodeHandle("~");

  tf_listener = new tf::TransformListener();

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  kinematic_model = robot_model_loader.getModel();

  robot_state_.reset(new robot_state::RobotState(kinematic_model));

  std::string marker_array_topic;
  nh->param("marker_array_topic", marker_array_topic,
            std::string("/selected_grasp_markers"));
  marker_array_pub =
      nh->advertise<visualization_msgs::MarkerArray>(marker_array_topic, 0);

  pose_array_pub =
      nh->advertise<geometry_msgs::PoseArray>("/selected_grasp_poses", 0);

  planning_scene_monitor_ =
      boost::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
          "robot_description");

  planning_scene_monitor_->startStateMonitor("/joint_states",
                                             "/attached_collision_object");
  planning_scene_monitor_->startSceneMonitor("/planning_scene");
  planning_scene_monitor_->startWorldGeometryMonitor();

  // Advertise Services
  ros::ServiceServer graspService = nh->advertiseService(
      "/apc_grasping/grasp_selection_from_model", &selectGraspPoseFromModel);
  ros::ServiceServer graspArrayService =
      nh->advertiseService("/apc_grasping/grasp_selection_from_candidates",
                           &selectGraspPoseFromCandidates);


  nh->param("/apc_global/which_arm", which_arm, std::string("left_arm"));

  ROS_INFO("grasp_pose service started.");

  ros::AsyncSpinner spinner(2);
  spinner.start();

  // Loop forever
  ros::Rate rate(50);
  while (ros::ok()) {
    rate.sleep();
  }

  return 0;
}
