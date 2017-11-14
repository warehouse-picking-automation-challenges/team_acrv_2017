#ifndef MOVEIT_ROBOT_H
#define MOVEIT_ROBOT_H

#include <eigen_conversions/eigen_msg.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

// Version Number for ROS obtained by running: rosversion roscpp
// indigo = 1.11.21, kinetic = 1.12.7

// NOTE moveit under kinetic updated the name for the move group interface

#if ROS_VERSION_MINIMUM(1, 12, 0) // if version is bigger than 1.12.0
    // Kinetic Code
    #include <moveit/move_group_interface/move_group_interface.h>
#else
    // Indigo Code
    #include <moveit/move_group_interface/move_group.h>
#endif

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <string>

// use this define for enabling debug information with move_group planner and
// execution
// #define DEBUG

class moveit_robot {
   public:
    // Class Constructor
    moveit_robot(std::string robotGroup);

    // Initialise move_group given planner as string and velocity scaling factor
    // from 0 to 1
    void initRobot(std::string planner, float velocity = 1.0);

    // move to a goal pose and frame id, using the number of max_attempts
    // The planning time will start at max_time/max_attempts and increase
    // to max time for the last attempt
    // velocity scale will determine the speed of the motion
   //  bool moveTo(Eigen::Affine3d pose_eigen, std::string frame_id,
   //              int max_attempts, double timeout, double max_planning_time,
   //              float velocity_scale = 1.0);
    bool moveTo(geometry_msgs::PoseStamped pose, int max_attempts,
                double timeout, double max_planning_time,
                float velocity_scale = 1.0, bool async = false);
   //  bool moveTo(std::string frame_id, Eigen::Vector3d position,
   //              Eigen::Vector4d orientation, int max_attempts, double timeout,
   //              double max_planning_time, float velocity_scale = 1.0);
    double moveToCartesianPath(std::vector<geometry_msgs::Pose> waypoints,
                               int max_attempts, double timeout,
                               double max_planning_time, float velocity_scale,
                               float step_size, float jump_threshold, double min_path_completion);
    bool moveToNamed(std::string namedGoal, int max_attempts, double timeout,
                     double max_planning_time, float velocity_scale = 1.0);
    bool moveToOrientationConstraint(geometry_msgs::PoseStamped pose,
                                     geometry_msgs::Quaternion qConstraint,
                                     std::string link_name, int max_attempts,
                                     double timeout, double max_planning_time,
                                     float velocity_scale = 1.0);

   bool stop();

    // execute a planned move!
    bool executePlan();

    void configureWorkspace();

    #if ROS_VERSION_MINIMUM(1, 12, 0) // if version is bigger than 1.12.0
        // Kinetic Code
        moveit::planning_interface::MoveGroupInterface::Plan robot_plan_;
        moveit::planning_interface::MoveGroupInterface move_group;
    #else
        // Indigo Code
        moveit::planning_interface::MoveGroup::Plan robot_plan_;
        moveit::planning_interface::MoveGroup move_group;
    #endif

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    float velocity_scale_;

    std::string ee_link_name;
    std::string planning_frame;
};

#endif  // MOVEIT_ROBOT_H
