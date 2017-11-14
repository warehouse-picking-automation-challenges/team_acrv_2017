#include "moveit_robot.h"

#include <cv_bridge/cv_bridge.h>
#include <eigen_conversions/eigen_msg.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <vector>

#define DEFAULT_PLANNING_TIME 5.0
// #define MAX_ATTEMPTS 5
#define MAX_PARALLEL_ATTEMPTS 15
#define WAIT_TIMEOUT 1.25

const std::string planner = "RRTConnectkConfigDefault";

moveit_robot::moveit_robot(std::string robot_group) : move_group(robot_group) {
    planning_frame = move_group.getPlanningFrame();
    initRobot(planner);
    // initRobot("RRTConnectkConfigDefault");
}

void moveit_robot::initRobot(std::string planner, float velocity_scale) {
    velocity_scale_ = velocity_scale;
    move_group.setPlannerId(planner);
    move_group.setPlanningTime(DEFAULT_PLANNING_TIME);
    // define the global velocity_scale
    move_group.setMaxVelocityScalingFactor(1.0);
    move_group.setStartStateToCurrentState();

    // seems to be quite important :)
    // move_group.allowReplanning(true);

    // number of parallel threads to start
    // https://groups.google.com/forum/#!topic/moveit-users/ADKVAPjs3Jc
    move_group.setNumPlanningAttempts(MAX_PARALLEL_ATTEMPTS);

    configureWorkspace();
}

// bool moveit_robot::moveTo(Eigen::Affine3d pose_eigen, std::string frame_id,
//                           int attempts, double timeout,
//                           double max_planning_time, float velocity_scale) {
//     geometry_msgs::PoseStamped poseStamped;
//     poseStamped.header.stamp = ros::Time::now();
//     poseStamped.header.frame_id = frame_id;
//     tf::poseEigenToMsg(pose_eigen, poseStamped.pose);
//
//     return moveTo(poseStamped, velocity_scale, attempts, timeout);
// }

// THAT:S WHAT THE MOVEIT PLUGIN USES
// void MotionPlanningFrame::configureForPlanning()
// {
//   move_group_->setStartState(*planning_display_->getQueryStartState());
//   move_group_->setJointValueTarget(*planning_display_->getQueryGoalState());
//   move_group_->setPlanningTime(ui_->planning_time->value());
//   move_group_->setNumPlanningAttempts(ui_->planning_attempts->value());
//   move_group_->setMaxVelocityScalingFactor(ui_->exec_time_factor->value());
//   configureWorkspace();
// }

void moveit_robot::configureWorkspace() {
    robot_model::VariableBounds bx, by, bz;
    bx.position_bounded_ = by.position_bounded_ = bz.position_bounded_ = true;

    // max of 3 m workspace from zero origin...
    robot_model::JointModel::Bounds b(3);
    bx.min_position_ = 0.5 - 3.0 / 2.0;
    bx.max_position_ = 0.5 + 3.0 / 2.0;
    by.min_position_ = 1.0 - 3.0 / 2.0;
    by.max_position_ = 1.0 + 3.0 / 2.0;
    bz.min_position_ = 0.5 - 3.0 / 2.0;
    bz.max_position_ = 0.5 + 3.0 / 2.0;

    move_group.setWorkspace(bx.min_position_, by.min_position_,
                            bz.min_position_, bx.max_position_,
                            by.max_position_, bz.max_position_);
    // planning_scene_monitor::PlanningSceneMonitorPtr psm =
    // move_group.getPlanningSceneMonitor();
    // // ::MoveGroupContext::planning_scene_monitor_;
    // //planning_display_->getPlanningSceneMonitor();
    // // get non-const access to the kmodel and update planar & floating joints
    // as indicated by the workspace settings
    // if (psm && psm->getRobotModelLoader() &&
    // psm->getRobotModelLoader()->getModel())
    // {
    //   const robot_model::RobotModelPtr &kmodel =
    //   psm->getRobotModelLoader()->getModel();
    //   const std::vector<robot_model::JointModel*> &jm =
    //   kmodel->getJointModels();
    //   for (std::size_t i = 0 ; i < jm.size() ; ++i)
    //     if (jm[i]->getType() == robot_model::JointModel::PLANAR)
    //     {
    //       jm[i]->setVariableBounds(jm[i]->getName() + "/" +
    //       jm[i]->getLocalVariableNames()[0], bx);
    //       jm[i]->setVariableBounds(jm[i]->getName() + "/" +
    //       jm[i]->getLocalVariableNames()[1], by);
    //     }
    //     else
    //       if (jm[i]->getType() == robot_model::JointModel::FLOATING)
    //       {
    //         jm[i]->setVariableBounds(jm[i]->getName() + "/" +
    //         jm[i]->getLocalVariableNames()[0], bx);
    //         jm[i]->setVariableBounds(jm[i]->getName() + "/" +
    //         jm[i]->getLocalVariableNames()[1], by);
    //         jm[i]->setVariableBounds(jm[i]->getName() + "/" +
    //         jm[i]->getLocalVariableNames()[2], bz);
    //       }
}

// Move the robot arm to the stamped pose provided
// uses the MoveIt planner
bool moveit_robot::moveTo(geometry_msgs::PoseStamped pose, int attempts,
                          double timeout, double max_planning_time,
                          float velocity_scale, bool async) {
    bool ret = false;

    while (attempts > 0) {
        move_group.setPlannerId(planner);
        move_group.setPoseTarget(pose);
        // move_group_->setJointValueTarget(*planning_display_->getQueryGoalState());
        move_group.setPlanningTime(max_planning_time / (double)attempts);
        move_group.setNumPlanningAttempts(MAX_PARALLEL_ATTEMPTS);
        move_group.setMaxVelocityScalingFactor(velocity_scale);
        configureWorkspace();

        // pose.header.stamp = ros::Time::now();
        std::cout << "Planning time we requested = "
                  << (max_planning_time / (double)attempts) << std::endl;
        // seems to make Baxter move smoother if we plan and execute in ONE step
        moveit::planning_interface::MoveItErrorCode execution_success;
        try {
            move_group.setStartStateToCurrentState();
            if(async) {
                execution_success = move_group.asyncMove();
            } else {
                execution_success = move_group.move();
            }
        } catch (...) {
            ROS_INFO_STREAM("Received exception from move_group.move(): "
                            << execution_success);
            ret = false;
        }

        if (execution_success) {
            ROS_INFO("Successfully planned to goal and executed the move!");
            attempts = 0;
            ret = true;
        } else {
            ROS_ERROR("Failed to plan and execute a motion!");
            std::cout << execution_success << std::endl;
            std::cout << "Failed to plan, trying to replan " << std::endl;
            attempts--;
            sleep(timeout);
        }
    }

    return ret;
}

// bool moveit_robot::moveTo(std::string frame_id, Eigen::Vector3d position,
//                           Eigen::Vector4d orientation, int attempts,
//                           double timeout, double max_planning_time,
//                           float velocity_scale) {
//     geometry_msgs::PoseStamped pose;
//     pose.header.frame_id = frame_id;
//     pose.pose.position.x = position[0];
//     pose.pose.position.y = position[1];
//     pose.pose.position.z = position[2];
//
//     pose.pose.orientation.x = orientation[0];
//     pose.pose.orientation.y = orientation[1];
//     pose.pose.orientation.z = orientation[2];
//     pose.pose.orientation.w = orientation[3];
//
//     return moveTo(pose, attempts, timeout, velocity_scale);
// }

double moveit_robot::moveToCartesianPath(
    std::vector<geometry_msgs::Pose> waypoints, int attempts, double timeout,
    double max_planning_time, float velocity_scale, float step_size,
    float jump_threshold, double min_path_completion) {
    double fraction_complete = 0.0;  // return value
    // The trajectory needs to be modified so it will include velocities as
    // well.
    moveit_msgs::RobotTrajectory trajectory_msg;  // eef_step // jump_threshold

    while (attempts > 0) {
        move_group.setStartStateToCurrentState();
        move_group.setMaxVelocityScalingFactor(velocity_scale);
        move_group.setNumPlanningAttempts(MAX_PARALLEL_ATTEMPTS);
        move_group.setPlanningTime(max_planning_time / attempts);
        // move_group.setPoseReferenceFrame(move_group.getPlanningFrame());

        printf("attempts: %d, max-t: %f\n\n\n ", attempts, max_planning_time);

        fraction_complete = move_group.computeCartesianPath(
            waypoints, step_size, jump_threshold, trajectory_msg);
        ROS_INFO("Cartesian Path Planning returned (%.2f%% acheived)",
                 fraction_complete * 100.0);

        // if (fraction_complete >
        //     0.25) {  // more than 25% done, that's good, so move the arm
        if (fraction_complete > min_path_completion) {
            robot_trajectory::RobotTrajectory rt(
                move_group.getCurrentState()->getRobotModel(),
                move_group.getName());

            // Second get a RobotTrajectory from trajectory
            rt.setRobotTrajectoryMsg(*move_group.getCurrentState(),
                                     trajectory_msg);

            // Thrid create a IterativeParabolicTimeParameterization object
            trajectory_processing::IterativeParabolicTimeParameterization iptp;

            // Fourth compute computeTimeStamps
            bool success = iptp.computeTimeStamps(rt, velocity_scale);
            ROS_INFO("Computed time stamp %s", success ? "SUCCEDED" : "FAILED");

            // Get RobotTrajectory_msg from RobotTrajectory
            rt.getRobotTrajectoryMsg(trajectory_msg);

            // Finally plan and execute the trajectory
            robot_plan_.trajectory_ = trajectory_msg;
            ROS_INFO("Visualizing plan (cartesian path) (%.2f%% acheived)",
                     fraction_complete * 100.0);

            try {
                moveit::planning_interface::MoveItErrorCode move_success =
                    move_group.execute(robot_plan_);
                if (move_success) {
                    // TODO check the succes/error variable?
                    if (fraction_complete > min_path_completion) return fraction_complete;
                } else {
                    ROS_ERROR("Execution of Cartesian path failed!");
                }
            } catch (moveit::planning_interface::MoveItErrorCode ex) {
                ROS_INFO_STREAM("Something went wrong. Failed to move to pose");
                return false;
            }
        }
        attempts--;
        ROS_INFO(
            "Cartesian Path Planning not Complete (%.2f%% acheived) number of "
            "attempts left = %i",
            fraction_complete * 100.0, attempts);
    }

    return fraction_complete;
}

// move the robot to a named position (ie. a joint pose from the SRDF)
bool moveit_robot::moveToNamed(std::string namedGoal, int attempts,
                               double timeout, double max_planning_time,
                               float velocity_scale) {
    bool ret = false;

    while (attempts > 0) {
        move_group.setPlannerId(planner);
        move_group.setNamedTarget(namedGoal);
        move_group.setMaxVelocityScalingFactor(velocity_scale);
        move_group.setPlanningTime(max_planning_time / attempts);
        move_group.setNumPlanningAttempts(MAX_PARALLEL_ATTEMPTS);
        configureWorkspace();

        moveit::planning_interface::MoveItErrorCode execution_success;
        try {
            move_group.setStartStateToCurrentState();
            execution_success = move_group.move();
        } catch (...) {
            ROS_INFO_STREAM("Received exception from move_group.move(): "
                            << execution_success);
            ret = false;
        }

        if (execution_success) {
            ROS_INFO("Successfully planned to goal and executed the move!");
            attempts = 0;
            ret = true;
        } else {
            std::cout << "Planning time we requested = "
                      << max_planning_time / attempts << std::endl;
            ROS_ERROR("Failed to plan and execute a motion!");
            std::cout << execution_success << std::endl;
            std::cout << "Failed to plan, trying to replan " << std::endl;
            attempts--;
            sleep(timeout);
        }
        // if (move_group.plan(robot_plan_)) {
        //   std::cout << "Successfully planned to " << namedGoal << std::endl;
        //   ROS_INFO("Successfully planned to goal");
        //   if (executeMove()) {
        //     attempts = 0;
        //     ret = true;
        //   }
        // } else {
        //   std::cout << "Failed to plan, trying to replan" << std::endl;
        //   attempts--;
        //   sleep(timeout);
        // }
    }

    return ret;
}

// moves arm while constraining the end effector orientation to the qConstraint
// variable
bool moveit_robot::moveToOrientationConstraint(
    geometry_msgs::PoseStamped pose, geometry_msgs::Quaternion qConstraint,
    std::string link_name, int attempts, double timeout,
    double max_planning_time, float velocity_scale) {
    moveit_msgs::OrientationConstraint ocm;
    ocm.link_name = link_name;
    ocm.header.frame_id = planning_frame;
    ocm.orientation = qConstraint;
    ocm.absolute_x_axis_tolerance = 0.5;
    ocm.absolute_y_axis_tolerance = 0.5;
    ocm.absolute_z_axis_tolerance = 0.5;
    ocm.weight = 1.0;

    moveit_msgs::Constraints constraints;
    moveit_msgs::Constraints oldConstraints;
    oldConstraints = move_group.getPathConstraints();
    constraints = oldConstraints;

    constraints.orientation_constraints.push_back(ocm);
    move_group.setPathConstraints(constraints);

    move_group.setMaxVelocityScalingFactor(velocity_scale);

    bool ret = false;

    while (attempts > 0) {
        pose.header.stamp = ros::Time::now();
        move_group.setPoseTarget(pose);
        move_group.setStartStateToCurrentState();
        // seems to make Baxter move smoother if we plan and execute in ONE step
        moveit::planning_interface::MoveItErrorCode execution_success =
            move_group.move();
        if (execution_success) {
            ROS_INFO("Successfully planned to goal and executed the move!");
            attempts = 0;
            ret = true;
        } else {
            ROS_ERROR("Failed to plan and execute a motion!");
            std::cout << execution_success << std::endl;
            std::cout << "Failed to plan, trying to replan " << std::endl;
            attempts--;
            sleep(timeout);
        }

        // success = move_group.plan(robot_plan_);
        //
        // if(success){
        //   ROS_INFO("Successfully planned to goal");
        //   if(executeMove()) {
        //     attempts = 0;
        //     ret = true;
        //   }else{
        //     ret = false;
        //   }
        // }else{
        //   attempts--;
        //   sleep(timeout);
        // }
    }

    move_group.setPathConstraints(oldConstraints);

    return ret;
}

bool moveit_robot::stop() {
    move_group.stop();
    return true;
}

// execute the planned moveTo
// TODO check if we have a valid plan!!
bool moveit_robot::executePlan() {
    try {
#ifdef DEBUG
        std::cout << "Press the ENTER key to move arm or r to RESET";
        unsigned char c = ' ';
        while (c << std::cin.get()) {
            if (c == '\n')
                break;
            else if (c == 'r')
                ;
            return false;
        }
#endif
        // TODO if(! robot_plan_) move group.move()
        bool flag = move_group.execute(robot_plan_);  // execute the plan stored
        std::cout << "Moved to Goal" << std::endl;
        return flag;
    } catch (moveit::planning_interface::MoveItErrorCode ex) {
        std::cout << "Something went wrong. Failed to move to pose"
                  << std::endl;
        return false;
    }
}
