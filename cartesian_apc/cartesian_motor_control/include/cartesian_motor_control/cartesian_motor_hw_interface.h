/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman
   Desc:   Example ros_control hardware interface blank template for the
   HarveyLift
           For a more detailed simulation example, see sim_hw_interface.h
*/

#ifndef Cartesian_Motor_HW_INTERFACE_H
#define Cartesian_Motor_HW_INTERFACE_H

// C++
#include <boost/scoped_ptr.hpp>

// ROS
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <dynamixel_msgs/JointState.h>



// ROS Controls
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_manager/controller_manager.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>

// #define x_idx      0
// #define y_idx      1
// #define z_idx      2
// #define yaw_idx    3
// #define roll_idx   4
// #define pitch_idx  5
// #define JOINTSTATE_LEN 6

enum joint_idx {
    x_idx = 0,
    y_idx,
    z_idx,
    yaw_idx,
    roll_idx,
    pitch_idx,
    JOINTSTATE_LEN
};
// #include <ros_control_boilerplate/generic_hw_interface.h>



namespace cartesian_motor_control {

/// \brief Hardware interface for a robot
class CartesianMotorHWInterface : public hardware_interface::RobotHW {
public:
  /**
   * \brief Constructor
   * \param nh - Node handle for topics.
   */
  CartesianMotorHWInterface(ros::NodeHandle &nh);

  void feedbackCb(const sensor_msgs::JointState &msg);
  void suckerFeedbackCb(const dynamixel_msgs::JointState &msg);

  /// \brief Initialize the hardware interface
  virtual void init();

  /** \brief Read the state from the robot hardware. */
  virtual void read(ros::Duration &elapsed_time);

  /** \brief Write the command to the robot hardware. */
  virtual void write(ros::Duration &elapsed_time);

  /** \breif Enforce limits for all values before writing */
  virtual void enforceLimits(ros::Duration &period);

  // virtual bool checkForConflict(const std::list<hardware_interface::ControllerInfo>& info);

protected:
  ros::NodeHandle nh_;
  ros::Publisher motor_cmd_publisher_;
  ros::Publisher sucker_motor_cmd_publisher_;
  ros::Subscriber motor_feedback_subscriber_;
  ros::Subscriber sucker_motor_feedback_subscriber_;

  std::string motorCommandTopic_, motorFeedbackTopic_;
  std::string suckerMotorCommandTopic_, suckerMotorFeedbackTopic_;

  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::PositionJointInterface position_joint_interface_;
  //hardware_interface::VelocityJointInterface velocity_joint_interface_;

  std::vector<std::string> joint_names_;
  std::vector<double> read_joint_position_;
  std::vector<double> joint_position_;
  std::vector<double> joint_velocity_;
  std::vector<double> joint_effort_;
  std::vector<double> joint_position_command_;
  std::vector<double> joint_velocity_command_;
  std::vector<double> prev_joint_velocity_command_;

  std::size_t num_joints_;

  std::string sucker_joint_name_;
  double sucker_read_joint_position_;
  double sucker_joint_position_;
  double sucker_joint_velocity_;
  double sucker_joint_effort_;
  double sucker_joint_position_command_;
  double sucker_joint_velocity_command_;
  double sucker_prev_joint_velocity_command_;
  double sucker_pitch_joint_scaling_;
  double sucker_pitch_joint_offset_;

  bool position_interface_running_;
  //bool velocity_interface_running_;

}; // class

} // namespace

#endif
