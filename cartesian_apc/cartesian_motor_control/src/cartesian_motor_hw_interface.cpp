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
   CartesianMotor
           For a more detailed simulation example, see sim_hw_interface.cpp
*/

#include <cartesian_motor_control/cartesian_motor_hw_interface.h>

namespace cartesian_motor_control {

static const std::string joint_names[] = {
    "x_axis_joint",
    "y_axis_joint",
    "z_axis_joint",
    "yaw_joint",
    "roll_joint",
    "pitch_joint"
};

CartesianMotorHWInterface::CartesianMotorHWInterface(ros::NodeHandle &nh) :
    nh_(nh),
    joint_names_(std::begin(joint_names), std::end(joint_names)),
    read_joint_position_(joint_idx::JOINTSTATE_LEN),
    joint_position_(joint_idx::JOINTSTATE_LEN),
    joint_velocity_(joint_idx::JOINTSTATE_LEN),
    joint_effort_(joint_idx::JOINTSTATE_LEN),
    joint_position_command_(joint_idx::JOINTSTATE_LEN),
    num_joints_(joint_idx::JOINTSTATE_LEN),

    sucker_joint_name_("sucker_pitch_joint"),
    sucker_read_joint_position_(0),
    sucker_joint_position_(0),
    sucker_joint_velocity_(0),
    sucker_joint_effort_(0),
    sucker_joint_position_command_(0)
{
    ROS_INFO_NAMED("cartesian_motor_hw_interface", "CartesianMotorHWInterface Ready.");
}

void CartesianMotorHWInterface::feedbackCb(const sensor_msgs::JointState &msg) {
    for(std::size_t i = 0; i < num_joints_; ++i){
        read_joint_position_[i] = msg.position[i];
        //ROS_DEBUG_STREAM_NAMED("cartesian_motor_hw_interface", "Reading joint positions" << read_joint_position_[i]);
    }
}

void CartesianMotorHWInterface::suckerFeedbackCb(const dynamixel_msgs::JointState &msg) {
    sucker_read_joint_position_ = msg.current_pos * -1 * sucker_pitch_joint_scaling_ + sucker_pitch_joint_offset_;
    //sucker_read_joint_position_ = msg.current_pos * -0.6;
}

void CartesianMotorHWInterface::init() {

  //nh_.param("robot_joint_name", joint_name_, std::string("cartesian_robot"));

  nh_.param("motor_command_topic", motorCommandTopic_,
            std::string("/motor_command"));
  nh_.param("motor_feedback_topic", motorFeedbackTopic_,
            std::string("/joint_feedback"));

  nh_.param("motor_command_topic", suckerMotorCommandTopic_,
            std::string("/elephant_controller/command"));
  nh_.param("motor_feedback_topic", suckerMotorFeedbackTopic_,
            std::string("/elephant_controller/state"));

  nh_.param("sucker_pitch_joint_scaling", sucker_pitch_joint_scaling_, 0.6);
  nh_.param("sucker_pitch_joint_offset", sucker_pitch_joint_offset_, 0.0);

  //lift_cmd_publisher_ = nh_.advertise<std_msgs::Float32>(liftCommandTopic_, 0);
  motor_feedback_subscriber_ = nh_.subscribe(
      motorFeedbackTopic_, 0, &CartesianMotorHWInterface::feedbackCb, this);

  sucker_motor_feedback_subscriber_ = nh_.subscribe(
      suckerMotorFeedbackTopic_, 0, &CartesianMotorHWInterface::suckerFeedbackCb, this);

  ros::Duration(5.0).sleep();

  motor_cmd_publisher_ = nh_.advertise<sensor_msgs::JointState>(motorCommandTopic_, 0);
  sucker_motor_cmd_publisher_ = nh_.advertise<std_msgs::Float64>(suckerMotorCommandTopic_, 0);


  // Initialize controller
  for (std::size_t i = 0; i < num_joints_; ++i) {

    ROS_DEBUG_STREAM_NAMED("cartesian_motor_hardware_interface",
                           "Loading joint name: " << joint_names_[i]);

   joint_state_interface_.registerHandle(hardware_interface::JointStateHandle(
       joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]));

    // Create position joint interface
    position_joint_interface_.registerHandle(hardware_interface::JointHandle(
        joint_state_interface_.getHandle(joint_names_[i]), &joint_position_command_[i]));

  }

  ROS_DEBUG_STREAM_NAMED("cartesian_motor_hardware_interface",
                         "Loading joint name: " << sucker_joint_name_);

  joint_state_interface_.registerHandle(hardware_interface::JointStateHandle(
      sucker_joint_name_, &sucker_joint_position_, &sucker_joint_velocity_, &sucker_joint_effort_));

   // Create position joint interface
   position_joint_interface_.registerHandle(hardware_interface::JointHandle(
       joint_state_interface_.getHandle(sucker_joint_name_), &sucker_joint_position_command_));

  registerInterface(&joint_state_interface_);    // From RobotHW base class.
  registerInterface(&position_joint_interface_); // From RobotHW base class.

  position_interface_running_ = false;
}

// bool CartesianMotorHWInterface::checkForConflict(const std::list<hardware_interface::ControllerInfo>& info){
//     return true;
// }

void CartesianMotorHWInterface::read(ros::Duration &elapsed_time) {
    for(std::size_t i = 0; i < num_joints_; ++i){
        joint_position_[i] = read_joint_position_[i];
        ROS_DEBUG_STREAM_NAMED("cartesian_motor_hw_interface", "Reading joint positions" << read_joint_position_[i]);
    }
    sucker_joint_position_ = sucker_read_joint_position_;
}

void CartesianMotorHWInterface::write(ros::Duration &elapsed_time) {
  // Safety
  // enforceLimits(elapsed_time);
  static int seq = 0;
  bool all_not_init = true;
  seq++;

  sensor_msgs::JointState motor_command;
  motor_command.position.resize(joint_idx::JOINTSTATE_LEN);
  motor_command.header.stamp = ros::Time::now();
  //Position Control
  for(std::size_t i = 0; i < joint_idx::JOINTSTATE_LEN; ++i){
      if( (joint_position_command_[i] >= -10e-10) && (joint_position_command_[i] <= 10e-10) ) joint_position_command_[i] = 0.000000001;
      motor_command.position[i] = joint_position_command_[i];
      if (joint_position_command_[i] != 1e-9) {
        all_not_init = false;
      }
      ROS_DEBUG_STREAM_NAMED("cartesian_motor_hw_interface", "Publishing Joint Command"
                                        << joint_position_command_[i]);
  }
  if(!all_not_init) {
    motor_cmd_publisher_.publish(motor_command);
  }

  std_msgs::Float64 sucker_motor_command;
  sucker_motor_command.data = (sucker_joint_position_command_ - sucker_pitch_joint_offset_) * -1.0 / sucker_pitch_joint_scaling_;
  //sucker_pitch_joint_scaling_ * (-1 * sucker_joint_position_command_);
  //sucker_motor_command.data = (sucker_joint_position_command_)/-0.6;

  sucker_motor_cmd_publisher_.publish(sucker_motor_command);
}

void CartesianMotorHWInterface::enforceLimits(ros::Duration &period) {
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
  //
  // CHOOSE THE TYPE OF JOINT LIMITS INTERFACE YOU WANT TO USE
  // YOU SHOULD ONLY NEED TO USE ONE SATURATION INTERFACE,
  // DEPENDING ON YOUR CONTROL METHOD
  //
  // EXAMPLES:
  //
  // Saturation Limits ---------------------------
  //
  // Enforces position and velocity
  // pos_jnt_sat_interface_.enforceLimits(period);
  //
  // Enforces velocity and acceleration limits
  // vel_jnt_sat_interface_.enforceLimits(period);
  //
  // Enforces position, velocity, and effort
  // eff_jnt_sat_interface_.enforceLimits(period);

  // Soft limits ---------------------------------
  //
  // pos_jnt_soft_limits_.enforceLimits(period);
  // vel_jnt_soft_limits_.enforceLimits(period);
  // eff_jnt_soft_limits_.enforceLimits(period);
  //
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
}

} // namespace
