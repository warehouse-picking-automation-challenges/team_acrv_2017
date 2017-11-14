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
   Desc:   Example ros_control main() entry point for controlling robots in ROS
*/

#include <cartesian_motor_control/cartesian_motor_hw_interface.h>
#include <controller_manager/controller_manager.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>

static const double BILLION = 1000000000.0;

// DON'T CHANGE THESE NUMBERS HERE.
double default_hz_rate = 100.0;  // Backup if not set in controllers.yaml
double default_cycle_time_error_threshold = 0.012; // Backup if not set in controlelrs.yaml

class HWControlLoop {

protected:
  ros::NodeHandle nh_;

  // Name of this class
  std::string name_ = "cartesian_motor";

  ros::Duration desired_update_freq_;
  double cycle_time_error_threshold_;

  // Timing
  ros::Timer non_realtime_loop_;
  ros::Duration elapsed_time_;
  double loop_hz_;

  struct timespec last_time_;
  struct timespec current_time_;

  boost::shared_ptr<cartesian_motor_control::CartesianMotorHWInterface>
      cartesian_motor_hw_interface_;
  boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;

public:
  HWControlLoop(ros::NodeHandle &nh,
                boost::shared_ptr<cartesian_motor_control::CartesianMotorHWInterface>
                    cartesian_motor_hw_interface)
      : nh_(nh), cartesian_motor_hw_interface_(cartesian_motor_hw_interface) {

    controller_manager_.reset(new controller_manager::ControllerManager(
        cartesian_motor_hw_interface_.get(), nh_));

    nh_.param("/hardware_control_loop/loop_hz", loop_hz_, default_hz_rate);
    nh_.param("/hardware_control_loop/cycle_time_error_threshold", cycle_time_error_threshold_,default_cycle_time_error_threshold);


    clock_gettime(CLOCK_MONOTONIC, &last_time_);
    ros::Duration desired_update_freq_ = ros::Duration(1 / loop_hz_);
    non_realtime_loop_ =
        nh_.createTimer(desired_update_freq_, &HWControlLoop::update, this);
  }

  void update(const ros::TimerEvent &e) {
    // Get change in time
    clock_gettime(CLOCK_MONOTONIC, &current_time_);
    elapsed_time_ =
        ros::Duration(current_time_.tv_sec - last_time_.tv_sec +
                      (current_time_.tv_nsec - last_time_.tv_nsec) / BILLION);
    last_time_ = current_time_;

    // Error check cycle time
    const double cycle_time_error =
        (elapsed_time_ - desired_update_freq_).toSec();
    if (cycle_time_error > cycle_time_error_threshold_) {
      ROS_WARN_STREAM_NAMED(
          name_, "Cycle time exceeded error threshold by: "
                     << cycle_time_error << ", cycle time: " << elapsed_time_
                     << ", threshold: " << cycle_time_error_threshold_);
    }

    // Input
    ROS_DEBUG_STREAM("reading from hardware interface");
    cartesian_motor_hw_interface_->read(elapsed_time_);

    // Control
    ROS_DEBUG_STREAM("Updating controller manager");
    controller_manager_->update(ros::Time::now(), elapsed_time_);

    // Output
    ROS_DEBUG_STREAM("writing commands to hardware interface");
    cartesian_motor_hw_interface_->write(elapsed_time_);
  }
};
int main(int argc, char **argv) {
  ros::init(argc, argv, "cartesian_motor_hw_control_loop");
  ros::NodeHandle nh;

  // NOTE: We run the ROS loop in a separate thread as external calls such
  // as service callbacks to load controllers can block the (main) control loop
  ros::AsyncSpinner spinner(2);
  spinner.start();
  boost::shared_ptr<cartesian_motor_control::CartesianMotorHWInterface>
      cartesian_motor_hw_interface(
          new cartesian_motor_control::CartesianMotorHWInterface(nh));
  cartesian_motor_hw_interface->init();

  HWControlLoop hw_control_loop(nh, cartesian_motor_hw_interface);
  // Create the hardware interface specific to your robot

  // Wait until shutdown signal recieved
  ros::waitForShutdown();

  return 0;
}
