


source /opt/ros/indigo/setup.bash
cd ~/matt_ws
source devel/setup.bash

## For moveit
roscore
roslaunch cartesian_description upload_moveit.launch
roslaunch cartesian_motor_control cartesian_motor_hardware.launch
roslaunch ros_arduino_python arduino.launch
roslaunch roslaunch cartesian_moveit_config CartMan_moveit.launch

## For manual control
rosrun rqt_gui rqt_gui
