# Run the Robot
## To run the cartesian robot in simulation

* `roslaunch cartesian_gazebo cartesian_gazebo.launch`
* `roslaunch cartesian_moveit_config CartMan_moveit.launch`

## To run the actual Robot

* Make sure the robot is in the home position.
* Power on the motor power supply, wait a few seconds for everyting to power up and settle.
* Press the e-stop switch.
* Plug in the teensy USB cable (if the e-stop isn't pressed when plugging or unplugging it will go a little crazy).
* Reset the e-stop switch.
* Run `roslaunch cartesian_motor_control cartesian_motor_hardware.launch`
* To move the joints manually use rqt.
* To plan and move run `roslaunch cartesian_moveit_config CartMan_moveit.launch`

## Debug
If the wrist joints aren't moving:
* Stop the cartesian_motor_control node running
* Make sure the robot is in the home position
* Press the e-stop
* Unplug the teensy
* Wait a few seconds
* Re-plug the teensy
* Reset the e-stop
* Re-start the software

# To run the realsense

`roslaunch realsense_camera sr300_nodelet_rgbd.launch`

To connect the `camera_link` to the `realsense` tf of the robot, run `roslaunch bottle_detector realsense_static_tf.launch`


# Run everything else

There are a lot of other nodes that are required to run the whole robot.  These will probably change frequently.  The most up to date list of required nodes can be found in the launch_robot.launch file in apc_state_machine.  This file will be used to launch all of the required nodes for running the demo at the time.  

Each of the other packages should have a readme file describing what they do and their functions/nodes/services.
