* Make sure everything is physically ok, storage and tote are in correct positions and not touching things.

* Make sure apc-puppy is booted and ready to rumble.

* start a `roscore`

* Home the robot:
  * Turn robot off
  * Hit e-stop
  * Unplug the teensy
  * Move robot to x,y = 0  (bottom right upright)
  * Plug in teensy
  * Release e-stop
  * Robot will home.
  * Watch out for cables getting caught.

* Start the robot software
  * (If you've rebooted the computer, make sure the robot is ttyACM0 and the arduino is ttyACM1. You can do this by unplugging both, wait 10s, plug in robot, plug in arduino).
  * `roslaunch state_machine launch_robot.launch`  (This starts the robot, dynamixels and arduino)
  * When rviz starts up, make sure that the robot looks like it's in the right place.  If it's not, start again.
  * Make sure you can move the robot using rviz.
  * *I recommend moving the robot forward from its home position otherwise the realsense hits the vacuum cable when it immediately moves down*

* Start all the scales:
  * Make sure all of the scales are plugged in, and tared with nothing in the storage/tote.
  * `roslaunch scales_interface all_scales.launch`
  * rostopic echo /stow_tote_scales/weight, /storage_A_scales/weight and /storage_B_scales/weight to make sure they're working
  * Obviously no stow_tote_scales for pick task.

* Start the vision stuff:
  * `roslaunch state_machine launch_vision.launch`
  * Make sure you can pull up a point cloud on rviz (realsense_wrist/depth_registered/points) just to make sure it is running. Everything else should be fine hopefully.

* Put the order files etc in the right place:
  * `~/APC_JSON_DATA/(stow or pick)`

* For stow task:
  * `rosrun state_machine APC_STOW_MACHINE.py`

* For pick task:
  * `rosrun state_machine APC_PICK_MACHINE.py`
