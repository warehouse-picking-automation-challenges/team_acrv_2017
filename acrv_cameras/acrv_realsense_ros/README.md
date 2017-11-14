# ros_realsense
A ROS wrapper for Intel Realsense (TM) cameras using the librealsense library.

This package is being developed for the [Australian Centre for Robotic Vision](http://roboticvision.org)'s team in the [Amazon Picking Challenge](http://amazonpickingchallenge.org/).

## Install
### Dependencies
Follow the installation methods at the links below:

* [librealsense](https://github.com/IntelRealSense/librealsense)
* [Robot Operating System (ROS)](http://wiki.ros.org/kinetic/Installation)

## Launch

First source the workspace from the ros workspace (ros_ws), then launch the camera:

```
source devel/setup.bash
roslaunch acrv_realsense_ros realsense_camera.launch
```
To launch the full pipeline, also launch image_pipeline:

```
roslaunch acrv_realsense_ros image_pipeline.launch
```
NOTE: this can be very resource intensive, check cpu usage.

Run the service node to enable the services which can perform a one-time subscription to realsense camera topics.

```
rosrun acrv_realsense_ros acrv_realsense_capture_service
```

To run the a client which subscribes to these services and saves images, run (UNFINISHED):

``
rosrun acrv_realsense_ros acrv_realsense_capture_client
```
