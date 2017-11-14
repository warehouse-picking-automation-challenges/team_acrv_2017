#! /usr/bin/python

import roslib
import rospy
roslib.load_manifest('cartesian_moveit_config')
import tf2_ros
import tf
import geometry_msgs.msg

if __name__ == '__main__':


	
	rospy.init_node('camera_frame_broadcaster')
	br = tf2_ros.TransformBroadcaster()
	t = geometry_msgs.msg.TransformStamped()
	listener = tf.TransformListener()

	t.header.stamp = rospy.Time.now()
	t.header.frame_id = "y_axis_carriage"
	t.child_frame_id = "camera_link"

	#define the x,y,z offset respect to y_axis
	t.transform.translation.x = 0.1
	t.transform.translation.y = 0.1
	t.transform.translation.z = 0.1


	#define rotation x,y,z,w respect to y_axis
	t.transform.rotation.x = 0
	t.transform.rotation.y = 0
	t.transform.rotation.z = 0
	t.transform.rotation.w = 1


	while not rospy.is_shutdown():
		t.header.stamp = rospy.Time.now()
		br.sendTransform(t)


