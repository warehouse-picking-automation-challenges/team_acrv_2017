#! /usr/bin/python

import roslib
import rospy
import sys
import moveit_lib.srv as msrv
import std_msgs.msg
import geometry_msgs.msg
import bottle_detector.srv
import tf.transformations
import tf2_ros
msg_seq_no = 0

if __name__ == "__main__":
    rospy.init_node('my_temp_node')
    reference_frame = 'camera_depth_optical_frame'

    moveit_service_name = '/moveit_lib/move_robot_pose'
    bottle_service_name = '/bottle_detector/get_object_pose'

    # Wait for the bottle pose service to be available.
    rospy.loginfo('Waiting for %s service ...' % bottle_service_name)
    try:
        rospy.wait_for_service(bottle_service_name, timeout=1)
    except:
        rospy.logerr('Service %s not available. Restart and try again.'
            % bottle_service_name)

    bottle_pose_service = rospy.ServiceProxy(bottle_service_name, bottle_detector.srv.GetObjectPose)

    # Get the bottle pose from the camera.
    req = bottle_detector.srv.GetObjectPoseRequest()
    req.empty = std_msgs.msg.Empty()
    #req.object_name.data = 'blue_bottle'
    res = bottle_pose_service.call(req)


    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    p = res.object_pose.pose

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = reference_frame
    t.child_frame_id = "bottle_pose"
    t.transform.translation.x = p.position.x
    t.transform.translation.y = p.position.y
    t.transform.translation.z = p.position.z

    # Transform the bottle pose to the correct rotation.
    q = tf.transformations.quaternion_from_euler(-0.25, 0, 0)
    q2 = tf.transformations.quaternion_from_euler(0, -1.57, 0)

    qf = tf.transformations.quaternion_multiply(q, q2)

    t.transform.rotation.x = qf[0] #p.orientation.x
    t.transform.rotation.y = qf[1] #p.orientation.y
    t.transform.rotation.z = qf[2] #p.orientation.z
    t.transform.rotation.w = qf[3] #p.orientation.w

    # If you want to publish the transform to the bottle, remove 'False and'
    while False and not rospy.is_shutdown():
        t.header.stamp = rospy.Time.now()
        br.sendTransform(t)

    # Request a move
    mrp = rospy.ServiceProxy(moveit_service_name, msrv.move_robot_pose)
    req = msrv.move_robot_poseRequest()
    req.target_pose.header.seq = msg_seq_no
    req.target_pose.header.stamp = rospy.Time.now()
    req.target_pose.header.frame_id = reference_frame

    req.target_pose.pose.position = res.object_pose.pose.position

    req.target_pose.pose.orientation.x = t.transform.rotation.x
    req.target_pose.pose.orientation.y = t.transform.rotation.y
    req.target_pose.pose.orientation.z = t.transform.rotation.z
    req.target_pose.pose.orientation.w = t.transform.rotation.w

    rospy.loginfo(req.target_pose)

    req.move_group.data = 'suction_gripper'
    mrp.wait_for_service()

    res = mrp(req).success.data
    print "Move: " + str(res)

    rospy.sleep(2)

    mrp = rospy.ServiceProxy('/moveit_lib/move_robot_named', msrv.move_robot_named)
    mrp.wait_for_service()
    req = msrv.move_robot_namedRequest()
    req.move_group.data = 'suction_gripper'
    req.named_pose.data = 'home'
    print "Moving CartMan to home"
    res = mrp(req)
    # resv = mvac(reqv)
    # print "Vacuum: " + str(resv.success)
    print "Move: " + str(res.success.data)

    # If you want to publish the transform to the bottle, remove 'False and'
    while False and not rospy.is_shutdown():
        t.header.stamp = rospy.Time.now()
        br.sendTransform(t)
