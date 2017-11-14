#! /usr/bin/python

import roslib
import rospy
import sys
import moveit_lib.srv as msrv
#import ros_arduino_msgs.srv as ram
msg_seq_no = 0
# demo_poses = ['home']
def move_gripper(position, reference_frame='test'):
    print "test"

if __name__ == "__main__":
    #setup the ros stuff
    # not sure what needs to go here
    if (len(sys.argv) != 4):
        raise IndexError('3 arguments required: move_to_xyz.py x y z')
    rospy.init_node('my_temp_node')
    reference_frame = 'base_link'
    service = '/moveit_lib/move_robot_pose'
    mrp = rospy.ServiceProxy(service, msrv.move_robot_pose)
    req = msrv.move_robot_poseRequest()
    req.target_pose.header.seq = msg_seq_no
    req.target_pose.header.stamp = rospy.Time.now()
    req.target_pose.header.frame_id = reference_frame

    req.target_pose.pose.position.x = float(sys.argv[1])
    req.target_pose.pose.position.y = float(sys.argv[2])
    req.target_pose.pose.position.z = float(sys.argv[3])

    req.target_pose.pose.orientation.x = float(0)
    req.target_pose.pose.orientation.y = float(0)
    req.target_pose.pose.orientation.z = float(0)
    req.target_pose.pose.orientation.w = float(1.0)

    req.move_group.data = 'suction_gripper'
    mrp.wait_for_service()

    res = mrp(req).success.data
    print "Move: " + str(res)
