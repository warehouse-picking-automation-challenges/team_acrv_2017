#! /usr/bin/python

import roslib
import rospy
import moveit_lib.srv as msrv
#import ros_arduino_msgs.srv as ram


NAMED_POSES = ['pre_grasp', 'grasp_approach', 'post_grasp','pre_drop',
                'drop_approach', 'final']

demo_poses = [NAMED_POSES[0],NAMED_POSES[1],NAMED_POSES[0],NAMED_POSES[2],
               NAMED_POSES[3],NAMED_POSES[4], NAMED_POSES[5]]
# demo_poses = ['home']
if __name__ == "__main__":
    #setup the ros stuff
    # not sure what needs to go here

    #iterate through the list of poses for the demo
    mrp = rospy.ServiceProxy('/moveit_lib/move_robot_named',msrv.move_robot_named)
    #mvac = rospy.ServiceProxy('/arduino/digital_write', ram.DigitalWrite)
    #reqv = ram.DigitalWriteRequest()
    while True:
        for pose in demo_poses:
            # reqv.pin = 2
            # reqv.value = not reqv.value
            # mvac.wait_for_service()
            mrp.wait_for_service()
            req = msrv.move_robot_namedRequest()
            req.move_group.data = 'suction_gripper'
            req.named_pose.data = pose
            print "Moving CartMan to " + pose
            res = mrp(req)
            # resv = mvac(reqv)
            # print "Vacuum: " + str(resv.success)
            print "Move: " + str(res.success.data)
