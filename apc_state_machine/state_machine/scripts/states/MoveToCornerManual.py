#!/usr/bin/env python
import rospy
import smach
import std_msgs.msg
import sys
import apc_msgs.srv
import tf
from helpers import movement as m
from helpers import transforms as t
import cartesian_calibration.srv

class MoveToCorner:

    def __init__(self):
        self.camera_reference_frame = 'realsense_wrist_rgb_optical_frame'
        # Wait for the bottle pose service to be available.rgb
        service_name = '/cartesian_calibration/One_Corner'

        # Wait for the bottle pose service to be available.
        rospy.loginfo('Waiting for %s service ...' % service_name)
        try:
            rospy.wait_for_service(service_name, timeout=1)
        except:
            rospy.logerr('Service %s not available. Restart and try again.'
                % service_name)
        self.pose_service = rospy.ServiceProxy(service_name, cartesian_calibration.srv.GetACoor)

    # ==========================================================
    def execute(self):
        # Get the bottle pose from the camera.

        square = 0.1175
        curr = t.current_robot_pose('global_xyz_link', 'gripper_endpoint')
        for i in range(6):
            for j in range(7):
                m.move_to_global(curr.position.x + square*i, curr.position.y + square*j, curr.position.z, 'gripper')
                rospy.sleep(4)


def main(args):
  rospy.init_node('MoveToCorner', anonymous=True)
  ic = MoveToCorner()
  ic.execute()

if __name__ == '__main__':
    main(sys.argv)
