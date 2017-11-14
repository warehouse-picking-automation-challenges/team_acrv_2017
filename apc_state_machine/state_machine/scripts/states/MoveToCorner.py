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
        rospy.loginfo('Moveing to view')
        m.move_to_named_pose('whole_arm', 'view_checkerboard')

        rospy.sleep(4)

        poses = []
        qt = tf.transformations.quaternion_from_euler(0.0, 0.0, -3.141592653/2)

        for i in range(16):
            rospy.loginfo('GET OBJECT POSE')
            b_req = cartesian_calibration.srv.GetACoorRequest()
            b_req.data_set="measured"
            b_req.object_idx = i
            b_res = self.pose_service.call(b_req)
            p = b_res.object_pose.pose
            # p.orientation.x = qt[0]
            # p.orientation.y = qt[1]
            # p.orientation.z = qt[2]
            # p.orientation.w = qt[3]


            object_global = t.convert_pose(p, self.camera_reference_frame, 'global_xyz_link')
            # object_global.position.z = min(0.80,p.position.z)
            poses.append(object_global)

        m.move_to_named_pose('realsense', 'tool_change_position')
        m.move_to_named_pose('wrist_only', 'sucker')

        for p in poses:

            if p.position.x == 0 and p.position.y == 0 and p.position.z == 0:
                rospy.logerr('Failed to find an object. %s' % i)
                # return 'failed'
                continue

            print("MOVE SUCKER TO ABOVE OBJECT")
            p = t.align_pose_orientation_to_frame(p, 'global_xyz_link', 'sucker_endpoint')

            t.publish_pose_as_transform(p, 'global_xyz_link', 'MOVE HERE', 0.5)
            res = m.move_to_global(p.position.x - 0.02, p.position.y+0.01, p.position.z - 0.02, 'sucker', orientation=p.orientation)

            rospy.sleep(4)


def main(args):
  rospy.init_node('MoveToCorner', anonymous=True)
  ic = MoveToCorner()
  ic.execute()

if __name__ == '__main__':
    main(sys.argv)
