import rospy
import smach

from helpers import movement as m
from helpers import transforms as t
from helpers.robot_constants import *

class TestJoints(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

    def execute(self, userdata):
        rospy.loginfo('MOVE TO SAFE HEIGHT')
        curr = t.current_robot_pose('global_xyz_link', 'realsense_endpoint')
        res = m.move_to_global(curr.position.x, min(curr.position.y, TOOL_CHANGE_SAFE_Y), TOOL_CHANGE_REALSENSE_GLOBAL_Z, 'realsense')
        if not res:
            return 'failed'

        rospy.loginfo('TESTING THAT THE WRIST MOVES')
        res = m.move_to_named_pose('wrist_only', 'test_joints')
        if not res:
            rospy.logerr('Wrist Failed to Move')
            return 'failed'

        return 'succeeded'
