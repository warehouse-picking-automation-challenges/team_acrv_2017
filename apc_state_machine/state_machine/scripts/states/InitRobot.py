import rospy
import smach
import std_msgs.msg
import bottle_detector.srv

import apc_msgs.srv

from helpers import suction
from helpers import movement as m
from helpers import transforms as t
from helpers import gripper

from helpers.robot_constants import *


class InitRobot(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'], input_keys=['data'], output_keys=['data'])

    def execute(self, userdata):
        suction.set_suck_level(0)
        suction.pump_off()
        gripper.open_gripper()
        rospy.sleep(0.5)

        userdata.data['move_objects_mode'] = False
        userdata.data['move_objects_between_bins_mode'] = None

        rospy.loginfo('MOVE TO TOOL CHANGE HEIGHT')
        curr = t.current_robot_pose('global_xyz_link', 'realsense_endpoint')

        # Don't hit the top bar.
        if curr.position.y >= TOOL_CHANGE_SAFE_Y:
            curr.position.y = TOOL_CHANGE_SAFE_Y

        # Don't hit the down bars.
        if curr.position.y >= TOOL_CHANGE_DANGEROUS_Y:
            # We might hit the front frame so move away from it.
            if curr.position.x >= TOOL_CHANGE_X_MAX:
                curr.position.x = TOOL_CHANGE_X_MAX
            if curr.position.x <= TOOL_CHANGE_X_MIN:
                curr.position.x = TOOL_CHANGE_X_MIN

        res = m.move_to_global(curr.position.x, min(curr.position.y, TOOL_CHANGE_SAFE_Y), TOOL_CHANGE_REALSENSE_GLOBAL_Z, 'realsense')
        if not res:
            return 'failed'

        rospy.loginfo('CHANGE TO NEUTRAL WRIST NEUTRAL')
        res = m.move_to_named_pose('wrist_only', 'neutral')
        if not res:
            return 'failed'

        return 'succeeded'
