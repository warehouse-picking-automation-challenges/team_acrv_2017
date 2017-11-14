import rospy
import smach
import std_msgs.msg

import apc_msgs.srv

from helpers import suction
from helpers import movement as m
from helpers import transforms as t
from helpers import gripper

from helpers.robot_constants import *


class MoveToStowTote(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'], input_keys=['data'], output_keys=['data'])

    def execute(self, userdata):
        rospy.loginfo('MOVE TO STOW TOTE')
        userdata.data['camera_location'] = 'stow_tote'
        userdata.data['picking_from'] = 'stow_tote'
        userdata.data['visible_objects'] = userdata.data['item_locations']['tote']['contents']
        if len(userdata.data['visible_objects']) == 0:
            # We think we're done, but re-init all of the items and do a final check.
            rospy.logerr('I think we\'re done.')
            userdata.data['i_think_im_done'] = True
            userdata.data['visible_objects'] = list(userdata.data['all_items'])
        res = m.move_to_named_pose('realsense', 'realsense_above_stow_tote')
        if not res:
            return 'failed'

        return 'succeeded'
