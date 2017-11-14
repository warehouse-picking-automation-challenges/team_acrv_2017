import rospy
import smach
import std_msgs.msg

import apc_msgs.srv

from helpers import suction
from helpers import movement as m
from helpers import transforms as t
from helpers import gripper

from helpers.robot_constants import *


class MoveToStorage(smach.State):
    def __init__(self):
        self.visible_objects = {}
        smach.State.__init__(self, input_keys=['data'], output_keys=['data'], outcomes=['succeeded', 'failed', 'repeat'])

        self.a_all_failed = 0
        self.b_all_failed = 0

    def choose(self, bin_id, userdata):
        rospy.loginfo('MOVE TO STORAGE %s' % bin_id)

        camera_locations = ['storage_%s', 'storage_%s_backup_1', 'storage_%s_backup_2']
        poses = ['realsense_above_storage_system_%s', 'realsense_above_storage_system_%s_backup_1', 'realsense_above_storage_system_%s_backup_2']

        userdata.data['chosen_bin'] = bin_id
        userdata.data['camera_location'] = camera_locations[len(userdata.data['failed_views'][bin_id])] % bin_id
        userdata.data['picking_from'] = 'storage_' + bin_id
        userdata.data['visible_objects'] = self.visible_objects[bin_id]

        res = m.move_to_named_pose('realsense', poses[len(userdata.data['failed_views'][bin_id])] % bin_id)
        if not res:
            return 'failed'
        else:
            return 'succeeded'


    def update_ignored(self, userdata):
        last_failed = userdata.data['last_failed']
        to_remove = []
        for l in last_failed:
            last_failed[l] += 1
            if last_failed[l] > 3:
                to_remove.append(l)
        for l in to_remove:
            del last_failed[l]


    def execute(self, userdata):
        # Decide which tote has the most items.
        items_in_A = 0
        items_in_B = 0
        items_in_A_inc_ignored = 0
        items_in_B_inc_ignored = 0
        self.visible_objects = {}
        last_failed = userdata.data['last_failed']
        ignored_objects = 0
        for i in userdata.data['wanted_items']:
            for b in userdata.data['item_locations']['bins']:
                if i in b['contents']:
                    if b['bin_id'] == 'A':
                        items_in_A_inc_ignored += 1
                        if i in last_failed:
                            ignored_objects += 1
                        else:
                            items_in_A += 1
                    elif b['bin_id'] == 'B':
                        items_in_B_inc_ignored += 1
                        if i in last_failed:
                            ignored_objects += 1
                        else:
                            items_in_B += 1
                self.visible_objects[b['bin_id']] = b['contents']

        if len(userdata.data['wanted_items']) == 0:
            return 'failed'  # we're done

        rospy.logerr("MTS: A Failed Views %s" % len(userdata.data['failed_views']['A']))
        rospy.logerr("MTS: B Failed Views %s" % len(userdata.data['failed_views']['B']))

        if items_in_A_inc_ignored == 0:
            self.a_all_failed = 0
            userdata.data['failed_views']['A'] = []

        if items_in_B_inc_ignored == 0:
            self.b_all_failed = 0
            userdata.data['failed_views']['B'] = []

        if len(userdata.data['failed_views']['A']) == 3 and len(userdata.data['failed_views']['B']) == 3:
            # We've failed everywhere, reset.
            rospy.logerr("MTS: 0")
            self.a_all_failed += 1
            self.b_all_failed += 1
            userdata.data['failed_views'] = {'A': [], 'B': []}

        # Should we go into move_objects_mode?
        if self.a_all_failed > 0 and self.b_all_failed > 0:
            rospy.logerr("MTS: 1")
            # Both storage systems have failed every view.
            rospy.logerr("Going into move items mode")
            userdata.data['last_failed'] = {}
            userdata.data['move_objects_mode'] = True
            if items_in_A_inc_ignored > items_in_B_inc_ignored:
                self.a_all_failed = 0
                userdata.data['failed_views']['A'] = []
                return self.choose('A', userdata)
            else:
                self.b_all_failed = 0
                userdata.data['failed_views']['B'] = []
                return self.choose('B', userdata)

        if items_in_A == 0 and self.b_all_failed > 0:
            rospy.logerr("MTS: 2")
            rospy.logerr("Going into move items mode")
            self.b_all_failed = 0
            userdata.data['move_objects_mode'] = True
            if items_in_A_inc_ignored == 0:
                userdata.data['move_objects_between_bins_mode'] = 'A'
            userdata.data['failed_views']['B'] = []
            userdata.data['last_failed'] = {}
            return self.choose('B', userdata)

        if items_in_B == 0 and self.a_all_failed > 0:
            rospy.logerr("MTS: 3")
            rospy.logerr("Going into move items mode")
            self.a_all_failed = 0
            userdata.data['move_objects_mode'] = True
            if items_in_B_inc_ignored == 0:
                userdata.data['move_objects_between_bins_mode'] = 'B'
            userdata.data['failed_views']['A'] = []
            userdata.data['last_failed'] = {}
            return self.choose('A', userdata)

        # First check if there's only items in one, then we don't have a choice.
        if items_in_A > 0 and items_in_B == 0:
            rospy.logerr("MTS: 4")
            # We have to choose A
            if items_in_B_inc_ignored: # Stop us getting in an endless loop of a single bin.
                self.update_ignored(userdata)
            if len(userdata.data['failed_views']['A']) == 3:
                self.a_all_failed += 1
                userdata.data['failed_views']['A'] = []
            return self.choose('A', userdata)

        if items_in_B > 0 and items_in_A == 0:
            rospy.logerr("MTS: 5")
            # We have to choose B
            if items_in_A_inc_ignored:  # Stop us getting in an endless loop of a single bin.
                self.update_ignored(userdata)
            if len(userdata.data['failed_views']['B']) == 3:
                self.b_all_failed += 1
                userdata.data['failed_views']['B'] = []
            return self.choose('B', userdata)

        # Check if we've already fully failed at one, if so move onto the other.
        if len(userdata.data['failed_views']['B']) == 3:
            rospy.logerr("MTS: 6")
            self.update_ignored(userdata)
            if items_in_A_inc_ignored == 0:
                return 'repeat'
            if self.a_all_failed > 0:
                self.a_all_failed = 0
                userdata.data['failed_views']['A'] = []
                userdata.data['last_failed'] = {}
                rospy.logerr("Going into move items mode")
                userdata.data['move_objects_mode'] = True
            return self.choose('A', userdata)

        elif len(userdata.data['failed_views']['A']) == 3:
            rospy.logerr("MTS: 7")
            self.update_ignored(userdata)
            if items_in_B_inc_ignored == 0:
                return 'repeat'
            if self.b_all_failed > 0:
                self.b_all_failed = 0
                userdata.data['failed_views']['B'] = []
                userdata.data['last_failed'] = {}
                rospy.logerr("Going into move items mode")
                userdata.data['move_objects_mode'] = True
            return self.choose('B', userdata)

        # Both are ok, choose the one with the most items
        if items_in_A_inc_ignored >= items_in_B_inc_ignored:
            rospy.logerr("MTS: 8")
            return self.choose('A', userdata)
        else:
            rospy.logerr("MTS: 9")
            return self.choose('B', userdata)

        # Never reach this, hopefully
        return 'failed'
