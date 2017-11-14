import rospy
import smach
import smach_ros

import tf

import geometry_msgs.msg as gmsg
from std_msgs.msg import Int16
from helpers import movement as m
from helpers import transforms as t
from helpers.robot_constants import *
from helpers.gripper import *
from helpers import weight as w

from helpers.item_meta import item_meta

class LiftObjectGripper(smach.State):
    def __init__(self, duration=1.0):
        smach.State.__init__(self, outcomes=['succeeded','failed', 'grip_failed', 'try_sucker'], input_keys=['data'], output_keys=['data'],)

    # ==========================================================
    def execute(self, userdata):
        # Only do it once.
        move_objects_mode = userdata.data['move_objects_mode']
        userdata.data['move_objects_mode'] = False

        # Lift the gripper up slowly so that it gets a good grip.
        curr = t.current_robot_pose('global_xyz_link', 'gripper_endpoint')
        res = m.move_to_global(curr.position.x, curr.position.y, max(curr.position.z - 0.05, GRIPPER_MIN_GLOBAL_Z), 'gripper', velo_scale=0.02)


        curr = t.current_robot_pose('global_xyz_link', 'gripper_endpoint')
        # Update current pose with new orientation.
        grip_height = curr.position.z
        curr.position.z = GRIPPER_LIFT_HEIGHT_STRAIGHT
        #t.publish_pose_as_transform(curr, 'global_xyz_link', 'MOVE HERE', 1.0)
        res = m.move_to_global(curr.position.x, curr.position.y, curr.position.z, 'gripper', orientation=curr.orientation, velo_scale=0.5)
        if not res:
            rospy.logerr('Failed to perform a move (lift object gripper)')
            return 'failed'

        hold_gripper()

        scales_topic = userdata.data['picking_from'] + '_scales/weight'

        if move_objects_mode and userdata.data['move_objects_between_bins_mode'] is None:
            mid_x, mid_y = POSITION_LIMITS[userdata.data['picking_from']]['centre']
            if curr.position.x > mid_x:
                curr.position.x -= 0.15
            else:
                curr.position.x += 0.15
            if curr.position.y > mid_y:
                curr.position.y -= 0.15
            else:
                curr.position.y += 0.15
            res, _, _ = m.move_to_global_monitor_weight(curr.position.x, curr.position.y, grip_height - 0.13, 'gripper', scales_topic, velo_scale=0.1)
            return 'grip_failed'

        if userdata.data['weight_before'] == -1:
            rospy.logerr('No pre-pick weight, ignoring weight check')
            return 'succeeded'

        try:
            weight = rospy.wait_for_message(scales_topic, Int16)
        except:
            rospy.logerr('Failed to get weight from %s, ignoring weight check')
            return 'succeeded'

        userdata.data['weight_after'] = weight.data
        dw = userdata.data['weight_before'] - weight.data

        if item_meta[userdata.data['item_to_pick']['label']].get('weight')*1000.0 < 5:
            # We can't do a weight check.
            return 'succeeded'

        if userdata.data['weight_failures'].get(userdata.data['item_to_pick']['label'], 0) > 3 and (len(userdata.data['visible_objects']) < 6 or userdata.data.get('i_think_im_done')):
            # We've failed weight check on this item heaps of times and we're almost finished so just pick it.
            return 'succeeded'

        if dw < 4:
            # We haven't got anything, go back to the camera position and try again.
            rospy.logerr('No detected change in weight, failed grasp.')
            if item_meta[userdata.data['item_to_pick']['label']]['grasp_type'] == 'grip_suck':
                return 'try_sucker'
            userdata.data['last_failed'][userdata.data['item_to_pick']['label']] = 0
            return 'grip_failed'

        if item_meta[userdata.data['item_to_pick']['label']].get('ignore_weight') == True:
            return 'succeeded'

        weight_correct, possible_matches = w.weight_matches(userdata.data['item_to_pick']['label'], dw, userdata.data['visible_objects'], 3)
        if not weight_correct:
            rospy.logerr(possible_matches)
            if userdata.data.get('i_think_im_done', False) == True:
                # Don't do weight reclassifications at the end.
                pass
            elif userdata.data['move_objects_between_bins_mode'] is not None:
                pass
            elif len(possible_matches) == 1 and possible_matches[0][0] < 5:
                # There's only one thing it could actually be!
                if userdata.data['task'] == 'stow' or (userdata.data['task'] == 'pick' and possible_matches[0][1] in userdata.data['wanted_items']):
                    rospy.logerr("I'M CHANGING THE OBJECT TO %s" % possible_matches[0][1])
                    userdata.data['weight_reclassifications'].append( (userdata.data['item_to_pick']['label'], possible_matches[0][1]) )
                    userdata.data['item_to_pick']['label'] = possible_matches[0][1]
                    return 'succeeded'

            # Put the item back down and bail.
            if userdata.data['item_to_pick']['label'] not in userdata.data['weight_failures']:
                userdata.data['weight_failures'][userdata.data['item_to_pick']['label']] = 1
            else:
                userdata.data['weight_failures'][userdata.data['item_to_pick']['label']] += 1
            userdata.data['last_failed'][userdata.data['item_to_pick']['label']] = 0
            rospy.logerr("WEIGHT DOESN'T MATCH: MEASURED: %s, EXPECTED: %s" % (dw, item_meta[userdata.data['item_to_pick']['label']].get('weight')*1000.0))

            # Move towards the centre.
            mid_x, mid_y = POSITION_LIMITS[userdata.data['picking_from']]['centre']
            if curr.position.x > mid_x:
                curr.position.x -= 0.08
            else:
                curr.position.x += 0.08
            if curr.position.y > mid_y:
                curr.position.y -= 0.08
            else:
                curr.position.y += 0.08

            res, _, _ = m.move_to_global_monitor_weight(curr.position.x, curr.position.y, grip_height - 0.13, 'gripper', scales_topic, velo_scale=0.1)
            if not res:
                rospy.logerr('Failed to perform a move')
                return 'failed'

            return 'grip_failed'

        return 'succeeded'
