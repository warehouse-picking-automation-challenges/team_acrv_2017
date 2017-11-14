import rospy
import smach
import smach_ros

from tf import transformations as tft

import geometry_msgs.msg as gmsg
from std_msgs.msg import Int16
from helpers import movement as m
from helpers import transforms as t
from helpers import suction
from helpers.robot_constants import *
from helpers.item_meta import item_meta
from helpers import weight as w

class LiftObjectSucker(smach.State):
    def __init__(self, duration=1.0):
        smach.State.__init__(self, outcomes=['succeeded', 'failed', 'suck_failed', 'try_gripper', 'secondary_check'], input_keys=['data'], output_keys=['data'])

    # ==========================================================
    def execute(self, userdata):
        # Only do it once.
        move_objects_mode = userdata.data['move_objects_mode']
        userdata.data['move_objects_mode'] = False

        rospy.loginfo('LIFTING OBJECT WITH SUCKER')
        curr = t.current_robot_pose('global_xyz_link', 'sucker_endpoint')
        # Update current pose with new orientation.

        curr.position.x = min(curr.position.x, SUCKER_MAX_GLOBAL_X - 0.01)
        curr.position.y = min(curr.position.y, SUCKER_MAX_Y_STRAIGHT)
        curr.position.y = max(curr.position.y, SUCKER_MIN_GLOBAL_Y + 0.01)
        suck_height = curr.position.z
        curr.position.z = SUCKER_LIFT_HEIGHT
        #t.publish_pose_as_transform(curr, 'global_xyz_link', 'MOVE HERE', 1.0)

        co = curr.orientation
        q = [co.x, co.y, co.z, co.w]
        e = tft.euler_from_quaternion(q)
        q = tft.quaternion_from_euler(0, 0, e[2])

        qm = gmsg.Quaternion()
        qm.x = 0
        qm.y = 0
        qm.z = 0
        qm.w = 1

        v_scale = 0.3
        if item_meta[userdata.data['item_to_pick']['label']]['weight'] > 0.5:
            # Heavy Item
            v_scale = 0.15

        res = m.move_to_global(curr.position.x, curr.position.y, curr.position.z, 'sucker', orientation=qm, velo_scale=v_scale)
        if not res:
            rospy.logerr('Failed to perform a move')
            return 'failed'

        if not suction.check_pressure_sensor():
            userdata.data['last_failed'][userdata.data['item_to_pick']['label']] = 0
            return 'suck_failed'

        scales_topic = userdata.data['picking_from'] + '_scales/weight'
        if move_objects_mode and userdata.data['move_objects_between_bins_mode'] is None:
            mid_x, mid_y = POSITION_LIMITS[userdata.data['picking_from']]['centre']
            if curr.position.x > mid_x:
                curr.position.x -= 0.15
            else:
                curr.position.x += 0.15
            if curr.position.y > mid_y:
                curr.position.y -= 0.2
            else:
                curr.position.y += 0.2
            res, _, _ = m.move_to_global_monitor_weight(curr.position.x, curr.position.y, suck_height - 0.13, 'sucker', scales_topic, velo_scale=0.2)
            return 'suck_failed'

        if userdata.data['weight_before'] < 0:
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
            rospy.logerr('I am ignoring the weight check since it has failed too many times.')
            return 'succeeded'

        if dw < 4:
            # We haven't got anything, go back to the camera position and try again.
            rospy.logerr('No detected change in weight, failed grasp.')
            userdata.data['last_failed'][userdata.data['item_to_pick']['label']] = 0
            return 'suck_failed'

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
                    suck_level = item_meta[possible_matches[0][1]].get('suction_pressure', 3)
                    suction.set_suck_level(suck_level)
                    return 'succeeded'

            elif len(possible_matches) > 1 and userdata.data['task'] == 'stow':
                userdata.data['double_check_objects'] = [i[1] for i in possible_matches]
                suck_pos = t.current_robot_pose('global_xyz_link', 'sucker_endpoint')
                suck_pos.position.z = suck_height
                userdata.data['original_pick_position'] = suck_pos
                return 'secondary_check'


            # Put the item back down and bail.
            userdata.data['last_failed'][userdata.data['item_to_pick']['label']] = 0
            if userdata.data['item_to_pick']['label'] not in userdata.data['weight_failures']:
                userdata.data['weight_failures'][userdata.data['item_to_pick']['label']] = 1
            else:
                userdata.data['weight_failures'][userdata.data['item_to_pick']['label']] += 1
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

            res, _, _ = m.move_to_global_monitor_weight(curr.position.x, curr.position.y, suck_height - 0.13, 'sucker', scales_topic, velo_scale=0.1)
            if not res:
                rospy.logerr('Failed to perform a move')
                return 'failed'

            return 'suck_failed'

        return 'succeeded'
