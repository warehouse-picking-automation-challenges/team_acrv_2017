import rospy
import smach
import smach_ros

import tf
import tf.transformations as tft
import random

from geometry_msgs.msg import Quaternion
import geometry_msgs.msg as gmsg
import helpers.movement as m
from states.helpers import suction
from states.helpers import gripper
from helpers import transforms as t
from helpers.robot_constants import *
from helpers import json_functions
from apc_msgs.srv import CheckGrippedItem
from apc_msgs.srv import ObjectPlacementPoseFromCloud, ObjectPlacementPoseFromCloudRequest

from sensor_msgs.msg import Image
from std_msgs.msg import String, Int16

from helpers.item_meta import item_meta

import cv_bridge
import cv2
import datetime

class DropObjectGripper(smach.State):
    def __init__(self, duration=1.0):
        smach.State.__init__(self, input_keys=['data'], output_keys=['data'], outcomes=['succeeded','failed'])
        self.file_path = '/home/apc-cameras/APC_JSON_DATA/'

        free_space_service_name = '/object_placement_pose_from_cloud'
        rospy.loginfo('Waiting for %s service ...' % free_space_service_name)
        try:
            rospy.wait_for_service(free_space_service_name, timeout=1)
        except:
            rospy.logerr('Service %s not available. Restart and try again.' % free_space_service_name)
        self.free_space_service = rospy.ServiceProxy(free_space_service_name, ObjectPlacementPoseFromCloud)

        self.crop_pub = rospy.Publisher('/state_machine/cropped_image', Image)

    # ==========================================================
    def execute(self, userdata):
        # wait for the specified duration or until we are asked to preempt
        curr = t.current_robot_pose('global_xyz_link', 'gripper_endpoint')
        rospy.loginfo('MOVE UP TO DROP POSITION')

        into_bin = 'B' # Default
        weight_before = -1

        drop_orientation = Quaternion() # Default straight
        drop_orientation.x = 0
        drop_orientation.y = 0
        drop_orientation.z = 0
        drop_orientation.w = 1

        x_pos = None
        y_pos = None
        z_pos = None

        if userdata.data['task'] == 'stow':
            #bins = ['storage_B', 'storage_A']
            bins = ['storage_B']  # Only B for finals.

        elif userdata.data['task'] == 'pick':
            if userdata.data['move_objects_between_bins_mode'] is not None:
                bins = ['storage_' + userdata.data['move_objects_between_bins_mode']]
            else:
                l = userdata.data['item_to_pick']['label']
                box_id = None
                for o in userdata.data['order']:
                    if l in o['contents']:
                        box_id = o['size_id']
                bins = [box_id]
                if box_id is None:
                    rospy.logerr("NO BOX FOR ITEM")
                    return 'failed'

        else:
            return 'failed'

        for bin_id in bins:
            # Try and find free space based on the point cloud.
            # Try both bins.
            if bin_id not in userdata.data['empty_space_capture']:
                continue
            pc = userdata.data['empty_space_capture'][bin_id].get('point_cloud', None)
            if pc is None:
                # Make sure we actually have one.
                continue

            req = ObjectPlacementPoseFromCloudRequest()
            req.cropped_cloud = userdata.data['empty_space_capture'][bin_id]['point_cloud']
            req.camera_to_world = userdata.data['empty_space_capture'][bin_id]['pose']
            l = userdata.data['item_to_pick']['label']

            dimensions = item_meta[l]['dimensions']
            dimensions.sort()
            req.object_longest_side_length.data = dimensions[2]
            req.object_middlest_side_length.data = dimensions[1]
            req.object_shortest_side_length.data = dimensions[0]
            req.resolution.data = 0.01

            res = self.free_space_service.call(req)

            if not res.success.data:
                continue

            x_pos = res.x.data
            y_pos = res.y.data
            z_pos = res.z.data - 0.13

            if res.degrees_to_global_y.data > 89:
                res.degrees_to_global_y.data = 89
            if res.degrees_to_global_y.data < -89:
                res.degrees_to_global_y.data = -89

            tq = tft.quaternion_from_euler(0, 0, res.degrees_to_global_y.data * 3.14/180.0)
            drop_orientation.x = tq[0]
            drop_orientation.y = tq[1]
            drop_orientation.z = tq[2]
            drop_orientation.w = tq[3]

            t.publish_tf_quaterion_as_transform((x_pos, y_pos, z_pos), tq, 'global_xyz_link', 'DROP_HERE', 0.5)
            userdata.data['empty_space_capture'][bin_id] = None
            break

        if userdata.data['task'] == 'stow':
            into_bin = bin_id[-1]
            if x_pos is None:
                rospy.logerr('Couldn\'t find space to place.')
                x_pos = 0.350
                y_pos = 0.440
                z_pos = 0.900
                into_bin = 'B'
                bin_id = 'storage_B'
                userdata.data['empty_space_capture']['storage_B'] = None
            if item_meta[userdata.data['item_to_pick']['label']].get('ignore_weight') == True:
                rospy.logerr("Overriding drop point for floppy item")
                x_pos = 0.350
                y_pos = 0.210
                z_pos = 0.800
                into_bin = 'B'
                bin_id = 'storage_B'
                userdata.data['empty_space_capture']['storage_B'] = None

            position_limits = POSITION_LIMITS['storage_' + into_bin]['gripper']
            x_pos = max(x_pos, position_limits['x_min'])
            x_pos = min(x_pos, position_limits['x_max'])
            y_pos = max(y_pos, position_limits['y_min'])
            y_pos = min(y_pos, position_limits['y_max'])

            scales_topic = bin_id + '_scales/weight'
            try:
                weight = rospy.wait_for_message(scales_topic, Int16, 2.0)
                weight_before = weight.data
            except:
                rospy.logerr('Unable to read weight from %s' % scales_topic)
                weight_before = -1


        elif userdata.data['task'] == 'pick' and userdata.data['move_objects_between_bins_mode'] is None:
            if x_pos is None:
                rospy.logerr('Couldn\'t find space to place.')
                box = userdata.data['boxes'][box_id]
                x_pos = (box['left'] + box['right'])/2
                y_pos = (box['top'] + box['bottom'])/2
                z_pos = 0.95
                yaw = 0
                userdata.data['empty_space_capture'][box_id] = None
            if item_meta[userdata.data['item_to_pick']['label']].get('ignore_weight') == True:
                rospy.logerr('Overriding drop point for floppy item')
                box = userdata.data['boxes'][box_id]
                x_pos = (box['left'] + box['right'])/2
                y_pos = box['top'] - 0.02
                z_pos = 0.9
                yaw = 0
                userdata.data['empty_space_capture'][box_id] = None

            x_pos = max(x_pos, SUCKER_MIN_GLOBAL_X)
            x_pos = min(x_pos, SUCKER_MAX_GLOBAL_X)
            y_pos = min(y_pos, SUCKER_MAX_Y_STRAIGHT)

        elif userdata.data['task'] == 'pick' and userdata.data['move_objects_between_bins_mode'] is not None:
            drop_pos = POSITION_LIMITS['storage_' + userdata.data['move_objects_between_bins_mode']]['centre']
            x_pos = drop_pos[0]
            y_pos = drop_pos[1]
            z_pos = 0.9
            into_bin = userdata.data['move_objects_between_bins_mode']

        res = m.move_to_global(x_pos, y_pos, GRIPPER_LIFT_HEIGHT_STRAIGHT + 0.01, 'gripper', orientation=drop_orientation)
        if not res:
            rospy.logerr('Failed to move to tote')
            return 'failed'

        curr = t.current_robot_pose('global_xyz_link', 'gripper_endpoint')
        pose = gmsg.Pose()
        pose.position.x = curr.position.x
        pose.position.y = curr.position.y
        pose.position.z = z_pos
        pose.orientation = drop_orientation

        if userdata.data['task'] == 'stow':
            # We can use the scales to check our movement.
            scales_topic = 'storage_%s_scales/weight' % into_bin
            (res, stopped, bail) = m.move_to_global_monitor_weight(pose.position.x, pose.position.y, pose.position.z, 'gripper', scales_topic, orientation=pose.orientation, velo_scale=0.4)
            if not res:
                return 'failed'
            if bail:
                m.move_relative(0, 0, -0.02, 'gripper_endpoint', 'gripper')

        else:
            # No scales, perform a normal movement
            res = m.move_to_global(pose.position.x, pose.position.y, pose.position.z, 'gripper', orientation=pose.orientation, velo_scale=0.4)
            if not res:
                rospy.logerr('Failed to perform a move')
                return 'failed'

        res = gripper.open_gripper()
        if not res:
            rospy.logerr('Failed to open gripper')
            return 'failed'

        rospy.sleep(1.0)

        userdata.data['last_drop_bin'] = into_bin

        if weight_before != -1 and userdata.data['task'] == 'stow':
            scales_topic = bin_id + '_scales/weight'
            try:
                weight = rospy.wait_for_message(scales_topic, Int16, 2.0)
                if (weight.data - weight_before) < 5:
                    rospy.logerr('No weight change, we must have dropped it.')
                    userdata.data['last_failed'][userdata.data['item_to_pick']['label']] = 0

                    # Undo an added weight reclassification if it happened.
                    if userdata.data['weight_reclassifications']:
                        if userdata.data['weight_reclassifications'][-1][1] == userdata.data['item_to_pick']['label']:
                            userdata.data['weight_reclassifications'] = userdata.data['weight_reclassifications'][:-1]


                    # Don't update the item locations.
                    return 'succeeded'
                rospy.logerr("SUCCESSFUL DROP")
            except:
                rospy.logerr('Unable to read weight from %s' % scales_topic)

        if userdata.data['task'] == 'pick' and userdata.data['weight_before'] != -1:
            # Make sure the weight in the source storage system is still down.
            scales_topic = userdata.data['picking_from'] + '_scales/weight'
            try:
                weight = rospy.wait_for_message(scales_topic, Int16)
            except:
                rospy.logerr('Failed to get weight from %s, ignoring weight check')
            else:
                dw = userdata.data['weight_before'] - weight.data
                if dw < 4:
                    # The item must have fallen out before we got to the box.
                    # Don't update the item locations
                    rospy.logerr('The source storage system didn\'t change weight, we probably dropped the item')
                    return 'succeeded'

        # Update item locations.
        if userdata.data['task'] == 'stow':
            cur_label = userdata.data['item_to_pick']['label']
            try:
                userdata.data['item_locations']['tote']['contents'].remove(cur_label)
            except ValueError:
                # The item wasn't meant to be in the tote, maybe we thought we already moved it.
                # Amend the list.
                for b in userdata.data['item_locations']['bins']:
                    if cur_label in b['contents']:
                        b['contents'].remove(cur_label)
            for b in userdata.data['item_locations']['bins']:
                if b['bin_id'] == into_bin:
                    b['contents'].append(cur_label)

        elif userdata.data['task'] == 'pick' and userdata.data['move_objects_between_bins_mode'] is None:
            cur_label = userdata.data['item_to_pick']['label']
            for b in userdata.data['item_locations']['bins']:
                if cur_label in b['contents']:
                    b['contents'].remove(cur_label)
            for b in userdata.data['item_locations']['boxes']:
                if b['size_id'] == box_id:
                    b['contents'].append(cur_label)

        elif userdata.data['task'] == 'pick' and userdata.data['move_objects_between_bins_mode'] is not None:
            cur_label = userdata.data['item_to_pick']['label']
            for b in userdata.data['item_locations']['bins']:
                if cur_label in b['contents']:
                    b['contents'].remove(cur_label)
                if b['bin_id'] == userdata.data['move_objects_between_bins_mode']:
                    b['contents'].append(cur_label)
            userdata.data['move_objects_between_bins_mode'] = None

        json_functions.save_file(userdata.data['task'], 'item_location_file_out.json', userdata.data['item_locations'])

        return 'succeeded'
