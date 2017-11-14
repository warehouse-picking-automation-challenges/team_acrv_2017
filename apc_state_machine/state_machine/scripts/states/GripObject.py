import rospy
import smach
import std_msgs.msg
import bottle_detector.srv

from std_msgs.msg import Int16

import apc_msgs.srv

from tf import transformations as tft

from helpers import suction
from helpers import movement as m
from helpers import transforms as t
from helpers import gripper

from helpers.robot_constants import *
from helpers.item_meta import item_meta


class GripObject(smach.State):
    def __init__(self):
        smach.State.__init__(self, input_keys=['data'], output_keys=['data'],
                             outcomes=['succeeded', 'failed', 'grip_failed', 'try_sucker'])

    def execute(self, userdata):
        gripper.open_gripper()

        position_limits = POSITION_LIMITS[userdata.data['picking_from']]['gripper']

        # Get the weight before the pick
        scales_topic = userdata.data['picking_from'] + '_scales/weight'
        if item_meta[userdata.data['item_to_pick']['label']]['grasp_type'] != 'suck_grip':
            # If we've just tried sucking, then don't re-weigh
            try:
                weight = rospy.wait_for_message(scales_topic, Int16, 2.0)
                userdata.data['weight_before'] = weight.data
            except:
                rospy.logerr('Unable to read weight from %s' % scales_topic)
                userdata.data['weight_before'] = -1
                if item_meta[userdata.data['item_to_pick']['label']]['grasp_point_type'] == 'rgb_centroid':
                    rospy.logerr("Aborting RGB Centroid pick because there aren't any scales.")
                    return 'grip_failed'

        # No way to detect failure yet, so just use one grasp point.
        object_global = userdata.data['item_to_pick']['grasp_poses'][0]

        object_global.position.x += 0.015
        object_global.position.y -= 0.03

        object_global.position.x = max(object_global.position.x, position_limits['x_min'])
        object_global.position.x = min(object_global.position.x, position_limits['x_max'])
        object_global.position.y = max(object_global.position.y, position_limits['y_min'])
        object_global.position.y = min(object_global.position.y, position_limits['y_max'])
        object_global.position.z = min(object_global.position.z, position_limits['z_max'])

        t.publish_pose_as_transform(object_global, 'global_xyz_link', 'MOVE HERE', 0.5)

        rospy.loginfo('MOVE TO TOOL CHANGE HEIGHT')
        curr = t.current_robot_pose('global_xyz_link', 'realsense_endpoint')
        # res = m.move_to_global(curr.position.x, curr.position.y, TOOL_CHANGE_REALSENSE_GLOBAL_Z, 'realsense')
        res = m.move_to_global(min(object_global.position.x, REALSENSE_MAX_GLOBAL_X), max(min(object_global.position.y, SUCKER_MAX_Y_STRAIGHT) - 0.02, SUCKER_MIN_GLOBAL_Y + 0.02) - SUCKER_REALSENSE_Y_OFFSET, TOOL_CHANGE_REALSENSE_GLOBAL_Z, 'realsense')
        if not res:
            return 'failed'

        rospy.loginfo('SELECTING GRIPPER')
        res = m.move_to_named_pose('wrist_only', 'gripper')
        if not res:
            return 'failed'

        object_global.position.z += 0.015
        pca_yaw = userdata.data['item_to_pick']['pca_yaw']
        q = (pca_yaw.x, pca_yaw.y, pca_yaw.z, pca_yaw.w)
        e = list(tft.euler_from_quaternion(q))
        if e[2] > 1.57:
            e[2] -= 3.14
        if e[2] < -1.57:
            e[2] += 3.14
        if e[2] > 1.55: # Stop gimbal lock and weird joint configurations.
            e[2] = 1.55
        if e[2] < -1.55:
            e[2] = -1.55

        q = tft.quaternion_from_euler(0, 0, e[2])
        object_global.orientation.x = q[0]
        object_global.orientation.y = q[1]
        object_global.orientation.z = q[2]
        object_global.orientation.w = q[3]

        rospy.loginfo('MOVE GRIPPER TO ABOVE OBJECT')
        res = m.move_to_global(object_global.position.x, object_global.position.y, max(object_global.position.z - 0.2, GRIPPER_MIN_GLOBAL_Z), 'gripper', orientation=object_global.orientation)
        if not res:
            return 'failed'

        rospy.loginfo('MOVING TO OBJECT')
        v_scale = 0.1
        if item_meta[userdata.data['item_to_pick']['label']]['grasp_point_type'] == 'rgb_centroid':
            # Don't trust the depth.
            v_scale = 0.1
        if userdata.data['item_to_pick']['label'] == 'mesh_cup':
            v_scale = 0.05
        (res, stopped, bail) = m.move_to_global_monitor_weight(object_global.position.x, object_global.position.y,
                               min(object_global.position.z, GRIPPER_MAX_GLOBAL_Z), 'gripper', scales_topic, velo_scale=v_scale, orientation=object_global.orientation)
        if not res:
            return 'failed'

        if bail:
            m.move_relative(0, 0, -0.02, 'gripper_endpoint', 'gripper')
        # rospy.sleep(0.5)

        rospy.loginfo('CLOSE THE GRIPPER')
        gripper.close_gripper()
        rospy.sleep(0.5)

        return('succeeded')
