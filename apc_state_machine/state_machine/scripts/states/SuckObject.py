import rospy
import smach

from tf import transformations as tft
from std_msgs.msg import Int16
from geometry_msgs.msg import Quaternion

from helpers import suction
from helpers import movement as m
from helpers import transforms as t

from helpers.robot_constants import *

from helpers.suction import set_suck_level
from helpers.item_meta import item_meta

import numpy as np


def decide_angled(grasp_pose, position_limits):
    # angle to vertical
    pt = [grasp_pose.position.x, grasp_pose.position.y, grasp_pose.position.z]
    pq = [grasp_pose.orientation.x, grasp_pose.orientation.y, grasp_pose.orientation.z, grasp_pose.orientation.w]
    R = tft.quaternion_matrix(pq)
    z_unit = R[0:3,2]
    val = z_unit[2]
    a = np.arccos(val/np.linalg.norm(z_unit))

    if abs(a) < 0.5:
        # The angle isn't steep enough so no point.
        return False

    edge_offset = 0.1
    # Check the Roll and Pitch are in the right directions given its location.
    if grasp_pose.position.x < position_limits['x_min'] + edge_offset and R[0, 2] > 0.1:
        rospy.logerr('1 POS X IS %s AND E1 IS %s' % (grasp_pose.position.x, R[0, 2]))
        return False
    if grasp_pose.position.x > position_limits['x_max'] - edge_offset and R[0, 2] < -0.1:
        rospy.logerr('2 POS X IS %s AND E1 IS %s' % (grasp_pose.position.x, R[0, 2]))
        return False
    if grasp_pose.position.y < position_limits['y_min'] + edge_offset and R[1, 2] > 0.1:
        rospy.logerr('1 POS Y IS %s AND E0 IS %s' % (grasp_pose.position.y, R[1, 2]))
        return False
    if grasp_pose.position.y > position_limits['y_max'] - edge_offset and R[1, 2] < -0.1:
        rospy.logerr('2 POS Y IS %s AND E0 IS %s' % (grasp_pose.position.y, R[1, 2]))
        return False

    return True


class SuckObject(smach.State):
    def __init__(self):
        smach.State.__init__(self, input_keys=['data'], output_keys=['data'],
                             outcomes=['succeeded', 'failed', 'suck_failed', 'try_gripper'])

    def execute(self, userdata):

        first_attamept = True
        label = userdata.data['item_to_pick']['label']
        suck_level = item_meta[label].get('suction_pressure', 3)
        set_suck_level(suck_level)

        USE_ANGLED = False

        position_limits = POSITION_LIMITS[userdata.data['picking_from']]['sucker']

        # Get the tote weight before the pick
        scales_topic = userdata.data['picking_from'] + '_scales/weight'
        if item_meta[userdata.data['item_to_pick']['label']]['grasp_type'] != 'grip_suck':
            # If we've just tried sucking, then don't re-weigh
            try:
                weight = rospy.wait_for_message(scales_topic, Int16, 2.0)
                userdata.data['weight_before'] = weight.data
            except:
                rospy.logerr('Unable to read weight from %s' % scales_topic)
                userdata.data['weight_before'] = -1
                if item_meta[userdata.data['item_to_pick']['label']]['grasp_point_type'] == 'rgb_centroid':
                    rospy.logerr("Aborting RGB Centroid pick because there aren't any scales.")
                    return 'suck_failed'

        for object_global in userdata.data['item_to_pick']['grasp_poses']:

            USE_ANGLED = decide_angled(object_global, position_limits)

            object_global.position.x += 0.015
            object_global.position.y -= 0.03

            if USE_ANGLED:
                rospy.logerr('ANGLED PICK')

                # Make sure we're inside the tote.
                edge_offset = 0.0
                object_global.position.x = max(object_global.position.x, position_limits['x_min'] - edge_offset)
                object_global.position.x = min(object_global.position.x, position_limits['x_max'] + edge_offset)
                object_global.position.y = max(object_global.position.y, position_limits['y_min'] - edge_offset)
                object_global.position.y = min(object_global.position.y, position_limits['y_max'] + edge_offset)

                object_global.position.z -= 0.02  # The endpoint is half way through the suction cup, so move up slightly

                #t.publish_pose_as_transform(object_global, 'global_xyz_link', 'ORIGINAL', seconds=2)

                translation = [object_global.position.x, object_global.position.y, object_global.position.z]
                qm = object_global.orientation
                q = (qm.x, qm.y, qm.z, qm.w)

                R = tft.quaternion_matrix(q)
                P = tft.projection_matrix([0, 0, 0], [0, 0, 1])  # XY plane
                pose_vec = R[:, 2]  # Z portion of matrix.
                proj_vec = np.dot(P, pose_vec)[:3]  # Projected onto XY plane as a unit vector.
                proj_vec_unit = tft.unit_vector(proj_vec)
                y_unit = [0, 1, 0]  # Unit vector in the y-direction.
                yaw_angle = np.arccos(np.dot(proj_vec_unit, y_unit))  # Angle to the grasp around Z.
                pitch_angle = np.arcsin(tft.vector_norm(proj_vec))  # Angle to tilt the suction cup
                if pitch_angle > 1.2:
                    pitch_angle = 1.2

                # Only get absolute angle from above.  Check the direction of the vector.
                if R[0, 2] > 0:
                    yaw_angle = -1 * yaw_angle

                # Create a quaternion with the right angles (note rotations are applied in the rotated frame in zyx order)
                q = tft.quaternion_from_euler(yaw_angle, 0, -1 * pitch_angle, 'rzyx')

                if yaw_angle > 1.571 or yaw_angle < -1.571:
                    # Rotate by 180 to normalise to the sucker workspace.
                    q2 = tft.quaternion_from_euler(0, 0, 3.1415)
                    q = tft.quaternion_multiply(q, q2)

                e = list(tft.euler_from_quaternion(q))
                if e[2] > 1.5: # Stop gimbal lock and weird joint configurations.
                    e[2] = 1.5
                if e[2] < -1.5:
                    e[2] = -1.5
                q = tft.quaternion_from_euler(e[0], e[1], e[2])

                object_global.orientation.x = q[0]
                object_global.orientation.y = q[1]
                object_global.orientation.z = q[2]
                object_global.orientation.w = q[3]

                R = tft.quaternion_matrix(q)

                t.publish_tf_quaterion_as_transform(translation, q, 'global_xyz_link', 'MOVE_HERE', seconds=0.5)

                # Create a pre-grasp pose away from the object.
                T = tft.translation_matrix(translation)
                T2 = tft.translation_matrix((0, 0, -0.05))
                F = tft.concatenate_matrices(T, R, T2)
                pre_grasp_t = tft.translation_from_matrix(F)
                pre_grasp_q = tft.quaternion_from_matrix(F)

                #t.publish_tf_quaterion_as_transform(pre_grasp_t, pre_grasp_q, 'global_xyz_link', 'PRE', seconds=5.0)

                grasp_orientation = Quaternion()
                grasp_orientation.x = q[0]
                grasp_orientation.y = q[1]
                grasp_orientation.z = q[2]
                grasp_orientation.w = q[3]

                rospy.loginfo('MOVE TO TOOL CHANGE HEIGHT')

                if first_attamept:
                    # Move the whole wrist and then select the sucker.
                    res = m.move_to_global(min(object_global.position.x, REALSENSE_MAX_GLOBAL_X), max(min(object_global.position.y, SUCKER_MAX_Y_STRAIGHT) - 0.02, SUCKER_MIN_GLOBAL_Y + 0.02) - SUCKER_REALSENSE_Y_OFFSET, TOOL_CHANGE_REALSENSE_GLOBAL_Z, 'realsense')
                    if not res:
                        return 'failed'
                    rospy.loginfo('SELECTING SUCKER')
                    res = m.move_to_named_pose('wrist_only', 'sucker')
                    if not res:
                        return 'failed'
                    first_attamept = False
                else:
                    # Straighten up the sucker from the last attempt and move into position
                    straight_q = Quaternion()
                    straight_q.x = 0
                    straight_q.y = 0
                    straight_q.z = 0
                    straight_q.w = 1
                    res = m.move_to_global(min(object_global.position.x, SUCKER_MAX_GLOBAL_X), max(min(object_global.position.y, SUCKER_MAX_Y_STRAIGHT) - 0.02, SUCKER_MIN_GLOBAL_Y + 0.02), TOOL_CHANGE_REALSENSE_GLOBAL_Z + 0.35, 'sucker', orientation=straight_q)
                    if not res:
                        return 'failed'

                rospy.loginfo("MOVE TO PRE-GRASP")
                res = m.move_to_global(pre_grasp_t[0], pre_grasp_t[1], min(pre_grasp_t[2], SUCKER_MAX_GLOBAL_Z), 'sucker', orientation=grasp_orientation, velo_scale=0.4)
                if not res:
                    return 'failed'

                rospy.loginfo("PUMP ON, MOVING TO OBJECT")
                suction.pump_on()
                (res, stopped, bail) = m.move_to_global_monitor_weight_and_suction(object_global.position.x, object_global.position.y, min(object_global.position.z, SUCKER_MAX_GLOBAL_Z), 'sucker', scales_topic, orientation=grasp_orientation, velo_scale=0.3)
                if not res:
                    return 'failed'

            else:
                # Straight Down Suck
                rospy.logerr('STRAIGHT PICK')

                # Make sure we're inside the tote.
                edge_offset = 0.0
                object_global.position.x = max(object_global.position.x, position_limits['x_min'] - edge_offset)
                object_global.position.x = min(object_global.position.x, position_limits['x_max'] + edge_offset)
                object_global.position.y = max(object_global.position.y, position_limits['y_min'] - edge_offset)
                object_global.position.y = min(object_global.position.y, position_limits['y_max'] + edge_offset)
                object_global.position.z = min(object_global.position.z, position_limits['z_max'])

                t.publish_pose_as_transform(object_global, 'global_xyz_link', 'MOVE_HERE', seconds=0.5)
                rospy.loginfo('MOVE TO TOOL CHANGE HEIGHT')

                # We only need to select the sucker on the first attempt.
                # Can cause it to break becasue the controllers don't switch properly
                #  if there's no actual movment and we change controllers too quickly. Not sure why, something to do with timing.
                if first_attamept:
                    res = m.move_to_global(object_global.position.x, max(min(object_global.position.y, SUCKER_MAX_Y_STRAIGHT), SUCKER_MIN_Y_STRAIGHT) - SUCKER_REALSENSE_Y_OFFSET, TOOL_CHANGE_REALSENSE_GLOBAL_Z, 'realsense')
                    if not res:
                        return 'failed'
                    rospy.loginfo('SELECTING SUCKER')
                    res = m.move_to_named_pose('wrist_only', 'sucker')
                    if not res:
                        return 'failed'
                    first_attamept = False
                    if item_meta[userdata.data['item_to_pick']['label']]['grasp_type'] == 'grip_suck':
                        # Weird async movement bug.
                        rospy.sleep(1.0)
                else:
                    straight_q = Quaternion()
                    straight_q.x = 0
                    straight_q.y = 0
                    straight_q.z = 0
                    straight_q.w = 1
                    res = m.move_to_global(object_global.position.x, max(min(object_global.position.y, SUCKER_MAX_Y_STRAIGHT), SUCKER_MIN_Y_STRAIGHT), TOOL_CHANGE_REALSENSE_GLOBAL_Z + 0.35, 'sucker', orientation=straight_q)
                    if not res:
                        return 'failed'

                # Correct the rotation if required.
                pca_yaw = userdata.data['item_to_pick']['pca_yaw']
                q = (pca_yaw.x, pca_yaw.y, pca_yaw.z, pca_yaw.w)
                e = list(tft.euler_from_quaternion(q))
                if e[2] > 1.57:
                    e[2] -= 3.14
                if e[2] < -1.57:
                    e[2] += 3.14
                if e[2] > 1.5: # Stop gimbal lock and weird joint configurations.
                    e[2] = 1.5
                if e[2] < -1.5:
                    e[2] = -1.5
                q = tft.quaternion_from_euler(0, 0, e[2])
                pca_yaw.x = q[0]
                pca_yaw.y = q[1]
                pca_yaw.z = q[2]
                pca_yaw.w = q[3]

                rospy.loginfo("PUMP ON, MOVING TO OBJECT")
                suction.pump_on()
                v_scale = 0.25
                if item_meta[userdata.data['item_to_pick']['label']]['grasp_point_type'] == 'rgb_centroid':
                    v_scale = 0.1
                (res, stopped, bail) = m.move_to_global_monitor_weight_and_suction(object_global.position.x, min(object_global.position.y, SUCKER_MAX_Y_STRAIGHT), min(object_global.position.z, SUCKER_MAX_GLOBAL_Z), 'sucker', scales_topic, orientation=pca_yaw, velo_scale=v_scale)
                if not res:
                    return 'failed'

            if bail:
                m.move_relative(0, 0, -0.02, 'sucker_endpoint', 'sucker')
                rospy.sleep(0.5)
            elif not suction.check_pressure_sensor():
                # If it's not already on, give it a sec.
                rospy.sleep(1.0)

            suction_state = suction.check_pressure_sensor() #query pressure sensor for state, has the object been sucked correctly
            rospy.loginfo("PRESSURE SENSOR STATUS: %s" % suction_state)

            if not suction_state and not bail:
                set_suck_level(3)
                # Just move down a touch and double check
                move_amount = 0.02
                if USE_ANGLED:
                    move_amount = 0.035
                if object_global.position.z < (SUCKER_MAX_GLOBAL_Z):
                    move_amount = min(move_amount, SUCKER_MAX_GLOBAL_Z - object_global.position.z)
                    res = m.move_relative(0, 0, move_amount, 'sucker_endpoint', 'sucker', plan_time=0.5)
                else:
                    res = True
                if res:
                    rospy.sleep(1.0)
                    suction_state = suction.check_pressure_sensor()

            if suction_state:
                #userdata.data['last_failed'] = None
                return 'succeeded'
            else:
                suction.pump_off()

        # Run out of grasp poses.
        suction.pump_off()
        if item_meta[userdata.data['item_to_pick']['label']]['grasp_type'] == 'suck_grip':
            return 'try_gripper'
        userdata.data['last_failed'][userdata.data['item_to_pick']['label']] = 0
        return 'suck_failed'
