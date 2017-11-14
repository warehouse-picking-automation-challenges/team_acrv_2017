#!/usr/bin/env python

import rospy
import geometry_msgs.msg as gmsg
import tf
from helpers import movement as m
from helpers import transforms as t

'''

SIMULATION TOOL CHANGE ANGLE TEST SCRIPT

'''

if __name__ == '__main__':

    move_error = 'Failed to complete a move'
    move_success = 'Move State: '
    tool = 'gripper'

    if tool is 'gripper':

        #TOOL CHANGE TO 45 DEGREES

        rospy.init_node('my_ros_node')
        curr = t.current_robot_pose('global_xyz_link', 'gripper_endpoint')
        pose = gmsg.Pose()
        qt = tf.transformations.quaternion_from_euler(0.785398, 0.0, 1.57)
        pose.orientation.x = qt[0]
        pose.orientation.y = qt[1]
        pose.orientation.z = qt[2]
        pose.orientation.w = qt[3]
        rospy.loginfo('MOVE UP TO 45 DEGREES POSITION')
        res = m.move_to_global(curr.position.x, curr.position.y, 0.6, 'gripper', orientation=pose.orientation)
        if not res:
            rospy.logerr(move_error)
        rospy.loginfo(move_success + str(res))

        #MOVE TO DROP POSITION

        curr = t.current_robot_pose('global_xyz_link', 'gripper_endpoint')
        rospy.loginfo('MOVE UP TO DROP POSITION')
        res = m.move_to_global(0.3, curr.position.y, curr.position.z, 'gripper', orientation=None)
        if not res:
            rospy.logerr(move_error)
        rospy.loginfo(move_success + str(res))

        #MOVE TO STRAIGHT DROP POSITION

        curr = t.current_robot_pose('global_xyz_link', 'gripper_endpoint')
        pose = gmsg.Pose()
        qt = tf.transformations.quaternion_from_euler(0.0, 0.0, 1.57)
        pose.position.x = 0.05
        pose.position.y = curr.position.y
        pose.position.z = 1.0
        pose.orientation.x = qt[0]
        pose.orientation.y = qt[1]
        pose.orientation.z = qt[2]
        pose.orientation.w = qt[3]
        t.publish_pose_as_transform(pose,'global_xyz_link', 'PINEAPPLE', seconds=5)
        rospy.loginfo('MOVE UP TO STRAIGHT DROP POSITION')
        res = m.move_to_global(pose.position.x, curr.position.y, pose.position.z, 'gripper', orientation=pose.orientation)
        if not res:
            rospy.logerr(move_error)
        rospy.loginfo(move_success + str(res))

    elif tool is 'sucker':

        #TOOL CHANGE TO 45 DEGREES

        rospy.init_node('my_ros_node')
        curr = t.current_robot_pose('global_xyz_link', 'sucker_endpoint')
        pose = gmsg.Pose()
        qt = tf.transformations.quaternion_from_euler(0.0, -0.785398, 0.0)
        pose.orientation.x = qt[0]
        pose.orientation.y = qt[1]
        pose.orientation.z = qt[2]
        pose.orientation.w = qt[3]
        rospy.loginfo('MOVE UP TO 45 DEGREES POSITION')
        res = m.move_to_global((curr.position.x-0.4), curr.position.y, 0.6, 'sucker', orientation=pose.orientation)
        if not res:
            rospy.logerr(move_error)
        rospy.loginfo(move_success + str(res))

        #MOVE TO DROP POSITION

        curr = t.current_robot_pose('global_xyz_link', 'sucker_endpoint')
        rospy.loginfo('MOVE UP TO DROP POSITION')
        res = m.move_to_global(-0.2, curr.position.y, curr.position.z, 'sucker', orientation=None)
        if not res:
            rospy.logerr(move_error)
        rospy.loginfo(move_success + str(res))

        #MOVE TO STRAIGHT DROP POSITION

        curr = t.current_robot_pose('global_xyz_link', 'sucker_endpoint')
        pose = gmsg.Pose()
        qt = tf.transformations.quaternion_from_euler(0.0, -0.01, 0.0)
        pose.position.x = 0.05
        pose.position.y = curr.position.y
        pose.position.z = 1.0
        pose.orientation.x = qt[0]
        pose.orientation.y = qt[1]
        pose.orientation.z = qt[2]
        pose.orientation.w = qt[3]
        rospy.loginfo('MOVE UP TO STRAIGHT DROP POSITION')
        t.publish_pose_as_transform(pose,'global_xyz_link', 'PINEAPPLE', seconds=5)
        res = m.move_to_global(pose.position.x, curr.position.y, pose.position.y, 'sucker', orientation=pose.orientation)
        if not res:
            rospy.logerr(move_error)
        rospy.loginfo(move_success + str(res))

    else:
        rospy.loginfo('INVALID TOOL')
