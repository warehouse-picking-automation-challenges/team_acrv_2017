import rospy
import smach
import smach_ros

from helpers import movement as m
from helpers import transforms as t
from helpers import gripper

class ToolChange(smach.State):
    def __init__(self, tool=None, duration=1.0):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
        self.tool = tool

    # ==========================================================
    def execute(self, userdata):
        gripper.open_gripper
        rospy.loginfo('TOOL CHANGE')

        rospy.loginfo('\tMove up to safe height')
        curr = t.current_robot_pose('global_xyz_link', 'realsense_endpoint')
        res = m.move_to_global(curr.position.x, curr.position.y, 0.505, 'realsense')
        if not res:
            rospy.logerr('Failed to move to safe position.')
            return 'failed'

        rospy.loginfo('\tMove to tool change position')
        res = m.move_to_named_pose('realsense', 'tool_change_position')
        if not res:
            rospy.logerr('Failed to move to tool_change_position')
            return 'failed'

        if self.tool == 'gripper':
            rospy.loginfo('\tSelect Gripper')
            tool_change_named_pose = 'tool_change_gripper'

        elif self.tool == 'sucker':
            rospy.loginfo('\tSelect Sucker')
            tool_change_named_pose = 'tool_change_sucker'

        elif self.tool == 'neutral':
            # Allow us to select neutral
            rospy.loginfo('\tSelect Neutral')
            tool_change_named_pose = 'tool_change_neutral'

        else:
            rospy.logerr('Incorrect tool name for tool change')
            return 'failed'

        res = m.move_to_named_pose('whole_arm', tool_change_named_pose)
        if not res:
            rospy.logerr('Failed to move to %s' % tool_change_named_pose)
            return 'failed'
        return 'succeeded'
