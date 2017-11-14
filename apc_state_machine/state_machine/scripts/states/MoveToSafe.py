import rospy
import smach
import smach_ros

from helpers import movement as m
from helpers import transforms as t
from helpers import suction

class MoveToSafe(smach.State):
    def __init__(self, duration=1.0):
        smach.State.__init__(self, outcomes=['succeeded','failed'])

    # ==========================================================
    def execute(self, userdata):
        suction.pump_off()

        curr = t.current_robot_pose('global_xyz_link', 'realsense_endpoint')

        rospy.loginfo('MOVE UP TO SAFE POSITION')
        res = m.move_to_global(curr.position.x, curr.position.y, 0.505, 'realsense')
        if not res:
            rospy.logerr('Failed to move to safe position.')
            return 'failed'

        rospy.sleep(10.0)

        return 'succeeded'
