import rospy
import smach
import smach_ros

from helpers import movement as m

class MoveToHome(smach.State):
    def __init__(self, duration=1.0):
        smach.State.__init__(self, outcomes=['succeeded','failed'])

    # ==========================================================
    def execute(self, userdata):
        # wait for the specified duration or until we are asked to preempt
        print("HOME")
        res = m.move_to_named_pose('whole_arm', 'home')

        if not res:
            return 'failed'

        rospy.sleep(2)

        return 'succeeded'
