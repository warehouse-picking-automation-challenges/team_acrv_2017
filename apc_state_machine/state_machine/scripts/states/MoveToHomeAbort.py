import rospy
import smach
import smach_ros

from states.helpers import movement as m

class MoveToHome(smach.State):
    def __init__(self, duration=1.0):
        smach.State.__init__(self, outcomes=['succeeded','failed'])

    # ==========================================================
    def execute(self, userdata):
        # wait for the specified duration or until we are asked to preempt
        print("ABORTING")
        while not rospy.is_shutdown():
            pass
