import rospy
import smach

# DO NOTHING
class StopAbort(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['loop'])

    def execute(self, userdata):
        rospy.sleep(5.0)
        return 'loop'
