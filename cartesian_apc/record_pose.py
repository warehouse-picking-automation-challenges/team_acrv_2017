#!/usr/bin/env python
import rospy
import baxter_interface
from std_msgs.msg import *
from sensor_msgs.msg import JointState
import sys

class Recorder(object):
    """docstring for """
    def __init__(self,which_arm="left_",position_name="bin_X"):
        self.pos_subscriber = None
        self.finished = True
        self.arm = which_arm
        self.position = position_name

    # read once from the topic then unregister and wait
    def callback(self, pos_received):
        if self.finished: return

        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", pos_received)
        #do processing here ...
        self.pos_subscriber.unregister()
        self.pos_subscriber = None

        pos_name = self.position
        prefix = self.arm
        print '\n<group_state name="%s" group="%sarm">' % (pos_name, prefix)
        for i in range(len(pos_received.name)):
            if pos_received.name[i].startswith(prefix):
                print '\t<joint name="%s" value="%s" />' % (pos_received.name[i], pos_received.position[i])

        print '</group_state>\n\n'

        self.finished = True

    def recordPose(self):
        if self.pos_subscriber == None:
            self.finished = False
            self.pos_subscriber = rospy.Subscriber("/robot/joint_states",
                                                   JointState, self.callback)


def main(which_arm, position_name):
    rospy.init_node('pose_listener', anonymous=True)

    finished = False
    rec = Recorder(which_arm,position_name)
    while(not finished):
        # wait for keyboard input
        text_in = raw_input("Press Enter to record a pose... (Q for quit)")
        # Python 3: input("Press Enter to record a pose...")
        if text_in == "Q" or text_in == "q": finished = True
        else:
            rec.recordPose()
            rospy.sleep(1)


if __name__ == '__main__':
    if len(sys.argv) == 1:
        print "Usage:\n\tpython record_pose.py <arm> <position_name>"
    else:
        main(sys.argv[1] + "_", sys.argv[2])
    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()
