#!/usr/bin/env python

import rospy
from dynamixel_msgs.msg import JointState
from dynamixel_controllers.srv import SetTorqueLimit

zero_count = {}

def status_callback(msg):
    global zero_count

    dx_name = msg.name
    if dx_name not in zero_count:
        zero_count[dx_name] = 0

    # No easy way to tell if the servo is overloaded.
    # The velocity and load will both be zero for an extended period though.
    # When not overloaded there are normally small fluctuations.
    if msg.velocity == 0 and msg.load == 0 and abs(msg.error) > 0.025:
        zero_count[dx_name] += 1
    else:
        zero_count[dx_name] = 0

    # Messages are published at about 30 hz
    # Anything shorter ends up with spurious values.
    if zero_count[dx_name] > 10:
        # possible verload detected.
        rospy.logerr('Possible overload detected on %s. Resetting it.' % dx_name)
        ser = rospy.ServiceProxy('/%s_controller/set_torque_limit' % dx_name, SetTorqueLimit)
        torque = rospy.get_param('/%s_controller/joint_torque_limit' % dx_name, 1.0)
        ser.call(torque)
        rospy.sleep(1)
        rospy.loginfo('Reset Done')

if __name__ == '__main__':
    rospy.init_node('dynamixel_resetter')
    gripper_motitor = rospy.Subscriber('/gripper_controller/state', JointState, status_callback, queue_size=1)
    elephant_motitor = rospy.Subscriber('/elephant_controller/state', JointState, status_callback, queue_size=1)
    release_valve_motitor = rospy.Subscriber('/release_valve_controller/state', JointState, status_callback, queue_size=1)
    rospy.loginfo('Dynamixel Resetter successfully started.')
    rospy.spin()
