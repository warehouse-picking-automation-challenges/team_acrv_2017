#!/usr/bin/env python

import rospy
from cartesian_motor_control.srv import *


def close_gripper():

	gripper_service_name = '/gripper/control_gripper'

	rospy.loginfo('Waiting for %s service ...' % gripper_service_name)

	try:
		rospy.wait_for_service(gripper_service_name, timeout=1)
	except:
		rospy.logerr('Service %s not available. Restart and try again' % gripper_service_name)

	gripper_control_service = rospy.ServiceProxy(gripper_service_name, ChangeGrip)
	req = ChangeGripRequest()
	req.state = 0 #open gripper
	req.desired_load = 0.0
	resp = gripper_control_service(req)
	return resp


def open_gripper():

	gripper_service_name = '/gripper/control_gripper'

	rospy.loginfo('Waiting for %s service ...' % gripper_service_name)

	try:
		rospy.wait_for_service(gripper_service_name, timeout=1)
	except:
		rospy.logerr('Service %s not available. Restart and try again' % gripper_service_name)

	gripper_control_service = rospy.ServiceProxy(gripper_service_name, ChangeGrip)
	req = ChangeGripRequest()
	req.state = 1 #close gripper
	req.desired_load = 0.0
	resp = gripper_control_service(req)
	return resp


if __name__ == '__main__':

    k = input("Open or Close gripper (1,0)?: ")
    if k == 1:
        open_gripper()
        print 'Opening Gripper...'
    elif k == 0:
        close_gripper()
        print 'Closing Gripper...'
    else:
        print 'Invalid Gripper state!'
