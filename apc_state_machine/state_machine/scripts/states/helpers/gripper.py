#!/usr/bin/env python

import rospy
from cartesian_motor_control.srv import *

gripper_service_name = '/gripper/control_gripper'
rospy.loginfo('Waiting for %s service ...' % gripper_service_name)
try:
	rospy.wait_for_service(gripper_service_name, timeout=1)
	gripper_control_service = rospy.ServiceProxy(gripper_service_name, ChangeGrip)
except:
	rospy.logerr('Service %s not available. Restart and try again' % gripper_service_name)
	gripper_control_service = None


def close_gripper():
	"""
	Close the gripper
	"""
	global gripper_control_service
	if gripper_control_service is None:
		return False

	try:
		rospy.wait_for_service(gripper_service_name, timeout=1)
	except:
		rospy.logerr('Service %s not available. Restart and try again' % gripper_service_name)
		return False

	gripper_control_service = rospy.ServiceProxy(gripper_service_name, ChangeGrip)
	req = ChangeGripRequest()
	req.state = 0
	resp = gripper_control_service(req)
	return resp.success


def open_gripper():
	"""
	Open the gripper
	"""
	global gripper_control_service
	if gripper_control_service is None:
		return False

	req = ChangeGripRequest()
	req.state = 1
	resp = gripper_control_service(req)
	return resp.success

	try:
		rospy.wait_for_service(gripper_service_name, timeout=1)
	except:
		rospy.logerr('Service %s not available. Restart and try again' % gripper_service_name)
		return False

def hold_gripper():
	""" Hold the gripper steady """
	global gripper_control_service
	if gripper_control_service is None:
		return False

	req = ChangeGripRequest()
	req.state = 2
	resp = gripper_control_service(req)
	return resp.success


if __name__ == '__main__':

	#open_gripper()
	close_gripper()
