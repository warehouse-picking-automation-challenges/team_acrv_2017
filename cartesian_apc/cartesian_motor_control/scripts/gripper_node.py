#!/usr/bin/env python

import math
import rospy
from std_msgs.msg import Float64
from dynamixel_msgs.msg import JointState
from cartesian_motor_control.srv import *

OPEN_POSITION = 150 * math.pi / 180
CLOSED_POSITION = -50 * math.pi / 180

class GripperController():
	def __init__(self):
		self.rate = rospy.Rate(100)
		self.pos = OPEN_POSITION
		self.command = 'open'
		self.gain = 0.1
		self.load = 0
		rospy.Service('/gripper/control_gripper', ChangeGrip, self.ControlGripperHandler)
		rospy.Subscriber('gripper_controller/state', JointState, self.state_callback)
		self.pub = rospy.Publisher('gripper_controller/command', Float64, queue_size='10')
		rospy.loginfo('Connecting to gripper and starting service....')

	def state_callback(self, data):
		self.load = data.load
		if self.command == 'hold' and data.is_moving == False:
			if self.load > 0.3:
				# check yo-self before you wreck yo-self
				self.pub.publish(data.current_pos)
			elif self.load < 0.15:
				self.pub.publish(Float64(CLOSED_POSITION))

	def ControlGripperHandler(self, req):
		res = ChangeGripResponse()
		res.success = True

		if req.state == 0: # Closed
			self.command = 'close'
			rospy.loginfo('Closing gripper')
			f = Float64(CLOSED_POSITION)
			self.pub.publish(f)
		elif req.state == 1: # Open
			self.command = 'open'
			rospy.loginfo('Opening gripper')
			f = Float64(OPEN_POSITION)
			self.pub.publish(f)
		elif req.state == 2: # Hold position
			self.command = 'hold'
		else:
			res.success = False
			rospy.logerr('State should be 0 or 1')

		return res

	def execute(self):
		while not rospy.is_shutdown():
			self.rate.sleep()

if __name__ == '__main__':
	rospy.init_node('gripper_control_node')

	try:
		myGripperController = GripperController()
		myGripperController.execute()
	except rospy.ROSInterruptException:
		pass
