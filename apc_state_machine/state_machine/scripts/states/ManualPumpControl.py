#! /usr/bin/env python

import rospy
import time
import sys
from ros_arduino_msgs.srv import *

def digital_write_client(value):

	digitalwrite_service_name = '/arduino/digital_write'

	rospy.loginfo('Waiting for %s service ...' % digitalwrite_service_name)

	try:
		rospy.wait_for_service(digitalwrite_service_name, timeout=1)
	except:
		rospy.logerr('Service %s not available. Restart and try again' % digitalwrite_service_name)

	arduino_digitalwrite_service = rospy.ServiceProxy(digitalwrite_service_name, DigitalWrite)

	#Start the pump (version 1)
	ss = arduino_digitalwrite_service
	req = DigitalWriteRequest()
	req.pin = 2
	req.value = value
	#ss.wait_for_service()
	resp = ss(req)

def usage():
	return "%s [value]"%sys.argv[0]

if __name__ == "__main__":
	if len(sys.argv) == 2:
		value = int(sys.argv[1])
		if value > 1:
			print usage()
			sys.exit()
	else:
		print usage()
		sys.exit(1)
	print "Changing pump state to: %s"%(value)
	digital_write_client(value)
