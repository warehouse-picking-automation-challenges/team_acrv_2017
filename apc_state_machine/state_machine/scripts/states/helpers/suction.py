import rospy
import time
import sys
import smach
from ros_arduino_msgs.srv import *
from std_msgs.msg import Float64

digitalwrite_service_name = '/arduino/digital_write'
rospy.loginfo('Waiting for %s service ...' % digitalwrite_service_name)
try:
	rospy.wait_for_service(digitalwrite_service_name, timeout=1)
	arduino_digitalwrite_service = rospy.ServiceProxy(digitalwrite_service_name, DigitalWrite)
except:
	rospy.logerr('Service %s not available. Restart and try again' % digitalwrite_service_name)
	arduino_digitalwrite_service = None

digitalread_service_name = '/arduino/digital_read'
rospy.loginfo('Waiting for %s service ...' % digitalread_service_name)
try:
	rospy.wait_for_service(digitalread_service_name, timeout=1)
	arduino_digitalread_service = rospy.ServiceProxy(digitalread_service_name, DigitalRead)
except:
	rospy.logerr('Service %s not available. Restart and try again' % digitalread_service_name)
	arduino_digitalread_service = None


release_valve_publisher = rospy.Publisher('/release_valve_controller/command', Float64, queue_size=1)


def pump_on():
	global arduino_digitalwrite_service
	if arduino_digitalwrite_service is None:
		return False
	req = DigitalWriteRequest()
	req.pin = 2
	req.value = 1
	resp = arduino_digitalwrite_service(req)
	return True


def pump_off():
	global arduino_digitalwrite_service
	if arduino_digitalwrite_service is None:
		return False
	req = DigitalWriteRequest()
	req.pin = 2
	req.value = 0
	resp = arduino_digitalwrite_service(req)
	return True


def check_pressure_sensor():
	global arduino_digitalread_service
	if arduino_digitalread_service is None:
		return False

	current_state = False
	req = DigitalReadRequest()
	req.pin = 3
	current_state = arduino_digitalread_service(req)

	return current_state.value


def set_suck_level(lvl):
	global release_valve_publisher
	msg = Float64()
	if lvl == 0:
		msg.data = -1.0
	elif lvl == 1:
		msg.data = -0.3
	elif lvl == 2:
		msg.data = -0.2
	elif lvl == 3:
		msg.data = -0.02

	release_valve_publisher.publish(msg)

	return True


if __name__ == '__main__':

	#simple pump test which turns pump on and checks the pressure sensor at a rate of 100hz
	rospy.init_node('simple_pump_controller')
	pump_on()
