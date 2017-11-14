#!/usr/bin/env python
# -*- coding: utf-8

import rospy
import sys
from acrv_realsense_ros.srv import *
import sensor_msgs
import std_msgs


if __name__=="__main__":
    rospy.init_node('whitebalance_test_node')
    #check = input('Waiting for keypress')
    get_grey_scalars = rospy.ServiceProxy('get_grey_scalars', get_scalars)
    img = rospy.wait_for_message("/realsense/rgb/image_raw", sensor_msgs.msg.Image)

    response1 = get_grey_scalars.call(img,0,0,640,360)
    ra = response1.ra
    ba = response1.ba
    print('Grey scalars obtained, ra: ', ra, 'ba: ', ba)
