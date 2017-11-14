#!/usr/bin/env python
# -*- coding: utf-8
from __future__ import print_function
import rospy
import numpy as np
import Image
import sys
from std_msgs.msg import String
from acrv_realsense_ros.srv import *
import sensor_msgs
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()
ra = 1.0
ba = 1.0
scalars_obtained = False
response1 = get_scalarsResponse()

# Node will output images continously with default (no) scaling until service run, then will output proper scalaing
def realsense_rgb_callback(img_msg):
    # Make values persistent to avoid calling service more than once
    global response1
    global scalars_obtained
    global ra
    global ba

    # Once service to get scalar values is run, do not run again
    if not scalars_obtained:
        get_grey_scalars = rospy.ServiceProxy('get_grey_scalars', get_scalars)
        response1 = get_grey_scalars(img_msg)       #TODO: - minor efficiency increase here by already cropping image
        scalars_obtained = response1.service_run
        ra = response1.ra
        ba = response1.ba
        print('Grey scalars obtained, ra: ', ra, 'ba: ', ba)
        #TODO: do I need to terminate service node after I run this?

    try:
        img = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    except CvBridgeError as e:
        print(e)

    img1 = img[66:185,497:580]

    white_balanced_img = grey_world1(img,ra,ba)

    white_balanced_crop_img_msg = bridge.cv2_to_imgmsg(img1, encoding="passthrough")
    pub_crop.publish(white_balanced_crop_img_msg)

    white_balanced_img_msg = bridge.cv2_to_imgmsg(white_balanced_img, encoding="passthrough")
    pub_white_balanced.publish(white_balanced_img_msg)


def grey_world1(nimg,ra,ba):
    nimg = nimg.transpose(2, 0, 1).astype(np.uint32)
    nimg[0] = np.minimum(nimg[0]*ra,255)
    nimg[2] = np.minimum(nimg[2]*ba,255)
    return  nimg.transpose(1, 2, 0).astype(np.uint8)


if __name__=="__main__":
    rospy.init_node('get_grey_scalars_client')
    pub_crop = rospy.Publisher('/realsense/rgb_white_balanced/grey_crop', sensor_msgs.msg.Image, queue_size=10)
    pub_white_balanced = rospy.Publisher('/realsense/rgb_white_balanced/image_raw', sensor_msgs.msg.Image, queue_size=10)
    rospy.Subscriber("/realsense/rgb/image_raw", sensor_msgs.msg.Image, realsense_rgb_callback)
    rospy.spin()
