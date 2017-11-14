#!/usr/bin/env python
# -*- coding: utf-8
import rospy
import numpy as np
import Image
import sys
from std_msgs.msg import String
import sensor_msgs
from cv_bridge import CvBridge, CvBridgeError
from acrv_realsense_ros.srv import *

bridge = CvBridge()

def handle_get_grey_scalars(req):
    try:
        img = bridge.imgmsg_to_cv2(req.img_msg, "bgr8")
    except CvBridgeError as e:
        print(e)

    img1 = img[66:185,497:580]
    img1 = img1.transpose(2, 0, 1).astype(np.uint32)
    mu_g = np.average(img1[1])
    res = get_scalarsResponse()
    res.ra = mu_g/np.average(img1[0])
    res.ba = mu_g/np.average(img1[2])
    res.service_run = True
    return (res)


def get_grey_scalars_server():
    rospy.init_node('get_grey_scalars_server')
    s = rospy.Service('get_grey_scalars', get_scalars, handle_get_grey_scalars)
    rospy.spin()


if __name__ == "__main__":
    get_grey_scalars_server()
