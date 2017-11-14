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

class WhiteBalance:
    def __init__(self):
        self.ra = 1.0
        self.ba = 1.0
        self.bridge = CvBridge()
        self.pub_white_balanced = rospy.Publisher('/realsense/rgb_white_balanced/image_raw', sensor_msgs.msg.Image, queue_size=1)
        self.grey_scalars_service = rospy.Service('get_grey_scalars', get_scalars, self.handle_get_grey_scalars)
        rospy.Subscriber("/realsense/rgb/image_raw", sensor_msgs.msg.Image, self.realsense_rgb_callback)

    def realsense_rgb_callback(self, img_msg):
        try:
            img = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        white_balanced_img = self.grey_world1(img,self.ra,self.ba)

        white_balanced_img_msg = self.bridge.cv2_to_imgmsg(white_balanced_img, encoding="passthrough")
        self.pub_white_balanced.publish(white_balanced_img_msg)

    def grey_world1(self, nimg,ra,ba):
        nimg = nimg.transpose(2, 0, 1).astype(np.uint32)
        nimg[0] = np.minimum(nimg[0]*ra ,255)
        nimg[2] = np.minimum(nimg[2]*ba ,255)
        return  nimg.transpose(1, 2, 0).astype(np.uint8)

    def handle_get_grey_scalars(self, req):
        try:
            img = self.bridge.imgmsg_to_cv2(req.img_msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        img1 = img[req.top_left_y:req.bot_right_y,req.top_left_x:req.bot_right_x,:]
        img1 = img1.transpose(2, 0, 1).astype(np.uint32)
        mu_g = np.average(img1[1])
        mu_r = np.average(img1[0])
        mu_b = np.average(img1[2])
        if mu_r > 0 and mu_b > 0:
            self.ra = mu_g/np.average(img1[0])
            self.ba = mu_g/np.average(img1[2])
        else:
            self.ra = 1
            self.ba = 1
        response1 = get_scalarsResponse()
        response1.ra = self.__ra
        response1.ba = self.__ba
        return response1


# Main Function:
if __name__=="__main__":
    rospy.init_node('get_grey_scalars_node')
    white_balance = WhiteBalance()
    rospy.spin()
