#!/usr/bin/env python
# -*- coding: utf-8
from __future__ import print_function
import rospy
import numpy as np
import Image
import sys
from std_msgs.msg import String
import sensor_msgs
from cv_bridge import CvBridge, CvBridgeError

pub_crop = rospy.Publisher('/realsense/rgb_white_balanced/grey_crop', sensor_msgs.msg.Image, queue_size=10)
pub_white_balanced = rospy.Publisher('/realsense/rgb_white_balanced/image_raw', sensor_msgs.msg.Image, queue_size=10)
bridge = CvBridge()

def realsense_rgb_callback(img_msg):
    print('got msg')
    try:
        img = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    except CvBridgeError as e:
        print(e)

    # img = to_pil(cv_image)

    img1 = img[66:185,497:580]

    # img1.show()
    # img.show()
    # to_pil(grey_world(from_pil(img)))[0].show()
    (ra,ba) = grey_world(img1)
    # print(ra,ba)
    white_balanced_img = grey_world1(img,ra,ba)

    # TODO publish white balanced image

    white_balanced_crop_img_msg = bridge.cv2_to_imgmsg(img1, encoding="passthrough")
    pub_crop.publish(white_balanced_crop_img_msg)

    white_balanced_img_msg = bridge.cv2_to_imgmsg(white_balanced_img, encoding="passthrough")
    pub_white_balanced.publish(white_balanced_img_msg)

# def from_pil(pimg):
#     pimg = pimg.convert(mode='RGB')
#     nimg = np.asarray(pimg)
#     nimg.flags.writeable = True
#     return nimg
#
# def to_pil(nimg):
#     return (Image.fromarray(np.uint8(nimg[0])),nimg[1],nimg[2])

def grey_world(nimg):
    nimg = nimg.transpose(2, 0, 1).astype(np.uint32)
    mu_g = np.average(nimg[1])
    # nimg[0] = np.minimum(nimg[0]*(mu_g/np.average(nimg[0])),255)
    # nimg[2] = np.minimum(nimg[2]*(mu_g/np.average(nimg[2])),255)
    ra = mu_g/np.average(nimg[0])
    ba = mu_g/np.average(nimg[2])
    # return  ((nimg.transpose(1, 2, 0).astype(np.uint8)),ra,ba)
    return  (ra,ba)

def grey_world1(nimg,ra,ba):
    nimg = nimg.transpose(2, 0, 1).astype(np.uint32)
    nimg[0] = np.minimum(nimg[0]*ra,255)
    nimg[2] = np.minimum(nimg[2]*ba,255)
    return  nimg.transpose(1, 2, 0).astype(np.uint8)

if __name__=="__main__":
    rospy.init_node('white_balance')
    rospy.Subscriber("/realsense/rgb/image_raw", sensor_msgs.msg.Image, realsense_rgb_callback)
    rospy.spin()
