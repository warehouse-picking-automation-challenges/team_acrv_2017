#!/usr/bin/env python

import rospy

import os
import re

import cv2

from std_msgs.msg import String
from acrv_apc_2017_perception.msg import autosegmenter_msg
import cv_bridge

if __name__ == "__main__":
    p = os.path.expanduser('~/cloudstor/acrv_apc_2017_data/results_refinenet/20170701_tote_bin/125')
    files = [f for f in os.listdir(p) if os.path.isfile(os.path.join(p, f)) and re.match('[0-9]{6}_[0-9]{6}_rgb.png$', f)]

    rospy.init_node('autosegmenter_test_script')
    rate = rospy.Rate(10)
    pub = rospy.Publisher(rospy.get_param('/autosegmenter_image_topic', '/realsense_wrist/rgb/image_rect'), autosegmenter_msg, queue_size=5)
    while not pub.get_num_connections():
        rate.sleep()

    bridge = cv_bridge.CvBridge()
    for f in files:
        print('Using image %s' % f)
        im = cv2.imread(os.path.join(p, f))
        im = cv2.cvtColor(im, cv2.COLOR_BGR2RGB)
        fname = f.replace('_rgb.png', '_map.txt')
        with open(os.path.join(p, fname), 'r') as fi:
            content = fi.readlines()
        content = [x.strip() for x in content]

        msg = autosegmenter_msg()

        msg.image = bridge.cv2_to_imgmsg(im, encoding='rgb8')

        msg.image_name.data = f

        for c in content:
            msg.content.append(String(c))

        print(msg.image_name.data)
        print(msg.content)
        pub.publish(msg)
        print('Published')
        if rospy.is_shutdown():
            break
        break
