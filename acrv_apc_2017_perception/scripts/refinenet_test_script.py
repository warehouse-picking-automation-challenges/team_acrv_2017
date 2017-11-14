#!/usr/bin/env python
import rospy
import time

from std_msgs.msg import String

import cv2
from cv_bridge import CvBridge, CvBridgeError

from acrv_apc_2017_perception.srv import rgbd_object_proposal, rgbd_object_proposalRequest

import os

if __name__ == "__main__":
    rospy.init_node('refinenet_test_script')
    cv_bridge = CvBridge()
    image_rect = cv2.imread('/home/apc/cloudstor/acrv_apc_2017_data/data_regenerated/tote/tote_mixed_20170509/img_00107/color_rect.png')
    # image_rect = cv2.imread(os.path.abspath(os.path.join(os.path.dirname(os.path.dirname(os.path.realpath(__file__))), 'src/refinenet/VOC_DATA/img_00047/color_rect.png')))
    image_rect_hd = cv2.imread('/home/apc/cloudstor/acrv_apc_2017_data/data_regenerated/tote/tote_mixed_20170509/img_00107/color_hd_rect.png')
    # image_rect_hd = cv2.imread(os.path.abspath(os.path.join(os.path.dirname(os.path.dirname(os.path.realpath(__file__))), 'src/refinenet/VOC_DATA/img_00047/color_hd_rect.png')))
    service = rospy.ServiceProxy('refinenet_classification', rgbd_object_proposal)

    labels = ['windex', 'mesh_cup', 'shower_curtain', 'scotch_sponges', 'harry_potter_dvd', 'band_aid_tape', 'black_fashion_gloves', 'utility_brush']
    # labels = ['hanes_socks', 'utility_brush']
    # labels = ['pie_plates', 'composition_book', 'epsom_salts']
    # labels = [
    #     'flashlight',
    #     'balloons',
    #     'dove_soap',
    #     'laugh_out_loud_jokes',
    #     'expo_eraser',
    #     'speed_stick',
    #     'duct_tape',
    #     'oral_b_toothbrush',
    #     'fiskars_scissors_red',
    #     'poland_spring_water',
    #     'irish_spring_soap',
    #     'pets_bowl',
    #     'cool_shot_glue_sticks',
    #     'band_aid_tape',
    #     'spoon_and_fork',
    #     'crayons',
    # ]
    expected_labels = []
    for label in labels:
        expected_labels.append(String(label))

    im_msg = cv_bridge.cv2_to_imgmsg(image_rect)
    im_msg_hd = cv_bridge.cv2_to_imgmsg(image_rect_hd)

    req = rgbd_object_proposalRequest()
    req.color = im_msg
    req.color_hd = im_msg_hd
    req.expected_labels = expected_labels

    start = time.time()
    res = service.call(req)
    end = time.time()
    print('Took ' + str(end - start))

    for i in res.label:
        print(i)
    image_res = cv_bridge.imgmsg_to_cv2(res.classification)
    overlay_img = cv_bridge.imgmsg_to_cv2(res.overlay_img)
    cv2.imshow('result', image_res)
    cv2.imshow('overlay', overlay_img)
    cv2.waitKey(0)
