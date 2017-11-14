#!/usr/bin/env python

import sys
import os
import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from acrv_apc_2017_perception.srv import rgbd_object_proposal,rgbd_object_proposalRequest, rgbd_object_proposalResponse
import std_msgs.msg

nodeName = 'image_loading_node'
serviceName = 'rgbd_object_proposal_service'


def call_perception_service():

    if len(sys.argv) < 2:
        print("no text argument to send")
        exit()

    basePath = sys.argv[1]
    colorImgPath = basePath +"/color.png"
    depthImgPath = basePath +"/depth_aligned.png"
    expectedItemsPath = basePath+"/map.txt"

    if not os.path.isfile(colorImgPath):
        print("Color image does not exist: %s" % colorImgPath)
        exit()

    if not os.path.isfile(depthImgPath):
        print("Depth image does not exist: %s" % depthImgPath)
        exit()

    if not os.path.isfile(expectedItemsPath):
        print("Expected items text file does not exist: %s" % expectedItemsPath)

    #load the images from file
    cv2ColorImg = cv2.imread(colorImgPath,cv2.IMREAD_COLOR)
    cv2DepthImg = cv2.imread(depthImgPath,cv2.IMREAD_GRAYSCALE)

    #resize the images
    cv2ColorHdImg = cv2ColorImg
    cv2ColorSmallImg = cv2.resize(cv2ColorImg[:,240:1680],(640,480),interpolation=cv2.INTER_NEAREST)
    cv2DepthSmallImg = cv2.resize(cv2DepthImg[:,240:1680],(640,480),interpolation=cv2.INTER_NEAREST)

    # cv2.imshow('Big',cv2ColorHdImg)
    # cv2.imshow('Small',cv2ColorSmallImg)
    # cv2.imshow('Small Depth',cv2DepthSmallImg)
    # cv2.waitKey()


    expectedLabels = []
    try:
        with open(expectedItemsPath) as f:
            for line in f.readlines():
                line = line.strip()
                if len(line)>0:
                    expectedLabels.append( std_msgs.msg.String(line) )
    except Exception as e:
        print("Could not read expected items form file: %s" % expectedItemsPath)
        print(e)



    #cv2.imshow('Depth',cv2DepthImg)
    #cv2.waitKey()

    rosColorImg = None
    rosDepthImg = None

    try:
        rosColorHdImg = CvBridge().cv2_to_imgmsg(cv2ColorHdImg,encoding="bgr8")
        rosColorSmallImg = CvBridge().cv2_to_imgmsg(cv2ColorSmallImg,encoding="bgr8")
        rosDepthSmallImg = CvBridge().cv2_to_imgmsg(cv2DepthSmallImg)
    except CvBridgeError as e:
        print(e)
        exit()

    req = rgbd_object_proposalRequest()
    req.color_hd = rosColorHdImg
    req.color = rosColorSmallImg
    req.depth = rosDepthSmallImg
    req.expected_labels = expectedLabels

    rospy.init_node(nodeName,anonymous = True)

    res = None
    try:
        serviceProxy = rospy.ServiceProxy(serviceName,rgbd_object_proposal)
        res = serviceProxy(req)
    except rospy.ServiceException as e:
        print(e)
        exit()

    for label in res.label:
        print(label.data)

    cv2ProposalImg = CvBridge().imgmsg_to_cv2(res.classification)
    cv2InpaintedDepth  = CvBridge().imgmsg_to_cv2(res.inpainted_depth)
    cv2.imshow('Proposals',cv2ProposalImg)
    cv2.imshow('Inpainted Depth',cv2InpaintedDepth)
    cv2.waitKey(1000)
    cv2.destroyAllWindows()


if __name__ == '__main__':
    for i in range(1):
        call_perception_service()
