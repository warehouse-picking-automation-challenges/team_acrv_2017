#!/usr/bin/env python

import sys
import rospy
import cv2
import numpy as np
import matlab.engine
from cv_bridge import CvBridge, CvBridgeError
from acrv_apc_2017_perception.srv import rgbd_object_proposal, rgbd_object_proposalRequest, rgbd_object_proposalResponse

matlabPath = "./../rgbd_object_proposals"

nodeName = 'rgbd_object_proposal_node'
serviceName = 'rgbd_object_proposal_service'

class MatlabObjectProposalService:
    def __init__(self):
        print("Starting Matlab Engine")
        self.eng = matlab.engine.start_matlab()
        print("Add paths")
        self.eng.addpath(self.eng.genpath(matlabPath))
        print("Matlab ready to rock n roll")

        rospy.init_node(nodeName,anonymous = False)
        print('Created RGBD Object Proposal Node')

        self.service = rospy.Service(serviceName, rgbd_object_proposal, self.service_callback)
        print('Created RGBD Object Proposal Service')



    def service_callback(self,data):
        print("\n### Service has been called ###")

        res = rgbd_object_proposalResponse()

        cv2ColorImg = None
        cv2DepthImg = None

        sys.stdout.write("Convert image message to cv2 image: ")
        try:
            cv2ColorImg = CvBridge().imgmsg_to_cv2(data.color)
            cv2DepthImg = CvBridge().imgmsg_to_cv2(data.depth)
        except CvBridgeError as e:
            print(e)
            return res
        print("Done")

        sys.stdout.write("Change the color order from BGR to RGB: ")
        cv2ColorImg = cv2.cvtColor(cv2ColorImg,cv2.COLOR_BGR2RGB)
        cv2DepthImg = cv2.cvtColor(cv2DepthImg,cv2.COLOR_BGR2RGB)
        print("Done")

        sys.stdout.write("Turn the cv2 images into matlab matries: ")
        matlabColorImg = matlab.uint8(cv2ColorImg.tolist())
        matlabDepthImg = matlab.uint8(cv2DepthImg.tolist())
        print("Done")

        print("Call the matlab object proposal function:\n")
        matlabProposalImg = self.eng.rgbd_object_proposal_python_interface(matlabColorImg,matlabDepthImg)
        print("\n")

        sys.stdout.write("Convert the image proposal response into numpy array: ")
        npProposalImg = np.array(matlabProposalImg._data)
        npProposalImg = npProposalImg.reshape(matlabProposalImg.size, order='F')#order='F' is very important
        npProposalImg = np.uint8(npProposalImg)
        print("Done")

        sys.stdout.write("Change the proposal color order from RGB to BGR: ")
        npProposalImg = cv2.cvtColor(npProposalImg,cv2.COLOR_RGB2BGR)
        print("Done")

        sys.stdout.write("Turn the proposal into an image message: ")
        try:
            res.proposal = CvBridge().cv2_to_imgmsg(npProposalImg)
        except CvBridgeError as e:
            print(e)
            return(res)
        print("Done")

        return res

if __name__ == '__main__':
    mobs = MatlabObjectProposalService()

    print("\nRGBD Object Proposal Service now running!!\n")
    rospy.spin()
