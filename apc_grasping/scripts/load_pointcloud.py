#!/usr/bin/env python

import rospy
import smach
import std_msgs.msg
import bottle_detector.srv

import apc_msgs.srv

from geometry_msgs.msg import Pose
from sensor_msgs.msg import PointCloud2, Image
from apc_msgs.msg import PointcloudRGB
from apc_msgs.srv import ExtractPca

import tf2_ros

def execute():

    grasp_service_name = '/apc_grasping/extract_pca'
    rospy.loginfo('Waiting for %s service ...' % grasp_service_name)
    try:
        rospy.wait_for_service(grasp_service_name, timeout=1)
    except:
        rospy.logerr('Service %s not available. Restart and try again.' % grasp_service_name)
    grasp_service = rospy.ServiceProxy(grasp_service_name, ExtractPca)

    rospy.loginfo('[CARTESIAN GRASP] Getting Point Cloud and Image from Camera')
    pc = rospy.wait_for_message('/realsense_wrist/depth_registered/points', PointCloud2)
    img = rospy.wait_for_message('/realsense_wrist/rgb/image_raw', Image)

    rospy.loginfo('[CARTESIAN GRASP] Getting grasp candidates')
    grasp_request = ExtractPcaRequest()
    grasp_request.cloud = pc
    #Visualize the pca in a new window
    grasp_request.visualize = False
    #Publish the transform to view in rviz
    grasp_request.publish = False
    gc_res = grasp_service.call(grasp_request)

    rospy.login('[CARTESIAN GRASP] Got a grasp pose: %s' % gc_res)

if __name__ == '__main__':

    rospy.init_node('extract_pca_pose_node')
    while not rospy.is_shutdown():
        execute()
