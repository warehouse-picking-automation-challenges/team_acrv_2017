import rospy
import smach

import datetime

from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2, Image

from apc_msgs.srv import DoSegmentation, DoSegmentationRequest
from apc_msgs.srv import CropCloud, CropCloudRequest
from apc_msgs.srv import DoObjectProposal, DoObjectProposalRequest
from apc_msgs.srv import SegmentPointcloudFromLabels, SegmentPointcloudFromLabelsRequest

from acrv_apc_2017_perception.srv import rgbd_object_proposal

from helpers.robot_constants import *
from helpers import vision
from helpers import json_functions

import cv_bridge
import cv2
import numpy as np

class GetObjectsRefineNet(smach.State):
    def __init__(self):
        smach.State.__init__(self, input_keys=['data'], output_keys=['data'],
                             outcomes=['succeeded', 'failed'])

        self.cb = cv_bridge.CvBridge()

        filter_service_name = '/apc_3d_vision/Crop_Tote_cloud'
        rospy.loginfo('Waiting for %s service ...' % filter_service_name)
        try:
            rospy.wait_for_service(filter_service_name, timeout=1)
        except:
            rospy.logerr('Service %s not available. Restart and try again.' % filter_service_name)
        self.filter_service = rospy.ServiceProxy(filter_service_name, CropCloud)

        refinenet_segmentation_service_name = '/tote_classifier/refinenet_classification'
        rospy.loginfo('Waiting for %s service ...' % refinenet_segmentation_service_name)
        try:
            rospy.wait_for_service(refinenet_segmentation_service_name, timeout=1)
        except:
            rospy.logerr('Service %s not available. Restart and try again.' % refinenet_segmentation_service_name)
        self.refinenet_segmentation_service = rospy.ServiceProxy(refinenet_segmentation_service_name, rgbd_object_proposal)

        pc_segment_service_name = '/segment_pointcloud_node/segment_pointcloud_from_labels'
        rospy.loginfo('Waiting for %s service ...' % pc_segment_service_name)
        try:
            rospy.wait_for_service(pc_segment_service_name, timeout=1)
        except:
            rospy.logerr('Service %s not available. Restart and try again.' % pc_segment_service_name)
        self.pc_segment_service = rospy.ServiceProxy(pc_segment_service_name, SegmentPointcloudFromLabels)

        self.overlay_pub = rospy.Publisher('/state_machine/overlay', Image)
        self.confidence_pub = rospy.Publisher('/state_machine/confidence', Image)

        self.failed_classificaitons_in_a_row = 0
        self.reclassification_failure_counts = {}

    def execute(self, userdata):
        rospy.loginfo('GET OBJECTS')
        rospy.loginfo('Getting Point Cloud and Image from Camera')
        t0 = datetime.datetime.now()

        try:
            depth = rospy.wait_for_message('/realsense_wrist/depth_registered/image_rect', Image, 5.0)
            img = rospy.wait_for_message('/realsense_wrist/rgb/image_rect', Image, 5.0)
            pc = rospy.wait_for_message('/realsense_wrist/depth_registered/points', PointCloud2, 5.0)
            rospy.logerr("waiting for images %s" % (datetime.datetime.now() - t0))
        except rospy.ROSException:
            vision.reset_vision()
            return 'failed'

        t0 = datetime.datetime.now()
        rospy.loginfo('Cropping Point Cloud and Images')
        req = CropCloudRequest()
        req.input_cloud = pc
        req.crop_cloud.data = True
        req.image = img
        req.depth = depth
        req.crop_image.data = True
        req.storage_name.data = userdata.data['camera_location']
        print(userdata.data['camera_location'])
        fil_res = self.filter_service.call(req)
        rospy.logerr("filtering %s" % (datetime.datetime.now() - t0))

        pc = fil_res.segmented_cloud_posonly
        img = fil_res.cropped_image
        depth = fil_res.cropped_depth

        userdata.data['point_cloud'] = pc

        rospy.loginfo('Calling classification service')
        visible_objects = userdata.data['visible_objects']
        rospy.loginfo(visible_objects)
        visible_objects_ros = [String(s) for s in visible_objects]

        if userdata.data['task'] == 'stow' and len(visible_objects) < 6 and not userdata.data['i_think_im_done']:
            visible_objects_ros.extend( [String(s[1]) for s in userdata.data['weight_reclassifications']] )

        visible_objects_ros.append(String('tote'))
        t0 = datetime.datetime.now()
        class_res = self.refinenet_segmentation_service.call(visible_objects_ros, img, img, depth)
        rospy.logerr("refinenet %s" % (datetime.datetime.now() - t0))

        rospy.loginfo('Remove cropped regions of the refinenet results')
        req = CropCloudRequest()
        req.crop_cloud.data = False
        req.image = class_res.overlay_img
        req.depth = class_res.classification
        req.crop_image.data = True
        req.storage_name.data = userdata.data['camera_location']
        fil_res = self.filter_service.call(req)

        # Publish the segmented image.
        self.overlay_pub.publish(fil_res.cropped_image)
        self.confidence_pub.publish(class_res.confidence_map)

        try:
            seg_img = self.cb.imgmsg_to_cv2(fil_res.cropped_depth, "mono8")
            certainty_img = self.cb.imgmsg_to_cv2(class_res.confidence_map, "mono8")
            segment_certainties = [i.data for i in class_res.segment_certainties]
        except cv_bridge.CvBridgeError:
            rospy.logerr('The refinenet service didn\'t return any data')
            return 'failed'

        t0 = datetime.datetime.now()
        if True:
            # Save the Images
            # TODO: Turn this off for real runs, saving the HD image can be a little bit slow.
            dt = datetime.datetime.now().strftime('%y%m%d_%H%M%S')
            with open('/storage/classification_output_robotronica/%s_items.txt' % dt, 'w') as f:
                for o in visible_objects:
                    f.write('%s\n' % o)

            with open('/storage/classification_output_robotronica/%s_labels.txt' % dt, 'w') as f:
                for l in class_res.label:
                    f.write('%s\n' % l.data)

            rgb_image = self.cb.imgmsg_to_cv2(img, 'bgr8')
            cv2.imwrite('/storage/classification_output_robotronica/%s_rgb.png' % dt, rgb_image)

            # rgb_image_hd = self.cb.imgmsg_to_cv2(img_hd, 'bgr8')
            # cv2.imwrite('/storage/classification_output_robotronica/%s_rgb_hd.png' % dt, rgb_image_hd)

            overlay_img = self.cb.imgmsg_to_cv2(fil_res.cropped_image, 'bgr8')
            cv2.imwrite('/storage/classification_output_robotronica/%s_overlay.png' % dt, overlay_img)

            cv2.imwrite('/storage/classification_output_robotronica/%s_certainty.png' % dt, certainty_img)

            depth_image = self.cb.imgmsg_to_cv2(depth)
            depth_image_16 = (depth_image * 65536.0/2.0).astype(np.uint16)
            cv2.imwrite('/storage/classification_output_robotronica/%s_depth.png' % dt, depth_image_16)

            cv2.imwrite('/storage/classification_output_robotronica/%s_classification.png' % dt, seg_img)
        rospy.logerr("saving images %s" % (datetime.datetime.now() - t0))

        masked_classifications = self.cb.cv2_to_imgmsg(seg_img, "mono8")

        t0 = datetime.datetime.now()
        rospy.loginfo('Segmenting point cloud using classified labels')
        req = SegmentPointcloudFromLabelsRequest()
        req.classification = masked_classifications
        req.input_cloud = pc
        seg_res = self.pc_segment_service.call(req)
        rospy.logerr("segmenting point cloud %s" % (datetime.datetime.now() - t0))

        labels = [l.data for l in class_res.label]

        undone_changes = []
        # Don't do this for robotronica because it sees things that don't exist.
        if False and userdata.data['task'] == 'stow' and len(visible_objects) < 6 and not userdata.data['i_think_im_done']:
            for f, t in userdata.data['weight_reclassifications']:
                if t in labels and f not in labels:
                    # We can see the thing we weight reclassified.
                    rospy.logerr("I'm not sure about changing %s to %s" % (f, t))
                    if (f, t) not in self.reclassification_failure_counts:
                        self.reclassification_failure_counts[(f,t)] = 1
                    else:
                        self.reclassification_failure_counts[(f,t)] += 1
                    if self.reclassification_failure_counts[(f,t)] > 2:
                        rospy.logerr("I'm undoing the change")
                        undone_changes.append( (f, t) )
                        try:
                            userdata.data['item_locations']['tote']['contents'].remove(f)
                        except ValueError:
                            # If we get here we've really messed up.
                            continue
                        userdata.data['item_locations']['tote']['contents'].append(t)
                        for b in userdata.data['item_locations']['bins']:
                            if t in b['contents']:
                                b['contents'].remove(t)
                                b['contents'].append(f)
            for u in undone_changes:
                userdata.data['weight_reclassifications'].remove(u)
            if undone_changes:
                json_functions.save_file(userdata.data['task'], 'item_location_file_out.json', userdata.data['item_locations'])

        identified_objects = {}
        identified_objects['labels'] = labels
        identified_objects['classifications'] = class_res.classification
        identified_objects['masked_classifications'] = masked_classifications
        identified_objects['point_clouds'] = seg_res.segmented_pointclouds
        identified_objects['aligned_dimensions'] = seg_res.aligned_dimensions
        identified_objects['segment_certainties'] = segment_certainties
        userdata.data['identified_objects'] = identified_objects

        view_data = {}
        view_data['labels'] = labels
        view_data['classifications'] = class_res.classification
        view_data['masked_classifications'] = seg_img
        view_data['aligned_dimensions'] = seg_res.aligned_dimensions
        view_data['segment_certainties'] = segment_certainties
        view_data['camera_location'] = userdata.data['camera_location']
        userdata.data['previous_views'].insert(0, view_data)
        userdata.data['previous_views'] = userdata.data['previous_views'][:10]


        if len(class_res.label) == 0:
            rospy.logerr('No segments returned from refinenet.')

            if userdata.data['task'] == 'pick':
                # We use multiple viewpoints for the pick task.
                userdata.data['failed_views'][userdata.data['chosen_bin']].append(userdata.data['camera_location'])

            if userdata.data['task'] == 'stow':
                self.failed_classificaitons_in_a_row += 1
                if self.failed_classificaitons_in_a_row > 2:
                    if userdata.data['i_think_im_done'] == True and self.failed_classificaitons_in_a_row > 3:
                        tote_objects = userdata.data['item_locations']['tote']['contents']
                        for b in userdata.data['item_locations']['bins']:
                            if b['bin_id'] == 'B':
                                b['contents'].extend(tote_objects)
                        userdata.data['item_locations']['tote']['contents'] = []
                        json_functions.save_file(userdata.data['task'], 'item_location_file_out.json', userdata.data['item_locations'])
                        rospy.logerr("I couldn't seen anything so I moved all of the items into the Storage B List (fingers crossed)")

                    userdata.data['i_think_im_done'] = True
                    userdata.data['visible_objects'] = list(userdata.data['all_items'])

            return 'failed'

        self.failed_classificaitons_in_a_row = 0

        return 'succeeded'
