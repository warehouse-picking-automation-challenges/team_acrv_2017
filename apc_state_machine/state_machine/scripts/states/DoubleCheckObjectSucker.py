import rospy
import smach
import smach_ros

from tf import transformations as tft

from helpers import movement as m
from helpers import transforms as t
from helpers import suction
from helpers.robot_constants import *
from helpers.item_meta import item_meta
from helpers import weight as w
from helpers import vision

import geometry_msgs.msg as gmsg
from sensor_msgs.msg import Image
from std_msgs.msg import Int16, String
from acrv_apc_2017_perception.srv import rgbd_object_proposal

import cv2
import cv_bridge

class DoubleCheckObjectSucker(smach.State):
    def __init__(self, duration=1.0):
        smach.State.__init__(self, outcomes=['succeeded', 'classification_failed', 'failed'], input_keys=['data'], output_keys=['data'],)

        refinenet_segmentation_service_name = '/tote_classifier/refinenet_classification'
        rospy.loginfo('Waiting for %s service ...' % refinenet_segmentation_service_name)
        try:
            rospy.wait_for_service(refinenet_segmentation_service_name, timeout=1)
        except:
            rospy.logerr('Service %s not available. Restart and try again.' % refinenet_segmentation_service_name)
        self.refinenet_segmentation_service = rospy.ServiceProxy(refinenet_segmentation_service_name, rgbd_object_proposal)

        self.overlay_pub = rospy.Publisher('/state_machine/overlay', Image)

        self.cb = cv_bridge.CvBridge()

    # ==========================================================
    def execute(self, userdata):

        rospy.loginfo('MOVE TO DOUBLE CHECK POSITION')
        res = m.move_to_global(0.579, 0.608, 0.630, 'sucker')
        if not res:
            return 'failed'

        try:
            img_msg = rospy.wait_for_message('/realsense/rgb_hd/image_raw', Image, 5.0)
        except rospy.ROSException:
            # Don't reset vision, this is a different camera.
            return 'classification_failed'

        img = self.cb.imgmsg_to_cv2(img_msg, 'rgb8')
        img = img[155:155+540, 340:340+960, :]  # Crop
        img = cv2.resize(img, (640,360))

        img_msg = self.cb.cv2_to_imgmsg(img)

        visible_objects = userdata.data['double_check_objects']
        rospy.loginfo(visible_objects)
        visible_objects_ros = [String(s) for s in visible_objects]

        visible_objects_ros.append(String('tote'))
        class_res = self.refinenet_segmentation_service.call(visible_objects_ros, img_msg, img_msg, img_msg)

        self.overlay_pub.publish(class_res.overlay_img)

        labels = [l.data for l in class_res.label]
        segment_certainties = [i.data for i in class_res.segment_certainties]

        print(labels)
        print(segment_certainties)

        if len(labels):

            top_certainty, top_label = max(zip(segment_certainties, labels))

            try:
                if top_label != 'tote':
                    if top_certainty > 10:
                        if userdata.data['task'] == 'stow' or (userdata.data['task'] == 'pick' and top_label in userdata.data['wanted_items']):
                            rospy.logerr("I'M CHANGING THE OBJECT TO %s" % top_label)
                            userdata.data['item_to_pick']['label'] = top_label
                            suck_level = item_meta[top_label].get('suction_pressure', 3)
                            suction.set_suck_level(suck_level)
                            return 'succeeded'
            except:
                pass

        mid_x, mid_y = POSITION_LIMITS[userdata.data['picking_from']]['centre']
        orig = userdata.data['original_pick_position']
        if orig.position.x > mid_x:
            orig.position.x -= 0.08
        else:
            orig.position.x += 0.08
        if orig.position.y > mid_y:
            orig.position.y -= 0.08
        else:
            orig.position.y += 0.08

        orig.position.y = min(orig.position.y, SUCKER_MAX_Y_STRAIGHT)

        res = m.move_to_global(orig.position.x, orig.position.y, SUCKER_LIFT_HEIGHT + 0.01, 'sucker')
        if not res:
            rospy.logerr('Failed to perform a move')
            return 'failed'

        scales_topic = userdata.data['picking_from'] + '_scales/weight'
        res, _, _ = m.move_to_global_monitor_weight(orig.position.x, orig.position.y, orig.position.z - 0.13, 'sucker', scales_topic, velo_scale=0.2)
        if not res:
            rospy.logerr('Failed to perform a move')
            return 'failed'

        return 'classification_failed'
