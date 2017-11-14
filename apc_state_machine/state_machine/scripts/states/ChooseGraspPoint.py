import rospy
import smach

import random

import tf
from tf import transformations as tft
import numpy as np
import scipy.misc
from scipy import ndimage

from std_msgs.msg import Int32
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Pose
from apc_msgs.srv import DetectGraspCandidates
from apc_msgs.srv import ExtractPca
from apc_msgs.srv import ImageToWorld
from geometry_msgs.msg import Quaternion
from visualization_msgs.msg import MarkerArray, Marker

from helpers import transforms as t

from helpers.robot_constants import *

from helpers.item_meta import item_meta

import cv_bridge

def publish_grasp_markers(poses, utilities):
    delete_array = MarkerArray()
    for i in range(5000):
        m = Marker()
        m.header.frame_id = 'realsense_wrist_rgb_optical_frame'
        m.header.stamp = rospy.Time.now()
        m.ns = 'grasp_poses'
        m.id = i
        m.action = Marker.DELETE
        delete_array.markers.append(m)

    ChooseGraspPoint.marker_pub.publish(delete_array)

    pub_array = MarkerArray()
    i = 0
    gu_min = min(utilities)
    gu_max = max(utilities)
    if gu_min == gu_max:
        gu_max += 0.01  # Avoid div by 0
    for p, u in zip(poses, utilities):
        m = Marker()
        m.header.frame_id = 'realsense_wrist_rgb_optical_frame'
        m.header.stamp = rospy.Time.now()
        m.ns = 'grasp_poses'

        m.pose.position = p.position
        tfq = [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
        rot = tft.quaternion_from_euler(0, 1.57, 0)
        tfq = tft.quaternion_multiply(tfq, rot)
        m.pose.orientation.x = tfq[0]
        m.pose.orientation.y = tfq[1]
        m.pose.orientation.z = tfq[2]
        m.pose.orientation.w = tfq[3]

        m.id = i
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.scale.x = 0.035 + 0.07* ((u - gu_min)/(gu_max - gu_min)) ** 2
        m.scale.y = 0.0035
        m.scale.z = 0.0035
        m.color.a = 1.0
        if u == gu_max:
            m.color.b = 1
        else:
            m.color.r = 1 - (u - gu_min)/(gu_max - gu_min)
            m.color.g = (u - gu_min)/(gu_max - gu_min)
            m.color.b = 0
        pub_array.markers.append(m)
        i += 1

    ChooseGraspPoint.marker_pub.publish(pub_array)


def rank_grasp_poses(grasp_poses, utilities):
    if len(grasp_poses) == 0:
        return []
    new_utilities = []
    zs = [p.position.z for p in grasp_poses]
    min_z = min(zs)
    max_z = max(zs)

    for gp, u in zip(grasp_poses, utilities):
        nu = u
        # Weight by verticality, i.e. angle to vertical
        pt = [gp.position.x, gp.position.y, gp.position.z]
        pq = [gp.orientation.x, gp.orientation.y, gp.orientation.z, gp.orientation.w]
        R = tft.quaternion_matrix(pq)
        z_unit = R[0:3,2]
        val = z_unit[2]
        a = abs(np.arccos(val/np.linalg.norm(z_unit)))

        if a > 1.57:
            a = 1.57
        if a < 0.3:
            a = 0

        # nu -= np.sin(a) * 0.05

        if a > 0.5:
            # Penalise angled points that are not pointing towards the centre of the tote even more.
            P = tft.projection_matrix([0, 0, gp.position.z], [0, 0, 1])
            pose_vec = np.dot(R, [0, 0, 1, 0])
            proj_vec = tft.unit_vector(np.dot(P, pose_vec)[:3])
            vec_to_origin = tft.unit_vector([gp.position.x, gp.position.y, 0])
            a = abs(np.arccos(np.dot(proj_vec, vec_to_origin)))  # Angle to the centre of the tote.

            if a > 1.57:
                a = 1.57
            nu -= np.sin(a) * 0.1

        # Min z is at the top
        if max_z - min_z > 0.05:
            height_score = (gp.position.z - min_z)/(max_z - min_z)
            nu -= height_score * 0.1

        new_utilities.append(nu)

    # Prune the grasp points to try and get a sparser list.
    # No point having lots of grasp points right next to each other.
    # Select a grasp point at random, find its nearest neighbour, keep the best one
    # Repeat until there are only 10 grasp pointes left.
    while True:
        l = len(grasp_poses)
        if l <= 15:
            break
        ri = random.randint(0, l-1)
        gp = grasp_poses[ri]
        closest_index = -1
        closest_distance = 100
        for i in range(l):
            if i == ri:
                continue
            gpc = grasp_poses[i]
            dx = gp.position.x - gpc.position.x
            dy = gp.position.y - gpc.position.y
            dz = gp.position.z - gpc.position.z
            d = np.sqrt(dx**2 + dy**2 + dz**2)
            if d < closest_distance:
                closest_distance = d
                closest_index = i

        if new_utilities[ri] > new_utilities[closest_index]:
            # Keep the original and delete the closest one
            del grasp_poses[closest_index]
            del new_utilities[closest_index]
        else:
            del grasp_poses[ri]
            del new_utilities[ri]


    a = list(zip(new_utilities, grasp_poses))
    a.sort(reverse=True)

    grasp_poses = [g[1] for g in a]
    new_utilities = [g[0] for g in a]
    publish_grasp_markers(grasp_poses, new_utilities)

    return grasp_poses



class ChooseGraspPoint(smach.State):
    marker_pub = rospy.Publisher('/state_machine/grasp_pose_array', MarkerArray, queue_size=10)

    def __init__(self):
        smach.State.__init__(self, input_keys=['data'], output_keys=['data'],
                             outcomes=['succeeded_sucker', 'succeeded_gripper', 'failed'])

        self.camera_reference_frame = 'realsense_wrist_rgb_optical_frame'

        grasp_candidate_service_name = '/apc_grasping/detect_grasp_candidates'
        rospy.loginfo('Waiting for %s service ...' % grasp_candidate_service_name)
        try:
            rospy.wait_for_service(grasp_candidate_service_name, timeout=1)
        except:
            rospy.logerr('Service %s not available. Restart and try again.' % grasp_candidate_service_name)
        self.grasp_candidate_service = rospy.ServiceProxy(grasp_candidate_service_name, DetectGraspCandidates)

        self.pc_pub = rospy.Publisher('/state_machine/point_cloud', PointCloud2, queue_size=1)

        pca_grasp_service_name = '/apc_grasping/extract_pca'
        rospy.loginfo('Waiting for %s service ...' % pca_grasp_service_name)
        try:
            rospy.wait_for_service(pca_grasp_service_name, timeout=1)
        except:
            rospy.logerr('Service %s not available. Restart and try again.' % pca_grasp_service_name)
        self.pca_grasp_service = rospy.ServiceProxy(pca_grasp_service_name, ExtractPca)

        image_to_world_service_name = '/image_to_world_node/image_to_world'
        rospy.loginfo('Waiting for %s service ...' % image_to_world_service_name)
        try:
            rospy.wait_for_service(image_to_world_service_name, timeout=1)
        except:
            rospy.logerr('Service %s not available. Restart and try again.' % image_to_world_service_name)
        self.image_to_world_service = rospy.ServiceProxy(image_to_world_service_name, ImageToWorld)

        self.cb = cv_bridge.CvBridge()

        self.failed_last_time = False

    # ==========================================================
    def execute(self, userdata):

        rospy.loginfo('Finding best segment.')
        segments = []
        ignored_segments = []
        move_mode_seen_wanted_items = []
        for i in range(len(userdata.data['identified_objects']['labels'])):

            l = userdata.data['identified_objects']['labels'][i]
            if l == 'tote':
                continue

            if userdata.data['task'] == 'stow':
                # If we're nearing the end of the stow task, we might be checking
                #  for recalssified items, so ignore them until they're bumped out of the list.
                reclass_item = False
                for _, to_item in userdata.data['weight_reclassifications']:
                    if l == to_item:
                        reclass_item = True
                        break
                if reclass_item:
                    rospy.logerr("Ignoring %s as reclassified item" % l)
                    continue

            if userdata.data.get('move_objects_mode', False):
                # Don't filter wanted items.
                if l in userdata.data['moved_objects']:
                    continue
            elif userdata.data['wanted_items'] is not None and l not in userdata.data['wanted_items']:
                continue

            s = userdata.data['identified_objects']['point_clouds'][i]
            ad = userdata.data['identified_objects']['aligned_dimensions'][i]
            if (ad.width.data < 0.01 and ad.height.data < 0.01) and self.failed_last_time == False:
                rospy.logwarn('Ignoring %s because the point cloud is too small' % l)
                continue

            if userdata.data['identified_objects']['segment_certainties'][i] < 6 and self.failed_last_time == False:
                rospy.logwarn('Ignoring %s because the certainty isn\'t high enough' % l)
                continue

            # Round to the nearest xx cm
            round_cm = 3.0
            height_sort = round( ad.average_z.data*(100.0/round_cm)) / (100.0/round_cm)

            if userdata.data.get('move_objects_mode', False):
                # sort by size (negative one to sort smallest to largest.)
                item_weight = ad.width.data * ad.height.data

                # Weight it higher if it intersects an object that we wanted in older views.
                found_intersection = False
                for view in userdata.data['previous_views']:
                    if l in view['labels']:
                        rospy.logerr('%s is in previsous view' % l)
                        # Figure out if the view contained any objects that we're trying to find.
                        seen_wanted_items = [label for label in view['labels'] if label in userdata.data['wanted_items']]
                        if not seen_wanted_items:
                            rospy.logerr("No wanted items in that view though")
                            continue
                        rospy.logerr("There were also wanted items in that view.")
                        item_mask = (view['masked_classifications'] == (view['labels'].index(l) + 1)).astype(np.uint8)
                        for wanted_item_label in seen_wanted_items:
                            wanted_item_id = view['labels'].index(wanted_item_label) + 1
                            wanted_item_mask = (view['masked_classifications'] == wanted_item_id).astype(np.uint8)
                            wanted_item_mask = ndimage.morphology.binary_dilation(wanted_item_mask)
                            intersection = item_mask * wanted_item_mask
                            if intersection.max() > 0:
                                rospy.logerr("Found intersection with %s" % wanted_item_label)
                                found_intersection = True
                                item_weight += 10
                                break

                        if found_intersection:
                            break

                segments.append( (-1 * item_weight, s, l, ad) )
            else:
                # Sort by height in the tote and certainty of the classification.
                segments.append( ((height_sort, 255 - userdata.data['identified_objects']['segment_certainties'][i]), s, l, ad) )

        if len(segments) == 0:
            rospy.loginfo("No Segments")
            if userdata.data['move_objects_mode'] == True:
                userdata.data['moved_objects'] = []
            if userdata.data['task'] == 'pick':
                # We use multiple viewpoints for the pick task.
                userdata.data['failed_views'][userdata.data['chosen_bin']].append(userdata.data['camera_location'])
            else:
                self.failed_last_time = True
            return 'failed'

        # Sort by size, biggest first.
        segments.sort()

        grasp_poses = []
        grasp_point_type = ''

        last_failed = userdata.data['last_failed']
        ignored_segments = 1
        pick_task_break = False
        # If we don't have grasp poses, but have ignored segments,
        # Then repeat the process, because eventually one of the ignored ones will become active again.

        IGNORED_ITEMS_RESET = 3
        if userdata.data['task'] == 'stow':
            IGNORED_ITEMS_RESET = 4

        while (not grasp_poses and ignored_segments > 0) and not pick_task_break:

            if userdata.data['task'] == 'pick':
                # Stop being in a loop of trying the same object, ingoring and and not moving.
                pick_task_break = True

            rospy.logerr(last_failed)
            ignored_segments = 0
            to_remove = []
            for l in last_failed:
                last_failed[l] += 1
                if last_failed[l] > IGNORED_ITEMS_RESET:
                    to_remove.append(l)
            for l in to_remove:
                del last_failed[l]

            for s in segments:
                grasp_poses = []
                item_label = s[2]
                grasp_point_type = item_meta[item_label]['grasp_point_type']

                if item_label == 'mesh_cup' and len(segments) > 1:
                    continue

                if item_label in userdata.data['last_failed']:
                    # Ignore it for now.
                    ignored_segments += 1
                    continue

                rospy.loginfo("\033[92mI WANT TO PICK ITEM: %s, GRASP POINT TYPE: %s\033[0m" % (item_label, grasp_point_type))

                normal_succeeded = False
                centroid_succeeded = False
                rgb_succeeded = False

                # Try and get fancy grasp poses.
                if grasp_point_type == 'normal':
                    try:
                        gc_res = self.grasp_candidate_service.call(s[1], 0.01, False)  # False = Don't use centroid.
                        normal_poses = gc_res.grasp_candidates.grasp_poses.poses
                        self.pc_pub.publish(s[1])
                        if len(normal_poses) > 0:
                            normal_poses = rank_grasp_poses(normal_poses, gc_res.grasp_candidates.grasp_utilities)
                            grasp_poses.extend(normal_poses[:2])
                            normal_succeeded = True
                        else:
                            rospy.logerr('Failed to find any normal points for object.')
                    except rospy.ServiceException:
                        rospy.logerr('Failed to find any normal grasp points for object.')

                # Get the centroid grasp pose.
                if grasp_point_type in ['normal', 'centroid']:
                    try:
                        gc_res = self.grasp_candidate_service.call(s[1], 0.01, True)  # True = Use Centroid.
                        centroid_poses = gc_res.grasp_candidates.grasp_poses.poses
                        if len(centroid_poses) > 0:
                            cp = centroid_poses[0]
                            # Make it straight up and down.
                            cp.orientation.x = 0
                            cp.orientation.y = 0
                            cp.orientation.z = 0
                            cp.orientation.w = 1
                            grasp_poses.append(cp)
                            centroid_succeeded = True
                        else:
                            rospy.logerr('Failed to find any centroid points for object.')
                    except IndexError:
                        rospy.logerr('Failed to find any centroid points for object.')

                if grasp_point_type == 'rgb_centroid' or \
                   (grasp_point_type in ['normal', 'centroid'] and centroid_succeeded == False):
                    try:
                        # Calculate the real world location of the RGB centroid.
                        index = userdata.data['identified_objects']['labels'].index(item_label)
                        class_mask = np.equal(index + 1, self.cb.imgmsg_to_cv2(userdata.data['identified_objects']['masked_classifications'], 'mono8')).astype(np.uint8)
                        com = ndimage.measurements.center_of_mass(class_mask)
                        img_y = Int32(com[0])
                        img_x = Int32(com[1])
                        res = self.image_to_world_service.call(userdata.data['point_cloud'], img_x, img_y)
                        p = Pose()
                        p.position.x = res.world_x.data
                        p.position.y = res.world_y.data
                        p.position.z = 1.2  # big number, will get overridden to the storage bottom.
                        p.orientation.w = 1
                        grasp_poses.append(p)
                        rgb_succeeded = True

                    except IndexError:
                        rospy.logerr('Failed to calculate RGB Centroid grasp point for object')

                if normal_succeeded == False and centroid_succeeded == False and rgb_succeeded == False:
                    continue

                break

        # Make sure we actually got something.
        if not grasp_poses:
            rospy.logerr('Didn\'t find grasp points for any objects')
            if userdata.data['move_objects_mode'] == True:
                userdata.data['moved_objects'] = []
            if userdata.data['task'] == 'pick':
                userdata.data['failed_views'][userdata.data['chosen_bin']].append(userdata.data['camera_location'])
            else:
                self.failed_last_time = True
            return 'failed'

        self.failed_last_time = False

        # Convert the grasp poses to the global reference frame
        converted_poses = []
        for p in grasp_poses:
            converted_poses.append(t.convert_pose(p, self.camera_reference_frame, 'global_xyz_link'))

        # Get the PCA Rotation of the RGB segment because its more reliable than the point cloud.
        index = userdata.data['identified_objects']['labels'].index(item_label)
        class_mask = np.equal(index + 1, self.cb.imgmsg_to_cv2(userdata.data['identified_objects']['masked_classifications'], 'mono8')).astype(np.uint8)
        y, x = np.nonzero(class_mask.squeeze())
        x = x - np.mean(x)
        y = y - np.mean(y)
        coords = np.vstack([x, y])
        cov = np.cov(coords)
        evals, evecs = np.linalg.eig(cov)
        sort_indices = np.argsort(evals)[::-1]
        evec1, evec2 = evecs[:, sort_indices]
        eig_x, eig_y = evec2
        angle = np.arctan2(eig_y, eig_x)
        if abs(eig_y) > abs(eig_x):
            angle *= -1.0
        if angle > 1.57:
            angle -= 3.14
        if angle < -1.57:
            angle += 3.14
        q = tft.quaternion_from_euler(0, 0, angle)
        pca_rotation = Quaternion()
        pca_rotation.x = q[0]
        pca_rotation.y = q[1]
        pca_rotation.z = q[2]
        pca_rotation.w = q[3]

        userdata.data['item_to_pick'] = {
            'label': item_label,
            'grasp_poses': converted_poses,
            'pointcloud': s[1],
            'grasp_point_type': grasp_point_type,
            'pca_yaw': pca_rotation,
            'aligned_dimensions': s[3]
        }

        if (userdata.data['move_objects_mode'] or userdata.data['move_objects_between_bins_mode'] is not None) and userdata.data['wanted_items'] is not None and item_label in userdata.data['wanted_items']:
            # Just in case we somehow now pick something we want.
            userdata.data['move_objects_mode'] = False
            userdata.data['move_objects_between_bins_mode'] = None
        else:
            userdata.data['moved_objects'].append(item_label)

        rospy.loginfo('Generated %s grasp poses for object.' % len(grasp_poses))

        # Publish the point cloud segment for debugging purposes.
        self.pc_pub.publish(s[1])

        # Select sucking or grasping.
        grasp_type = item_meta[item_label]['grasp_type']
        if grasp_type.startswith('grip'):
            return 'succeeded_gripper'
        elif grasp_type.startswith('suck'):
            return 'succeeded_sucker'
        else:
            rospy.logerr("UNKNOWN GRASP TYPE %s for %s" % (grasp_type, item_label))
            return 'succeeded_sucker'

        return 'failed'
