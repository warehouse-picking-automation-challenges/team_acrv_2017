import rospy
import smach
import std_msgs.msg
import bottle_detector.srv

from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Pose
from apc_msgs.msg import PointcloudRGB
from apc_msgs.srv import CropCloud, CropCloudRequest

from helpers import movement as m
from helpers import transforms as t
from helpers.boxes import boxes
from helpers.robot_constants import *
from helpers import vision

class GetEmptySpace(smach.State):
    def __init__(self):
        smach.State.__init__(self, input_keys=['data'], output_keys=['data'],
                             outcomes=['succeeded', 'failed'])

        filter_service_name = '/apc_3d_vision/Crop_Tote_cloud'
        rospy.loginfo('Waiting for %s service ...' % filter_service_name)
        try:
            rospy.wait_for_service(filter_service_name, timeout=1)
        except:
            rospy.logerr('Service %s not available. Restart and try again.' % filter_service_name)
        self.filter_service = rospy.ServiceProxy(filter_service_name, CropCloud)

        self.pc_pub = rospy.Publisher('/state_machine/empty_space_cropped', PointCloud2)

    # ==========================================================
    def get_point_cloud_and_position(self, storage):
        try:
            pc = rospy.wait_for_message('/realsense_wrist/depth_registered/points', PointCloud2, 5.0)
            pc = rospy.wait_for_message('/realsense_wrist/depth_registered/points', PointCloud2, 5.0)
        except rospy.ROSException:
            vision.reset_vision()
            return None, None

        rospy.loginfo('\tRemoving storage system from point cloud')
        fil_request = CropCloudRequest()
        fil_request.input_cloud = pc
        fil_request.crop_image.data = False
        fil_request.crop_cloud.data = True
        fil_request.storage_name.data = storage
        res_fil = self.filter_service.call(fil_request)
        pc = res_fil.segmented_cloud_posonly
        pose = t.current_robot_pose('global_xyz_link', 'realsense_wrist_rgb_optical_frame')
        return pc, pose

    def execute(self, userdata):

        if userdata.data['task'] == 'stow':
            if userdata.data['empty_space_capture'].get('storage_A') is None:
                # We need to update storage_A
                res = m.move_to_named_pose('realsense', 'realsense_above_storage_system_A')
                if not res:
                    return 'failed'

                pc, pose = self.get_point_cloud_and_position('storage_A')
                if pc is None and pose is None:
                    return 'failed'
                userdata.data['empty_space_capture']['storage_A'] = {'point_cloud': pc, 'pose': pose}

            if userdata.data['empty_space_capture'].get('storage_B') is None:
                # We need to update storage_A
                res = m.move_to_named_pose('realsense', 'realsense_above_storage_system_B')
                if not res:
                    return 'failed'

                pc, pose = self.get_point_cloud_and_position('storage_B')
                if pc is None and pose is None:
                    return 'failed'
                userdata.data['empty_space_capture']['storage_B'] = {'point_cloud': pc, 'pose': pose}

        elif userdata.data['task'] == 'pick':
            for b in boxes:
                if userdata.data['empty_space_capture'].get(b) is None:
                    bd = boxes[b]
                    # Find centre of box in x direction
                    x_pos = (bd['right'] + bd['left'])/2
                    x_pos = min(x_pos, REALSENSE_MAX_GLOBAL_X)
                    x_pos = max(x_pos, REALSENSE_MIN_GLOBAL_X)

                    y_pos = 0.819  # This stops us hitting the gripper on the frame.

                    res = m.move_to_global(x_pos, y_pos, 0.575, 'realsense')
                    if not res:
                        return 'failed'

                    rospy.set_param('/storage_specs/%s/h_high' % b, 180)
                    rospy.set_param('/storage_specs/%s/h_inside' % b, True)
                    rospy.set_param('/storage_specs/%s/h_low' % b, 0)
                    rospy.set_param('/storage_specs/%s/s_high' % b, 255)
                    rospy.set_param('/storage_specs/%s/s_low' % b, 0)
                    rospy.set_param('/storage_specs/%s/v_high' % b, 255)
                    rospy.set_param('/storage_specs/%s/v_low' % b, 0)

                    rospy.set_param('/storage_specs/%s/x_max' % b, (x_pos - bd['right']) + 0.02)
                    rospy.set_param('/storage_specs/%s/x_min' % b, (x_pos - bd['left']) - 0.02)
                    rospy.set_param('/storage_specs/%s/y_max' % b, (y_pos - bd['bottom']) + 0.02)
                    rospy.set_param('/storage_specs/%s/y_min' % b, (y_pos - bd['top']) - 0.04)
                    rospy.set_param('/storage_specs/%s/z_max' % b, 1000)
                    rospy.set_param('/storage_specs/%s/z_min' % b, 0.4)

                    pc, pose = self.get_point_cloud_and_position(b)
                    if pc is None and pose is None:
                        return 'failed'
                    userdata.data['empty_space_capture'][b] = {'point_cloud': pc, 'pose': pose}
                    self.pc_pub.publish(pc)

        return 'succeeded'
