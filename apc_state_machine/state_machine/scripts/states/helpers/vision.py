import rospy
import rosnode

def reset_vision():
    # This will kill the realsense node on the NUC and the image pipeline. Both are set in the launch files to auto-restart.
    rospy.logerr('Resetting the realsense nodes.')
    rosnode.kill_nodes(['/realsense_nodelet_manager', '/acrv_realsense_wrist_ros'])
    rospy.sleep(5)
