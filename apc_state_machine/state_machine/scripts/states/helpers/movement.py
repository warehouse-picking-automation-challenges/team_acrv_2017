import rospy
import std_msgs.msg

import moveit_lib.srv as msrv
import controller_manager_msgs.srv as cmsv
from moveit_msgs.msg import MoveGroupActionResult
from std_srvs.srv import Trigger
from std_msgs.msg import Int16
import geometry_msgs.msg as gmsg
import apc_msgs.srv
import tf
import tf2_ros
import transforms

import suction

import inspect

def get_calling_function_name():
    return inspect.stack()[2][3]

MOVE_GROUP_LINKS = {
    'whole_arm': 'pitch_link',
    'gripper': 'gripper_endpoint',
    'sucker': 'sucker_endpoint',
    'realsense': 'realsense_endpoint',
    'wrist_only': 'pitch_link'
}

moveit_service_name = '/moveit_lib/move_robot_pose'
rospy.loginfo('Waiting for %s service...' % moveit_service_name)
try:
    rospy.wait_for_service(moveit_service_name, timeout=1)
except:
    rospy.logerr('Service %s not available. Try again.' % moveit_service_name)
mrp_pose_service = rospy.ServiceProxy(moveit_service_name, msrv.move_robot_pose)


moveit_service_name = '/moveit_lib/move_robot_named'
rospy.loginfo('Waiting for %s service...' % moveit_service_name)
try:
    rospy.wait_for_service(moveit_service_name, timeout=1)
except:
    rospy.logerr('Service %s not available. Try again.' % moveit_service_name)
mrp_named_service = rospy.ServiceProxy(moveit_service_name, msrv.move_robot_named)


list_controllers_service_name = '/controller_manager/list_controllers'
rospy.loginfo('Waiting for %s service...' % list_controllers_service_name)
try:
    rospy.wait_for_service(list_controllers_service_name, timeout=1)
except:
    rospy.logerr('Service %s not available. Try again.' % list_controllers_service_name)
list_controllers_service = rospy.ServiceProxy(list_controllers_service_name, cmsv.ListControllers)


switch_controllers_service_name = '/controller_manager/switch_controller'
rospy.loginfo('Waiting for %s service...' % switch_controllers_service_name)
try:
    rospy.wait_for_service(switch_controllers_service_name, timeout=1)
except:
    rospy.logerr('Service %s not available. Try again.' % switch_controllers_service_name)
switch_controllers_service = rospy.ServiceProxy(switch_controllers_service_name, cmsv.SwitchController)


stop_execution_service_name = '/moveit_lib/stop_async'
rospy.loginfo('Waiting for %s service...' % stop_execution_service_name)
try:
    rospy.wait_for_service(stop_execution_service_name, timeout=1)
except:
    rospy.logerr('Service %s not available. Try again.' % stop_execution_service_name)
stop_execution_service = rospy.ServiceProxy(stop_execution_service_name, Trigger)


def move_to_named_pose(move_group, named_pose, velo_scale=1.0):
    """
    Move move_group to a named pose.
    """
    global mrp_named_service
    rospy.loginfo('MOVING TO NAMED POSE')

    change_controller(move_group)

    req = msrv.move_robot_namedRequest()
    req.move_group.data = move_group
    req.named_pose.data = named_pose
    req.max_attempts.data = 3
    req.velocity_scaling_factor.data = velo_scale

    rospy.loginfo('Moving %s to named pose %s' % (move_group, named_pose))
    return mrp_named_service(req).success.data


def move_to(pose, reference_frame, move_group, velo_scale=1.0, plan_time=1.0, attempts=3, async=False):
    """
    Move the robot to a pose
        pose                -> Desired robot pose (geometry_msgs.msg/Pose)
        reference_frame     -> Name of reference frame of pose (string)
        move_group          -> Name of move group to move (string)
        velo_scale          -> 0-1 defining the scaling of maximum velocity
        plan_time           -> Max time allowed for move_it to plan a movement
        attempts            -> Number of allowed attempts for movement planning
    """
    global mrp_pose_service
    change_controller(move_group)

    req = msrv.move_robot_poseRequest()
    req.target_pose.header.seq = 0
    req.target_pose.header.stamp = rospy.Time.now()
    req.target_pose.header.frame_id = reference_frame
    req.velocity_scaling_factor.data = velo_scale
    req.max_attempts.data = attempts
    req.max_planning_time.data = plan_time
    req.target_pose.pose = pose
    req.move_group.data = move_group
    req.async.data = async

    #Execute move request and log the request
    rospy.loginfo('EXECUTING MOVE REQUEST')
    res = mrp_pose_service(req).success.data
    return res


def move_to_global(x, y, z, move_group, orientation=None, velo_scale=1.0, plan_time=1.0, attempts=3, async=False):
    """
    Move to a point in the global x,y,z frame defined by global_xyz_link.
    If no orientation is given, then the current robot orientation is kept.
        x,y,z               -> xyz location in the global reference_frame 'global_xyz_link'
        orientation         -> geometry_msgs.msg/Quaternion
        move_group          -> Move group name.
        velo_scale          -> Movement velocity (0-1)
        plan_time           -> Max time allowed for move_it to plan a movement
        attempts            -> Number of allowed attempts for movement planning
    """
    rospy.loginfo('MOVING TO GLOBAL POSITION')

    if move_group not in MOVE_GROUP_LINKS:
        rospy.logerr('%s is not a valid move group' % move_group)
        return False
    move_group_rf = MOVE_GROUP_LINKS[move_group]

    #Create pose from x y z
    global_pose = gmsg.Pose()
    global_pose.position.x = x
    global_pose.position.y = y
    global_pose.position.z = z

    # Set the orientation for the end effector.
    if orientation is None:
        #Create orientation from get robot pose
        current_pose = transforms.current_robot_pose('global_xyz_link', move_group_rf)
        if current_pose is None:
            rospy.logerr('Failed to get current robot pose')
            return False
        global_pose.orientation = current_pose.orientation
    else:
        global_pose.orientation = orientation

    # Call the move_to function.
    #transforms.publish_pose_as_transform(global_pose, 'global_xyz_link', 'MOVE HERE', 1.0)
    res = move_to(global_pose, 'global_xyz_link', move_group, velo_scale, plan_time, attempts, async)
    return res


execution_complete = False
execution_status = False
def execution_complete_callback(msg):
    global execution_complete
    global execution_status
    execution_status = msg.status.status == 3
    execution_complete = True


current_weight = -1
def weight_callback(msg):
    global current_weight
    current_weight = msg.data


def move_to_global_monitor_weight(x, y, z, move_group, scales_topic, orientation=None, velo_scale=1.0, plan_time=1.0, attempts=1):
    global execution_complete
    global execution_status
    global current_weight
    execution_complete = False
    execution_status = False
    current_weight = -1

    try:
        init_weight = rospy.wait_for_message(scales_topic, Int16, 1.5).data
    except:
        rospy.logerr('Unable to read %s, ignoring weight' % scales_topic)
        init_weight = -1

    move_sub = rospy.Subscriber('/move_group/result', MoveGroupActionResult, execution_complete_callback)
    weight_sub = rospy.Subscriber(scales_topic, Int16, weight_callback, queue_size=1)
    rospy.sleep(0.05)  # Otherwise if the async move request return immediately then the subscriber won't be ready yet sometimes.

    move_to_global(x, y, z, move_group, orientation, velo_scale, plan_time, attempts, async=True)

    stopped = False
    bail = False
    while not execution_complete:
        if current_weight > -1 and current_weight - init_weight > 50:
            stop_execution_service.call()
            rospy.logerr('ABORTED MOVEMENT DUE TO WEIGHT CHANGE')
            rospy.logerr(current_weight - init_weight)
            execution_status = True
            stopped = True
            if (current_weight - init_weight) > 200:  # Hit hard.
                bail = True
            break
        rospy.sleep(0.01)

    move_sub.unregister()
    weight_sub.unregister()

    return (execution_status, stopped, bail)


def move_to_global_monitor_weight_and_suction(x, y, z, move_group, scales_topic, orientation=None, velo_scale=1.0, plan_time=1.0, attempts=1):
    global execution_complete
    global execution_status
    global current_weight
    execution_complete = False
    execution_status = False
    current_weight = -1

    try:
        init_weight = rospy.wait_for_message(scales_topic, Int16, 1.5).data
    except:
        rospy.logerr('Unable to read %s, ignoring weight' % scales_topic)
        init_weight = -1

    move_sub = rospy.Subscriber('/move_group/result', MoveGroupActionResult, execution_complete_callback)
    weight_sub = rospy.Subscriber(scales_topic, Int16, weight_callback, queue_size=1)
    rospy.sleep(0.05)  # Otherwise if the async move request return immediately then the subscriber won't be ready yet sometimes.

    move_to_global(x, y, z, move_group, orientation, velo_scale, plan_time, attempts, async=True)

    stopped = False
    bail = False
    while not execution_complete:
        suck_state = False
        try:
            suck_state = suction.check_pressure_sensor()
        except:
            rospy.logerr('Encountered and error reading the pressure switch.')
            pass

        if suck_state:
            stop_execution_service.call()
            rospy.logerr('ABORTED MOVEMENT ON SUCTION')
            execution_status = True
            stopped = True
            break
        if init_weight > -1 and current_weight - init_weight > 50:
            stop_execution_service.call()
            rospy.logerr('ABORTED MOVEMENT DUE TO WEIGHT CHANGE')
            rospy.logerr(current_weight - init_weight)
            execution_status = True
            stopped = True
            if (current_weight - init_weight) > 200:  # Hit hard.
                bail = True
            break
        rospy.sleep(0.02)

    if execution_complete:
        rospy.logerr("DETECTED MOVEMENT COMPLETE")

    move_sub.unregister()
    weight_sub.unregister()

    return (execution_status, stopped, bail)



def move_relative(dx, dy, dz, reference_frame, move_group, velo_scale=1.0, plan_time=1.0, attempts=1):
    """
    Move relative to current position by the offset given in the specified reference frame_id.
    Orientation is unchanged.
        dx,dy,dz            -> Offset distance (metres) in xyz
        reference_frame     -> Reference frame of the offset
        move_group          -> Move group name.
        velo_scale          -> Movement velocity (0-1)
        plan_time           -> Max time allowed for move_it to plan a movement
        attempts            -> Number of allowed attempts for movement planning
    """
    rospy.loginfo('MOVING TO RELATIVE POSITION')

    # Transform the current robot pose into the desired reference_frame
    if move_group not in MOVE_GROUP_LINKS:
        rospy.logerr('%s is not a valid move group' % move_group)
        return False
    move_group_rf = MOVE_GROUP_LINKS[move_group]

    current_pose = transforms.current_robot_pose(reference_frame, move_group_rf)

    # Add increment
    current_pose.position.x += dx
    current_pose.position.y += dy
    current_pose.position.z += dz

    # Call the move_to function
    return move_to(current_pose, reference_frame, move_group, velo_scale, plan_time, attempts)


def change_controller(move_group, second_try=False):
    """
    Changes between motor controllers
    move_group     -> Name of required move group.
    """
    global list_controllers_service
    global switch_controllers_service

    controller_map = {
        'gripper': 'cartesian_motor_controller',
        'whole_arm': 'cartesian_motor_controller',
        'realsense': 'cartesian_motor_controller_realsense',
        'sucker': 'cartesian_motor_controller_sucker',
        'wrist_only': 'cartesian_motor_controller_wrist'
    }

    rospy.loginfo('SWITCHING CONTROLLERS')

    if move_group not in controller_map:
        rospy.logerr('%s is not a valid move group for switching controllers' % move_group)
        return False

    wanted_controller = controller_map[move_group]

    c_list = list_controllers_service.call()
    running_controllers = []
    for c in c_list.controller:
        if c.name == 'joint_state_controller':
            continue
        if c.name == wanted_controller and c.state == 'running':
            rospy.loginfo('Controller %s is already running' % wanted_controller)
            return True
        if c.state == 'running':
            running_controllers.append(c.name)

    controllerSwitch = cmsv.SwitchControllerRequest()
    controllerSwitch.strictness = 1
    controllerSwitch.start_controllers = [wanted_controller]
    controllerSwitch.stop_controllers = running_controllers

    # Return True if controller was successfully switched
    res = switch_controllers_service(controllerSwitch).ok
    if res:
        rospy.loginfo('Successfully switched controllers for move group %s' % move_group)
        return res
    elif second_try == False:
        rospy.logerr('Failed to switch controllers for move group %s' % move_group)
        rospy.sleep(1.0)
        return change_controller(move_group, True)
    else:
        return False
