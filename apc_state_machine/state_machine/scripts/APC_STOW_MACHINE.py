#! /usr/bin/env python

import roslib
import rospy
import smach
import smach_ros

import threading

from states.InitData import InitData
from states.TestJoints import TestJoints
from states.InitRobot import InitRobot
from states.StopAbort import StopAbort

from states.GetEmptySpace import GetEmptySpace

from states.MoveToHome import MoveToHome
from states.MoveToSafe import MoveToSafe
from states.MoveToStowTote import MoveToStowTote

#from states.GetObjectsMATLAB import GetObjectsMATLAB
from states.GetObjectsRefineNet import GetObjectsRefineNet
from states.ChooseGraspPoint import ChooseGraspPoint
from states.GetEmptySpace import GetEmptySpace

from states.ToolChange import ToolChange

from states.SuckObject import SuckObject
from states.GripObject import GripObject

from states.LiftObjectGripper import LiftObjectGripper
from states.LiftObjectSucker import LiftObjectSucker

from states.DropObjectGripper import DropObjectGripper
from states.DropObjectSucker import DropObjectSucker

from states.DoubleCheckObjectSucker import DoubleCheckObjectSucker


if __name__ == '__main__':
    rospy.init_node('smach_shelf_reach_testing')

    # Create the top level SMACH state machine
    sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])

    with sm:
        sm.add('init_data', InitData(task='stow'),
                transitions={'succeeded':'test_joints'})

        sm.add('test_joints', TestJoints(),
                transitions={'succeeded':'init_robot',
                             'failed':'stop_abort'})

        sm.add('init_robot', InitRobot(),
                transitions={'succeeded':'get_empty_space',
                             'failed':'stop_abort'})

        sm.add('get_empty_space', GetEmptySpace(),
                transitions={'succeeded':'move_to_stow_tote',
                             'failed':'init_robot'})

        sm.add('move_to_stow_tote', MoveToStowTote(),
                transitions={'succeeded':'get_objects',
                             'failed':'init_robot'})

        sm.add('get_objects', GetObjectsRefineNet(),
                transitions={'succeeded':'choose_grasp_point',
                             'failed':'get_objects'})

        sm.add('choose_grasp_point', ChooseGraspPoint(),
        transitions={'succeeded_sucker':'get_object_sucker',
                     'succeeded_gripper':'get_object_gripper',
                     'failed':'get_objects'})

        sm.add('get_object_sucker', SuckObject(),
                transitions={'succeeded':'lift_object_sucker',
                             'failed':'init_robot',
                             'suck_failed': 'init_robot',
                             'try_gripper': 'get_object_gripper'})

        sm.add('lift_object_sucker', LiftObjectSucker(),
                transitions={'succeeded':'drop_object_sucker',
                             'failed':'init_robot',
                             'suck_failed': 'init_robot',
                             'try_gripper': 'get_object_gripper',
                             'secondary_check': 'sucker_secondary_check'})

        sm.add('drop_object_sucker', DropObjectSucker(),
                transitions={'succeeded':'init_robot',
                             'failed':'init_robot'})

        sm.add('get_object_gripper', GripObject(),
                transitions={'succeeded':'lift_object_gripper',
                             'failed':'init_robot',
                             'grip_failed':'init_robot',
                             'try_sucker': 'get_object_sucker'})

        sm.add('lift_object_gripper', LiftObjectGripper(),
                transitions={'succeeded':'drop_object_gripper',
                             'grip_failed': 'init_robot',
                             'failed':'init_robot',
                             'try_sucker': 'get_object_sucker'})

        sm.add('drop_object_gripper', DropObjectGripper(),
                transitions={'succeeded':'init_robot',
                             'failed':'init_robot'})

        sm.add('sucker_secondary_check', DoubleCheckObjectSucker(),
                transitions = {'succeeded': 'drop_object_sucker',
                               'classification_failed': 'init_robot',
                               'failed': 'init_robot'})

        # Do Nothing
        sm.add('stop_abort', StopAbort(),
                transitions={'loop':'stop_abort'})


    # Create and start the introspection server
    #  (smach_viewer is broken in indigo + 14.04, so need for that now)
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # run the state machine
    #   We start it in a separate thread so we can cleanly shut it down via CTRL-C
    #   by requesting preemption.
    #   The state machine normally runs until a terminal state is reached.
    smach_thread = threading.Thread(target=sm.execute)
    smach_thread.start()
    # sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()

    # request the state machine to preempt and wait for the SM thread to finish
    sm.request_preempt()
    smach_thread.join()

    # stop the introspection server
    sis.stop()
