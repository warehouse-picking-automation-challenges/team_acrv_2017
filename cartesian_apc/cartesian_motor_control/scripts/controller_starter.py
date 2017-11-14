#! /usr/bin/env python
import rospy as rp
import controller_manager_msgs.srv as cmm
import time

controller_set = set()
controller_set.add('joint_state_controller')
controller_set.add('cartesian_motor_controller')

if __name__ == '__main__':
    rp.init_node('controller_starter')
    lp = rp.ServiceProxy('/controller_manager/list_controllers', cmm.ListControllers)
    lp.wait_for_service()
    while True:
        res = lp()
        controllers = set([r.name for r in res.controller])
        if controller_set <= controllers:
            break
        time.sleep(0.5)
    sp = rp.ServiceProxy('/controller_manager/switch_controller', cmm.SwitchController)
    sp.wait_for_service()
    req = cmm.SwitchControllerRequest()
    req.start_controllers.append('joint_state_controller')
    req.start_controllers.append('cartesian_motor_controller')
    req.strictness = req.BEST_EFFORT
    res = sp(req)
    if res.ok == True:
        rp.loginfo('Controllers started')
