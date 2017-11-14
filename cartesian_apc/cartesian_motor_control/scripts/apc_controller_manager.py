#! /usr/bin/env python

import rospy

from controller_manager import ControllerManager

from dynamixel_controllers.srv import StartController
from dynamixel_controllers.srv import StartControllerResponse
from dynamixel_controllers.srv import StopController
from dynamixel_controllers.srv import StopControllerResponse
from dynamixel_controllers.srv import RestartController
from dynamixel_controllers.srv import RestartControllerResponse

class APCControllerManager(ControllerManager):
    def stop_controller(self, req):
        controller_name = req.controller_name
        self.stop_controller_lock.acquire()

        if controller_name in self.controllers:
            self.controllers[controller_name].stop()
            del self.controllers[controller_name]
            self.stop_controller_lock.release()
            return StopControllerResponse(True, 'controller %s successfully stopped.' % controller_name)
        else:
            self.stop_controller_lock.release()
            return StopControllerResponse(False, 'controller %s was not running.' % controller_name)

    def restart_controller(self, req):
        stop_req = StopController()
        stop_req.controller_name = req.controller_name
        response1 = self.stop_controller(stop_req)
        response2 = self.start_controller(req)
        res = RestartControllerResponse()
        res.success = response1.success and response2.success
        res.reason = '%s\n%s' % (response1.reason, response2.reason)
        return res

if __name__ == '__main__':
    try:
        manager = APCControllerManager()
        rospy.spin()
    except rospy.ROSInterruptException: pass
