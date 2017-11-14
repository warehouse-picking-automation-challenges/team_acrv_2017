import rospy

from states.helpers import movement as m
from states.helpers.robot_constants import *

from states.InitRobot import InitRobot

rospy.init_node('test')

ir = InitRobot()

class blah(object):
    def __init__(self):
        self.data = {'move_objects_mode': False,
                'move_objects_between_bins_mode': None}

b = blah()

ir.execute(b)

m.move_to_named_pose('wrist_only', 'sucker')

for location in POSITION_LIMITS:
    p = POSITION_LIMITS[location]['sucker']

    m.move_to_global(p['x_max'], p['y_min'], 0.9, 'sucker', velo_scale=0.2)
    m.move_to_global(p['x_max'], p['y_min'], p['z_max'], 'sucker', velo_scale=0.05)
    m.move_to_global(p['x_max'], p['y_min'], 0.9, 'sucker', velo_scale=0.2)

    m.move_to_global(p['x_min'], p['y_max'], 0.9, 'sucker', velo_scale=0.2)
    m.move_to_global(p['x_min'], p['y_max'], p['z_max'], 'sucker', velo_scale=0.05)
    m.move_to_global(p['x_min'], p['y_max'], 0.9, 'sucker', velo_scale=0.2)

ir.execute(b)

m.move_to_named_pose('wrist_only', 'gripper')

for location in POSITION_LIMITS:
    p = POSITION_LIMITS[location]['gripper']

    m.move_to_global(p['x_max'], p['y_min'], 0.9, 'gripper', velo_scale=0.2)
    m.move_to_global(p['x_max'], p['y_min'], p['z_max'], 'gripper', velo_scale=0.05)
    m.move_to_global(p['x_max'], p['y_min'], 0.9, 'gripper', velo_scale=0.2)

    m.move_to_global(p['x_min'], p['y_max'], 0.9, 'gripper', velo_scale=0.2)
    m.move_to_global(p['x_min'], p['y_max'], p['z_max'], 'gripper', velo_scale=0.05)
    m.move_to_global(p['x_min'], p['y_max'], 0.9, 'gripper', velo_scale=0.2)

ir.execute(b)
