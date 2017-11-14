import rospy
import smach
from helpers import json_functions
from helpers import boxes
from helpers import suction
from state_machine.srv import add_box, add_boxRequest

class InitData(smach.State):
    def __init__(self, task=None):
        smach.State.__init__(self, outcomes=['succeeded'], input_keys=['data'], output_keys=['data'])
        self.task = task

    def execute(self, userdata):
        suction.pump_off()

        userdata.data = {}

        userdata.data['camera_location'] = ''
        userdata.data['last_failed'] = {}
        userdata.data['weight_failures'] = {}
        userdata.data['weight_reclassifications'] = []

        userdata.data['move_objects_mode'] = False  # For the pick task.
        userdata.data['move_objects_between_bins_mode'] = None
        userdata.data['moved_objects'] = []

        userdata.data['previous_views'] = []

        if self.task == 'stow':
            userdata.data['task'] = 'stow'
            userdata.data['item_locations'] = json_functions.load_file('stow', 'item_location_file.json')
            userdata.data['wanted_items'] = None  # Treated as All in ChooseGraspPoint
            userdata.data['empty_space_capture'] = {'storage_A': None, 'storage_B': None}
            userdata.data['last_drop_bin'] = 'A'
            userdata.data['visible_objects'] = userdata.data['item_locations']['tote']['contents']
            userdata.data['all_items'] = list(userdata.data['visible_objects'])
            userdata.data['i_think_im_done'] = False

        if self.task == 'pick':
            userdata.data['task'] = 'pick'
            userdata.data['item_locations'] = json_functions.load_file('pick', 'item_location_file.json')
            userdata.data['boxes'] = boxes.boxes
            userdata.data['empty_space_capture'] = {}
            for b in boxes.boxes:
                userdata.data['empty_space_capture'][b] = None
            userdata.data['order'] = boxes.order
            userdata.data['wanted_items'] = boxes.wanted_items
            userdata.data['chosen_bin'] = ''
            userdata.data['failed_views'] = {'A': [], 'B': []}

        return 'succeeded'
