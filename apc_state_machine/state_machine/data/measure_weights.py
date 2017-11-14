#! /usr/bin/env python

import os
import json

import rospy

from std_msgs.msg import Int16


rospy.init_node('meaure_weights')

measured_weights = {}
try:
    with open('measured_weights.json', 'r') as f:
        measured_weights = json.load(f)
except:
    pass

scales_topic = '/stow_tote_scales/weight'

raw_input("Confirm Empty Tote")
empty_weight = rospy.wait_for_message(scales_topic, Int16).data / 1000.0

# Load each item that we have data for.
for d, dirs, files in os.walk('item_data'):
  for f in files:
    if f.endswith('.json'):
      #print 'Loading metadata from %s' % f
      with open(os.path.join(d, f), 'r') as jf:
        print(jf)
        j = json.load(jf)
        item_name = j['name']

        if item_name not in measured_weights:
            raw_input("Put %s into the tote and press ENTER" % item_name)
            weight = rospy.wait_for_message(scales_topic, Int16).data/1000.0 - empty_weight
            print("Recorded %skg" % weight)
            measured_weights[item_name] = round(weight, 3)

with open('measured_weights.json', 'w') as f:
    json.dump(measured_weights, f, indent=4, separators=(',', ': '))
