import os
import json

import rospy
import rospkg

from collections import OrderedDict

rospack = rospkg.RosPack()
data_path = rospack.get_path('state_machine') + '/data'
print(data_path)

# This is our item information.  It gets updated with the amazon metadata at the bottom of this file.
with open(data_path + '/acrv_item_data.json', 'r') as f:
  item_meta = json.load(f)

with open(data_path + '/acrv_item_data.json', 'r') as f:
  item_meta_update = OrderedDict(json.load(f))

# Update the item metadata with the information provided by amazon
for d, dirs, files in os.walk(data_path + '/item_data'):
  for f in files:
    if f.endswith('.json'):
      print 'Loading metadata from %s' % f
      with open(os.path.join(d, f), 'r') as jf:
        j = json.load(jf)
        item_name = j['name']
        if item_name not in item_meta:
          # Set some defaults so we don't crash out.
          print('YOU HAVENT CONFIGURED %s IN acrv_item_data.json. ' % item_name)
          grasp_type = raw_input('\nGrasp Type: [suck/grip/suck_grip/grip_suck]: ')
          grasp_point_type = raw_input('Grasp Point Type: [normal/centroid/rgb_centroid]: ')
          suction_pressure = int(raw_input('Suction Pressure [1-3]: '))
          ignore_weight = bool(int(raw_input('Ingore Weight [1/0]: ')))

          item_meta[item_name] =     {'grasp_type': grasp_type,
                                      'grasp_point_type': grasp_point_type,
                                      'suction_pressure': suction_pressure,
                                      'ignore_weight': ignore_weight}

          item_meta_update[item_name] = {'grasp_type': grasp_type,
                              'grasp_point_type': grasp_point_type,
                              'suction_pressure': suction_pressure,
                              'ignore_weight': ignore_weight}
        item_meta[item_name].update(j)

        for i in ['grasp_type', 'grasp_point_type', 'suction_pressure']:
          if i not in item_meta[item_name]:
            print('%s is not configured for %s' % (i, item_name))
        if item_meta[item_name]['grasp_type'] not in ['suck', 'grip', 'suck_grip', 'grip_suck']:
          print('%s grasp type is not valid' % item_name)
        if item_meta[item_name]['grasp_point_type'] not in ['normal', 'centroid', 'rgb_centroid']:
          print('%s grasp_point_type is not valid' % item_name)


# Try and load measured weights to overwrite the given weights
try:
  with open(data_path + '/measured_weights.json') as f:
    measured_weights = json.load(f)
    for item_name, weight in measured_weights.items():
      item_meta[item_name]['weight'] = weight
except:
  print('FAILED TO LOAD MEASURED WEIGHTS')

with open(data_path + '/acrv_item_data.json', 'w') as f:
  json.dump(item_meta_update, f, indent=4, separators=(',', ': '))
