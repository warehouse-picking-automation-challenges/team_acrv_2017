import rospy
import datetime
import os
import json

from std_msgs.msg import String
from sensor_msgs.msg import Image
from acrv_apc_2017_perception.msg import autosegmenter_msg

import cv_bridge
import cv2

NUM_IMGS = 7

seen_items = [
    "plastic_wine_glass",
    "hinged_ruled_index_cards",
    "black_fashion_gloves",
    "fiskars_scissors",
    "colgate_toothbrush_4pk",
    "ticonderoga_pencils",
    "tennis_ball_container",
    "expo_eraser",
    "balloons",
    "flashlight",
    "white_facecloth",
    "scotch_sponges",
    "robots_everywhere",
    "speed_stick",
    "marbles",
    "windex",
    "duct_tape",
    "bath_sponge",
    "epsom_salts",
    "burts_bees_baby_wipes",
    "toilet_brush",
    "ice_cube_tray",
    "robots_dvd",
    "table_cloth",
    "irish_spring_soap",
    "avery_binder",
    "hanes_socks",
    "glue_sticks",
    "reynolds_wrap",
    "mouse_traps",
    "measuring_spoons",
    "tissue_box",
    "pie_plates",
    "band_aid_tape",
    "hand_weight",
    "poland_spring_water",
    "mesh_cup",
    "crayons",
    "laugh_out_loud_jokes",
    "composition_book"
]

rospy.init_node('image_capture')

cb = cv_bridge.CvBridge()

pub = rospy.Publisher(rospy.get_param('/autosegmenter_image_topic', '/autosegmenter_image_topic'), autosegmenter_msg, queue_size=1000)

start = datetime.datetime.now()

dt = datetime.datetime.now().strftime('%y%m%d_%H%M%S')
os.makedirs('captured_images/%s'%dt)

objects_list_ros = []
objects_pairs_list = []
obj_pair = []

# Load the objects from the unseen_items folder
for d, dirs, files in os.walk('item_data/'):
  for f in files:
    if f.endswith('.json'):
      with open(os.path.join(d, f), 'r') as jf:
        print(jf)
        j = json.load(jf)
        item_name = j['name']

        if item_name in seen_items:
          print('ignored seen item')
          continue

        obj_pair.append(item_name)
        objects_list_ros.append(String(item_name))
    if len(obj_pair) == 2:
      objects_pairs_list.append(obj_pair)
      obj_pair = []
if len(obj_pair) == 1:
  objects_pairs_list.append(obj_pair)

print('\nITS IMPORTANT THAT YOU RUN THIS SCRIPT IN THE DATA FOLDER\n')

# Take pictures and pass them to the auto segmenter service.
for op in objects_pairs_list:
  print('Put %s into the tote. Enter to Continue' % op)

  for i in range(NUM_IMGS):
    raw_input()
    img = rospy.wait_for_message('/realsense_wrist/rgb/image_rect', Image)
    msg = autosegmenter_msg()
    msg.image = img
    msg.image_name.data = '_'.join(op)
    op_ros = [String(o) for o in op]
    msg.content = op_ros
    msg.all_items = objects_list_ros
    pub.publish(msg)

    rgb_image = cb.imgmsg_to_cv2(img, 'bgr8')
    cv2.imwrite('captured_images/%s/%s_%s.png' % (dt, '_'.join(op), i), rgb_image)

    print('Done %s/%s.  Move the objects. Enter to continue' % (i+1, NUM_IMGS))

end = datetime.datetime.now()
print('FINISHED.')
print('Took %s' % (end-start))
print('Leave this node running until all of the images have been processed.')

while True:
    rospy.sleep(0.5)
