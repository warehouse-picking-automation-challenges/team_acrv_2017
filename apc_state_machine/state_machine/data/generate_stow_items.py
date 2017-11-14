import os
import json
import random

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

seen = []
unseen = []

# Load each item that we have data for.
for d, dirs, files in os.walk('item_data'):
  for f in files:
    if f.endswith('.json'):
      #print 'Loading metadata from %s' % f
      with open(os.path.join(d, f), 'r') as jf:
        j = json.load(jf)
        item_name = j['name']

        if item_name in seen_items:
          seen.append(item_name)
        else:
          unseen.append(item_name)

random.shuffle(seen)
random.shuffle(unseen)
items = seen[:10] + unseen[:10]
items.sort()

print(json.dumps(items, indent=4, separators=(',', ': ')))
