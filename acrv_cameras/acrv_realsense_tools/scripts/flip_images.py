#!/usr/bin/env python

import numpy as np
import cv2
import sys
import getopt
import glob
import math

# local modules
# from common import splitfn

# built-in modules
import os
import sys
import math

def saveCV2ImageToFile(image_data, file_name, save_dir):
    try:
        if save_dir.endswith('/'):
            save_dir = save_dir[:-1]
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)
        cv2.imwrite(str(save_dir)+'/'+str(file_name), image_data)
    except CvBridgeError, e:
        print e

out_dir = '../data/output/color/'
im_paths = sorted(glob.glob('../data/color/*.png'))

for im_path in im_paths:
      print('processing %s... ' % im_path)
      im_name = os.path.basename(im_path)
      print(im_name)
      img = cv2.imread(im_path)
      if img is None:
          print("Failed to load", im_path)
          continue
      img = cv2.flip(img, 0)
      img = cv2.flip(img, 1)
      saveCV2ImageToFile(img, im_name, out_dir)
