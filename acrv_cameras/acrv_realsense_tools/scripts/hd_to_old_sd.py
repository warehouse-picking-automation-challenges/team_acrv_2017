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

im_paths = sorted(glob.glob('../data/color/*.png'))
out_dir = '../data/color_old_sd/'

# cv::Mat crop_img = color_hd_image(cv::Rect(240,0,1440,1080));
#             cv::Mat color_image;
#             cv::resize(crop_img, color_image, cv::Size(640,480), cv::INTER_CUBIC);
#
#             // Crop 240 a side from the rgb image
#             // Downscale by 2.25 to get 640x480
#             // 1440/2.25 = 640
#             // 1080/2.25 = 480

for im_path in im_paths:
      print('processing %s... ' % im_path)
      im_name = os.path.basename(im_path)
      print(im_name)
      img = cv2.imread(im_path)
      if img is None:
          print("Failed to load", im_path)
          continue
      img = img[:,240:1680]
      img = cv2.resize(img, (640,480))
      saveCV2ImageToFile(img, im_name, out_dir)
