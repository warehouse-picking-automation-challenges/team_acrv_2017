#!/usr/bin/env python

'''
camera calibration for distorted images with chess board samples
reads distorted images, calculates the calibration and write undistorted images

usage:
    calibrate.py
'''

# Python 2/3 compatibility
from __future__ import print_function

import numpy as np
import cv2
import sys
import getopt
import glob

# local modules
# from common import splitfn

# built-in modules
import os


def splitfn(fn):
    path, fn = os.path.split(fn)
    name, ext = os.path.splitext(fn)
    return path, name, ext

if __name__ == '__main__':
    out_dir = '../data/output/color/'
    # out_dir = '../data/output/ir/'
    img_names = glob.glob('../data/color/*.png')
    # img_names = glob.glob('../data/ir/*.png')

    square_size = 0.1

    pattern_size = (8, 6)
    pattern_points = np.zeros((np.prod(pattern_size), 3), np.float32)
    pattern_points[:, :2] = np.indices(pattern_size).T.reshape(-1, 2)
    pattern_points *= square_size

    obj_points = []
    img_points = []
    h, w = 0, 0
    img_names_undistort = []

    im_num = 0

    for fn in img_names:
        print('processing %s... ' % fn, end='')
        img = cv2.imread(fn, 0)
        if img is None:
            print("Failed to load", fn)
            continue

        h, w = img.shape[:2]
        found, corners = cv2.findChessboardCorners(img, pattern_size)
        if found:
            term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.1)
            cv2.cornerSubPix(img, corners, (11, 11), (-1, -1), term)
            outfile = os.path.join(out_dir, '%d_distorted.png' % im_num)
            im_num = im_num + 1
            cv2.imwrite(outfile, img)
            img_names_undistort.append(outfile)

        if not found:
            print('chessboard not found')
            continue

        img_points.append(corners.reshape(-1, 2))
        obj_points.append(pattern_points)

        print('ok')

    # calculate camera distortion
    rms, camera_matrix, dist_coefs, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, (w, h), None, None)

    print("\nRMS:", rms)
    print("camera matrix:\n", camera_matrix)
    print("distortion coefficients: ", dist_coefs.ravel())

    # undistort the image with the calibration
    for i, img_found in enumerate(img_names_undistort):
        img = cv2.imread(img_found)

        h,  w = img.shape[:2]
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coefs, (w, h), 0, (w, h))  # Crop the image
        # newcameramtx, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coefs, (w, h), 1, (w, h))  # Keep curved edges

        dst = cv2.undistort(img, camera_matrix, dist_coefs, None, newcameramtx)

        # crop and save the image
        x, y, w, h = roi
        dst = dst[y:y+h, x:x+w]
        outfile = os.path.join(out_dir, '%d_undistorted.png' % i)
        cv2.imwrite(outfile, dst)
    print("new camera matrix:\n", newcameramtx)

    cv2.destroyAllWindows()
