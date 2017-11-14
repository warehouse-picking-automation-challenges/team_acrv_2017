#!/usr/bin/env python

'''
camera calibration for distorted images with chess board samples
reads distorted images, calculates the calibration and write undistorted images

usage:
    stereo_calibrate.py
'''

# Python 2/3 compatibility
from __future__ import print_function

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
import yaml

def get_list_of_keys_that_match_value(dictionary, value):
    return [k for k,v in dictionary.iteritems() if v == value]

def splitfn(fn):
    path, fn = os.path.split(fn)
    name, ext = os.path.splitext(fn)
    return path, name, ext

# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

def saveCV2ImageToFile(image_data, file_name, save_dir):
    try:
        if save_dir.endswith('/'):
            save_dir = save_dir[:-1]
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)
        save_path = str(save_dir)+'/'+str(file_name)
        cv2.imwrite(save_path, image_data)
    except CvBridgeError, e:
        print(e)
    return save_path


# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R) :

    assert(isRotationMatrix(R))

    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])

    singular = sy < 1e-6

    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0

    return np.array([z, y, x])

def calibrate_camera_intrinsics(square_size, pattern_size, img_names, out_dir):
    storage = {}

    pattern_points = np.zeros((np.prod(pattern_size), 3), np.float32)
    pattern_points[:, :2] = np.indices(pattern_size).T.reshape(-1, 2)
    pattern_points *= square_size

    obj_points = []
    img_points = []
    h, w = 0, 0
    img_names_undistort = []

    im_num = 0

    for i, fn in enumerate(img_names):
        print('processing %s... ' % fn, end='')
        img = cv2.imread(fn, 0)
        if img is None:
            print("Failed to load", fn)
            continue

        h, w = img.shape[:2]

        img_size = (w, h)

        # For Photoneo + RealSense
        # found, corners = cv2.findChessboardCorners(img, pattern_size, flags=cv2.CALIB_CB_FAST_CHECK)
        # For RealSense
        found, corners = cv2.findChessboardCorners(img, pattern_size)
        if found:
            term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.1)
            cv2.cornerSubPix(img, corners, (11, 11), (-1, -1), term)
            # outfile = os.path.join(out_dir, '%d_distorted.png' % im_num)
            im_num = im_num + 1
            save_path = saveCV2ImageToFile(img, '%d_distorted.png' % im_num, out_dir)
            # cv2.imwrite(outfile, img)
            img_names_undistort.append(save_path)

        if not found:
            print('chessboard not found')
            storage[i] = None
            continue

        img_points.append(corners.reshape(-1, 2))
        obj_points.append(pattern_points)

        storage[i] = (corners.reshape(-1, 2), pattern_points)

        print('ok')

    # calculate camera distortion
    rms, camera_matrix, dist_coefs, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, (w, h), None, None)

    # undistort the image with the calibration
    for i, img_found in enumerate(img_names_undistort):
        img = cv2.imread(img_found)

        h,  w = img.shape[:2]
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coefs, (w, h), 0, (w, h))  # Crop the image

        dst = cv2.undistort(img, camera_matrix, dist_coefs, None, newcameramtx)

        # crop and save the image
        x, y, w, h = roi
        dst = dst[y:y+h, x:x+w]
        # outfile = os.path.join(out_dir, '%d_undistorted.png' % i)
        # cv2.imwrite(outfile, dst)
        saveCV2ImageToFile(dst, '%d_undistorted.png' % i, out_dir)

    return camera_matrix, dist_coefs, newcameramtx, img_size, storage

if __name__ == '__main__':
    color_out_dir = '../data/output/color/'
    ir_out_dir = '../data/output/ir/'
    color_img_names = sorted(glob.glob('../data/color/*.png'))
    ir_img_names = sorted(glob.glob('../data/ir/*.png'))

    if len(color_img_names) != len(ir_img_names):
        print("There are an uneven number of color to IR images")
        print("%d color images" % len(color_img_names))
        print("%d IR images" % len(ir_img_names))
        sys.exit(1)

    square_size = 0.035
    pattern_size = (8, 6)

    color_camera_matrix, color_dist_coefs, color_newcameramtx, color_img_size, color_storage = calibrate_camera_intrinsics(square_size, pattern_size, color_img_names, color_out_dir)
    ir_camera_matrix, ir_dist_coefs, ir_newcameramtx, ir_img_size, ir_storage = calibrate_camera_intrinsics(square_size, pattern_size, ir_img_names, ir_out_dir)

    ir_keys_to_delete = get_list_of_keys_that_match_value(color_storage, None)
    color_keys_to_delete = get_list_of_keys_that_match_value(ir_storage, None)

    for key in ir_keys_to_delete:
        ir_storage[key] = None

    for key in color_keys_to_delete:
        color_storage[key] = None

    good_color_obj_points = []
    good_color_img_points = []
    good_ir_obj_points = []
    good_ir_img_points = []

    for key, val in color_storage.iteritems():
        if val is not None:
            good_color_obj_points.append(val[1])
            good_color_img_points.append(val[0])

    for key, val in ir_storage.iteritems():
        if val is not None:
            good_ir_obj_points.append(val[1])
            good_ir_img_points.append(val[0])

    print('Calibrating extrinsics on %d images...' % len(good_color_obj_points))

    rotation_matrix = None
    transform = None
    criteria = (cv2.TERM_CRITERIA_MAX_ITER + cv2.TERM_CRITERIA_EPS, 100, 1e-5)
    flags = (cv2.CALIB_FIX_INTRINSIC)

    _, cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, rotation_matrix, transform, E, F = cv2.stereoCalibrate(objectPoints=good_color_obj_points, imagePoints1=good_color_img_points, imagePoints2=good_ir_img_points, imageSize=color_img_size, cameraMatrix1=color_camera_matrix, distCoeffs1=color_dist_coefs, cameraMatrix2=ir_camera_matrix, distCoeffs2=ir_dist_coefs, R=rotation_matrix, T=transform, E=None, F=None, criteria=criteria, flags=flags)

    # Add the extra column of zeros into the projection matrix as required by ros things

    color_newcameramtx_ros = np.zeros((color_newcameramtx.shape[0], color_newcameramtx.shape[1] + 1))
    ir_newcameramtx_ros = np.zeros((ir_newcameramtx.shape[0], ir_newcameramtx.shape[1] + 1))
    color_newcameramtx_ros[:,:-1] = color_newcameramtx
    ir_newcameramtx_ros[:,:-1] = ir_newcameramtx

    print("color camera matrix:\n", color_camera_matrix)  # K in ros camera info -> http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html
    print("color new camera matrix:\n", color_newcameramtx_ros)  # P in ros camera info
    print("color distortion coefficients: ", color_dist_coefs.ravel())  # D in ros camera info
    print("ir camera matrix:\n", ir_camera_matrix)
    print("ir new camera matrix:\n", ir_newcameramtx_ros)
    print("ir distortion coefficients:\n", ir_dist_coefs.ravel())
    print("rotation matrix:\n", rotation_matrix)  # For the camera frame transforms
    print("transform:\n", transform)  # For the camera frame transforms
    print("rotation matrix:\n", rotationMatrixToEulerAngles(rotation_matrix))  # For the camera frame transforms

    d = {}
    d['image_width'] = color_img_size[0]
    d['image_height'] = color_img_size[1]
    d['camera_name'] = 'INSERT_CAMERA_NAME'
    d['camera_matrix'] = {'rows' : 3, 'cols' : 3, 'data' : (color_camera_matrix.flatten()).tolist()}
    d['distortion_model'] = 'plumb_bob'
    d['distortion_coefficients'] = {'rows' : 1, 'cols' : 5, 'data' : (color_dist_coefs.flatten()).tolist()}
    d['rectification_matrix'] = {'rows' : 3, 'cols' : 3, 'data' : [1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0]}
    d['projection_matrix'] = {'rows' : 3, 'cols' : 4, 'data' : (color_newcameramtx_ros.flatten()).tolist()}
    with open('rgb_hd_camera_info.yaml', 'w') as f:
        yaml.dump(d, f, width=1000)

    d = {}
    d['image_width'] = ir_img_size[0]
    d['image_height'] = ir_img_size[1]
    d['camera_name'] = 'INSERT_CAMERA_NAME'
    d['camera_matrix'] = {'rows' : 3, 'cols' : 3, 'data' : (ir_camera_matrix.flatten()).tolist()}
    d['distortion_model'] = 'plumb_bob'
    d['distortion_coefficients'] = {'rows' : 1, 'cols' : 5, 'data' : (ir_dist_coefs.flatten()).tolist()}
    d['rectification_matrix'] = {'rows' : 3, 'cols' : 3, 'data' : [1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0]}
    d['projection_matrix'] = {'rows' : 3, 'cols' : 4, 'data' : (ir_newcameramtx_ros.flatten()).tolist()}
    with open('depth_camera_info.yaml', 'w') as f:
        yaml.dump(d, f, width=1000)
    cv2.destroyAllWindows()
