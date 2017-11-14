#!/usr/bin/env python

import os
import sys
import inspect
import math
import datetime

from PIL import ImageFont
from PIL import Image
from PIL import ImageDraw
from skimage.segmentation import find_boundaries, mark_boundaries
from skimage import measure
from scipy import ndimage

currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0, parentdir)
os.environ["MXNET_CUDNN_AUTOTUNE_DEFAULT"] = "0"

import numpy as np

import rospy
import mxnet as mx
import src.refinenet.cores.symbols.refineNet101 as net
from src.refinenet.cores.utils import misc
from src.refinenet.cores.io.EvalDataProducer import EvalDataProducer

from cv_bridge import CvBridge, CvBridgeError
import cv2

from acrv_apc_2017_perception.srv import rgbd_object_proposal, rgbd_object_proposalResponse
from std_msgs.msg import String, UInt8
from sensor_msgs.msg import Image as sImage

from src.eval_semseg import items

import json
from collections import namedtuple

class RefineNet(object):
    def __init__(self):
        self.cv_bridge = CvBridge()

        self.debug_pub = rospy.Publisher('/rfnet_conf_map', sImage, queue_size=5)

        self.cmap = np.load("/home/apc/ros_ws/src/acrv_apc_2017_perception/src/refinenet/SUNcmap.npy")

        self.tote_classification = rospy.Service('~refinenet_classification', rgbd_object_proposal, self.classifier_callback)
        # self.curtain_classification = rospy.Service('~curtain_classification', rgbd_object_proposal, self.classifier_callback)
        self.binary_classification = rospy.Service('~binary_classification', rgbd_object_proposal, self.binary_callback)
        self.ctx = [mx.gpu(int(0))]

        fs = 14
        self.font = ImageFont.truetype("/usr/share/fonts/dejavu/DejaVuSans.ttf", fs)

        self.epoch = EPOCH

        with open(os.path.join(SNAPSHOT_FOLDER, '%s-items.json' % (MODEL_NAME)), 'r') as f:
            self.item_list = json.load(f)
            self.item_list = [x.lower() for x in self.item_list]

        seg_net_prefix = os.path.join(SNAPSHOT_FOLDER, MODEL_NAME)
        self.arg_dict, self.aux_dict, _ = misc.load_checkpoint(seg_net_prefix, self.epoch, load_symbol=False)

        # self.debug_pub = rospy.Publisher('/rfnet_conf_map', sImage, queue_size = 5)

        # seg_net = net.create_infer(CLASSNUM, WORKSPACE)
        # self.mod = mx.module.Module(seg_net, data_names=('data',), label_names=(), context=self.ctx)
        # self.mod.bind(data_shapes=[("data", (1, 3, 640, 640))], for_training=False, grad_req='null')
        # self.mod.init_params(arg_params=self.arg_dict, aux_params=self.aux_dict, allow_missing=True)
        # self.Batch = namedtuple('Batch', ['data'])

    def classifier_callback(self, req):
        return self.classifier(req, True)

    def binary_callback(self, req):
        return self.classifier(req, False)

    def classifier(self, req, zero_out):
        expected_items = []
        for i in req.expected_labels:
            expected_items.append(i.data)

        try:
            hd_image = self.cv_bridge.imgmsg_to_cv2(req.color, desired_encoding="passthrough")
        except CvBridgeError as e:
            rospy.logerr(e)

        os.environ["MXNET_CUDNN_AUTOTUNE_DEFAULT"] = "0"

        # t0 = datetime.datetime.now()
        seg_net = net.create_infer(CLASSNUM, WORKSPACE)
        mod = mx.module.Module(seg_net, data_names=('data',), label_names=(), context=self.ctx)
        # t1 = datetime.datetime.now()
        # rospy.logerr('seg_net creation took %s' % (t1 - t0))

        # t0 = datetime.datetime.now()
        mod.bind(data_shapes=[("data", (1, 3, 640, 640))], for_training=False, grad_req='null')
        # t1 = datetime.datetime.now()
        # rospy.logerr('module bind took %s' % (t1 - t0))

        # t0 = datetime.datetime.now()
        mod.init_params(arg_params=self.arg_dict, aux_params=self.aux_dict, allow_missing=True)
        # t1 = datetime.datetime.now()
        # rospy.logerr('module parameter init took %s' % (t1 - t0))

        rgb_mean = 128
        hd_image = hd_image.astype(np.float32)
        im = hd_image - rgb_mean
        im = np.swapaxes(im, 0, 2)
        im = np.swapaxes(im, 1, 2)
        im = np.expand_dims(im, axis=0)
        im, orig_size = misc.pad_image(im, 32)

        # t0 = datetime.datetime.now()
        mod.reshape([("data", im.shape)])
        # t1 = datetime.datetime.now()
        # rospy.logerr('module reshape took %s' % (t1 - t0))

        # t0 = datetime.datetime.now()
        mod.bind(data_shapes=[("data", (1, 3, 640, 640))], for_training=False, grad_req='null')
        # t1 = datetime.datetime.now()
        # rospy.logerr('module bind took %s' % (t1 - t0))

        # t0 = datetime.datetime.now()
        mod.forward(mx.io.DataBatch(data=[mx.nd.array(im)], label=None, pad=None, index=None))
        # t1 = datetime.datetime.now()
        # rospy.logerr('module forward took %s' % (t1 - t0))

        # t0 = datetime.datetime.now()
        pred = mod.get_outputs()[0].asnumpy().squeeze()
        # t1 = datetime.datetime.now()
        # rospy.logerr('module prediction took %s' % (t1 - t0))

        pred = misc.crop_pred(pred, orig_size)

        # t0 = datetime.datetime.now()
        if zero_out:
            for i in self.item_list[1:]:
                j = self.item_list.index(i)
                if i in expected_items or j < 1 or j >= CLASSNUM:
                    continue
                if j < pred.shape[0]:
                    pred[j] = np.zeros_like(pred[j])
                    continue

        pred_label = pred.argmax(0)

        # t1 = datetime.datetime.now()
        # rospy.logerr('bad prediction elimination took %s' % (t1 - t0))

        # s0 = datetime.datetime.now()
        best_scores_map, second_best_scores_map, diff_conf_map = misc.confidence_maps(pred)
        # s1 = datetime.datetime.now()
        # rospy.logerr('confidence map creation took %s' % (s1 - s0))

        mean_conf_map = np.zeros_like(diff_conf_map)
        ccs, num_ccs = measure.label(pred_label, return_num=True)
        for cc in range(1, num_ccs + 1):
            mean_conf = diff_conf_map[np.where(ccs==cc)].mean()
            mean_conf_map[np.where(ccs == cc)] = mean_conf
        mean_conf_map = (mean_conf_map * 255).astype(np.uint8)
        mean_conf_map[mean_conf_map == 0] = 1

        # t0 = datetime.datetime.now()
        binmask = np.zeros_like(pred_label)
        segment_certainties = []
        empty_img = np.zeros_like(mean_conf_map)
        conf_map = np.zeros_like(mean_conf_map)
        pred_label_unique = np.unique(pred_label)
        if zero_out:
            for c in pred_label_unique:
                # -3 for storage, tote, and because zero indexing
                if c < 1 or c > (CLASSNUM-3):
                    continue

                binmask = pred_label == c
                cnts = cv2.findContours(binmask.astype(np.uint8), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[1]
                if len(cnts) != 0:
                    max_cnt = -1
                    max_certainty = 0
                    cnts = sorted(cnts, key = cv2.contourArea, reverse=True)
                    empty_img = empty_img * 0
                    for cc in range(len(cnts)):
                        col = cc + 1
                        cv2.drawContours(image=empty_img, contours=cnts, contourIdx=cc, color=col, thickness=-1)
                        certainty = (mean_conf_map * (empty_img == col)).max()
                        if certainty > max_certainty:
                            max_certainty = certainty
                            max_cnt = col
                    segment_certainties.append(max_certainty)
                    mask = empty_img == max_cnt
                    conf_map[np.where(mask)] = max_certainty
            pred_label[np.where(conf_map == 0)] = 0
        pred_label_unique = np.unique(pred_label)
        labels = []
        for i in self.item_list[1:]:
            j = self.item_list.index(i)
            if i in expected_items and j in pred_label_unique:
                labels.append(i)

        # t1 = datetime.datetime.now()
        # rospy.logerr('Took %s' % (t1 - t0))
        out_lab = np.uint8(np.copy(pred_label))

        out_lab = Image.fromarray(out_lab)
        out_lab_raw = out_lab.copy()

        out_lab.putpalette(self.cmap)
        out_lab = out_lab.convert('RGB')
        rgb_img = hd_image

        rgb_img_arr = np.array(rgb_img, dtype=np.float32) / 255

        bnd_img = mark_boundaries(rgb_img_arr, pred_label, mode='thick')

        bnd_img = Image.fromarray((bnd_img*255).astype('uint8'));
        out_img = Image.blend(bnd_img, out_lab, 0.6)
        # t1 = datetime.datetime.now()
        # rospy.logerr('output image creation took %s' % (t1 - t0))

        # Stay at 1 because state machine code uses 1 indexing because historical reasons
        k = 1
        for c in pred_label_unique:
            # -3 for storage, tote, and because zero indexing
            if c < 1 or c > (CLASSNUM-3):
                continue
            binmask = pred_label == c
            pred_label[np.where(pred_label==c)] = k
            k += 1
            com = ndimage.measurements.center_of_mass(binmask)

            fs = 14
            draw = ImageDraw.Draw(out_img)

            draw.text((com[1], com[0]), 'O', (0,0,255), font=self.font)
            draw.text((com[1]-len(self.item_list[c])/2*fs/2, com[0]), self.item_list[c], (255,255,0), font=self.font)

            draw = ImageDraw.Draw(out_img)

        # t1 = datetime.datetime.now()
        # rospy.logerr('output image creation overall took %s' % (t1 - t0))

        # t0 = datetime.datetime.now()
        res = rgbd_object_proposalResponse()
        try:
            res.classification = self.cv_bridge.cv2_to_imgmsg(pred_label.astype(np.uint8), 'mono8')
        except CvBridgeError as e:
            rospy.logerr(e)

        try:
            oi = np.array(out_img)
            oi = cv2.cvtColor(oi, cv2.COLOR_BGR2RGB)
            res.overlay_img = self.cv_bridge.cv2_to_imgmsg(oi, 'rgb8')
        except CvBridgeError as e:
            rospy.logerr(e)

        try:
            res.confidence_map = self.cv_bridge.cv2_to_imgmsg(mean_conf_map, 'mono8')
        except CvBridgeError as e:
            rospy.logerr(e)

        for i in labels:
            res.label.append(String(i))
        for i in segment_certainties:
            res.segment_certainties.append(UInt8(i))
        # t1 = datetime.datetime.now()
        # rospy.logerr('response creation took %s' % (t1 - t0))

        return res

    def execute(self):
        rospy.loginfo('RefineNet server ready')
        rospy.spin()

if __name__ == "__main__":
    rospy.init_node('refinenet_classifier')
    EPOCH = int(rospy.get_param('~epoch', '0'))
    CLASSNUM = int(rospy.get_param('~classnum'))
    WORKSPACE = int(rospy.get_param('~workspace'))
    MODEL_NAME = str(rospy.get_param('~model_name'))
    SNAPSHOT_FOLDER = str(rospy.get_param('~snapshot_folder'))
    print('Epoch = ' + str(EPOCH))
    print('Classnum = ' + str(CLASSNUM))
    print('Workspace = ' + str(WORKSPACE))
    print('Model name = ' + str(MODEL_NAME))
    print('Snapshot folder = ' + str(SNAPSHOT_FOLDER))
    rfnet = RefineNet()
    rfnet.execute()
