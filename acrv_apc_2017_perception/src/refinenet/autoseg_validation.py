#!/usr/bin/env python

import cv2
import numpy as np

import time

import os
import sys
sys.path.append(os.path.expanduser('~/ros_ws/src/acrv_apc_2017_perception/src/eval_semseg'))
import items

class ManualSegmenter:
    def __init__(self):
        self.points = []
        self.image_name = 'image'
        self.mask_name = 'mask'

    def segment_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.points.append((x,y))

    def dummy_callback(self, even, x, y, flags, param):
        pass

    def execute(self, orig_image, orig_mask, possible_items, all_items):

        # cv2.namedWindow(self.image_name, cv2.WINDOW_NORMAL)
        # cv2.resizeWindow(self.image_name, 1280, 720)

        output_mask = orig_mask.copy()

        idx = 0

        item_key = possible_items[idx]
        idx = (idx + 1) % len(possible_items)
        item_value = all_items.index(item_key)
        mask = orig_mask == item_value
        base_image = orig_image.copy()
        base_image[np.where(mask)] = (base_image[np.where(mask)] * 0.5).astype(np.uint8)

        window_name = 'Is the mask of %s okay?' % item_key
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 1280, 720)

        items_okay = len(possible_items)
        while items_okay > 0:
            new_mask = np.zeros((orig_image.shape[0], orig_image.shape[1]), dtype=np.uint8)

            contours = np.array(self.points, dtype=np.int32)
            cv2.drawContours(new_mask, [contours], 0, 255 - item_value, -1)
            new_mask = (new_mask != 0).astype(np.bool)
            image = base_image.copy()
            image[np.where(new_mask)] = (image[np.where(new_mask)] * 0.5).astype(np.uint8)


            cv2.imshow(window_name, image)
            # cv2.imshow('debug', orig_mask)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('u'):
                if self.points:
                    self.points.pop()
            elif key == ord('y'):
                cv2.destroyWindow(window_name)
                orig_mask[np.where(new_mask != 0)] = item_value
                self.points = []

                item_key = possible_items[idx]
                idx = (idx + 1) % len(possible_items)
                item_value = all_items.index(item_key)
                mask = orig_mask == item_value
                base_image = orig_image.copy()
                base_image[np.where(mask)] = (base_image[np.where(mask)] * 0.5).astype(np.uint8)

                items_okay -= 1
                if not items_okay:
                    break
                window_name = 'Is the mask of %s okay?' % item_key
                cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(window_name, 1280, 720)
            elif key == ord('n'):
                cv2.destroyWindow(window_name)
                base_image = orig_image.copy()
                orig_mask[np.where(orig_mask == item_value)] = all_items.index('tote')
                window_name = 'Resegment %s manually, press \'y\' when done and \'u\' to undo' % item_key
                cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(window_name, 1280, 720)
                cv2.setMouseCallback(window_name, self.segment_callback)
            elif key == ord('q'):
                break
            time.sleep(0.1)
        return orig_mask

if __name__ == "__main__":
    im = cv2.imread('image.png')
    mask = cv2.imread('image_mask.png', cv2.IMREAD_GRAYSCALE)
    seg = ManualSegmenter()

    items_list = items.getAllItems()
    seg.execute(im, mask, ['unknown', 'acrv_cap', 'wall_clock', 'tote'], items_list)
