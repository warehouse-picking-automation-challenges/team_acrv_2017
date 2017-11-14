#!/usr/bin/env python
from skimage import measure
from skimage import io

from PIL import ImageFont
from PIL import Image
from PIL import ImageDraw

from skimage.segmentation import find_boundaries, mark_boundaries
from sklearn.metrics.pairwise import pairwise_distances
from scipy import ndimage

import numpy as np
from PIL import Image

import sys
import os
import json

import rospy
import cv_bridge

import cv2

import autoseg_validation as av

from std_msgs.msg import String
from sensor_msgs.msg import Image as sensorImage
from acrv_apc_2017_perception.srv import rgbd_object_proposal, rgbd_object_proposalRequest
from acrv_apc_2017_perception.msg import autosegmenter_msg

sys.path.append(os.path.expanduser('~/ros_ws/src/acrv_apc_2017_perception/src/eval_semseg'))
import items
tr_items = items.getTrainingItems()
all_items = [x.lower() for x in tr_items]

# flist: Path to file to read
# Returns list of files to read images from
# flist should contain lines in the following format
# fname1_rgb.png
# fname2_rgb.png
def read_file_list(flist):
    with open(flist) as fl:
        imfiles = fl.readlines()
    imfiles = [x.strip() for x in imfiles]
    return imfiles



class AutoSegmenter(object):
    def __init__(self):
        self.base_path = os.path.expanduser('~/cloudstor/acrv_apc_2017_data/results_refinenet/20170701_tote_bin')
        self.io_path = '125'
        self.f1shots_path = 'f1shots'
        self.scene = 'tote'
        self.bbox = (21, 0, 602, 350)
        self.img_count = {}
        self.imfile_count = {}

        self.bridge = cv_bridge.CvBridge()

        self.im_pub = rospy.Publisher('/label_image', sensorImage, queue_size=1)

        self.mask_service = rospy.ServiceProxy('/binary_classifier/binary_classification', rgbd_object_proposal)

        self.image_consumer = rospy.Subscriber(rospy.get_param('/autosegmenter_image_topic', '/autosegmenter_image_topic'), autosegmenter_msg, self.autosegment_callback, queue_size=10000)

        self.received_message = False

        self.validator = av.ManualSegmenter()

    def start(self):
        rospy.loginfo('Autosegmenter running')
        rospy.spin()

    def autosegment_callback(self, msg):
        rospy.loginfo('Executing')
        req = rgbd_object_proposalRequest()
        # labels 0 and 1, workaround because the refinenet interface wants labels
        # but the binary model just outputs 0 and 1
        req.expected_labels = msg.content
        rgb = self.bridge.imgmsg_to_cv2(msg.image, desired_encoding='bgr8')

        req.color = msg.image
        res = self.mask_service.call(req)
        self.im_pub.publish(res.classification)
        label = self.bridge.imgmsg_to_cv2(res.classification).copy()
        content = []
        for c in msg.content:
            content.append(c.data)

        if not self.received_message:
            for item in msg.all_items:
                all_items.append(item.data.lower())
            all_items.append('tote')
            all_items.append('storage')
            with open(os.path.expanduser('~/ros_ws/src/acrv_apc_2017_perception/src/refinenet/snapshots/ITEM_LIST_NEW-items.json'), 'w') as f:
                json.dump(all_items, f, indent=4, separators=(',', ': '))
            self.received_message = True
        print(all_items)
        self.execute_with_img(label, rgb, content, self.base_path, self.io_path, msg.image_name.data, self.f1shots_path, self.scene, self.bbox)

    def execute(self, base_path, io_path, content, imfile_path, f1shots_path, scene, bbox, rotate=False):
        raw_output_file = os.path.join(base_path, io_path, 'raw_' + imfile_path)
        lab = io.imread(raw_output_file)

        rgbname = os.path.join(base_path, io_path, imfile_path)
        rgb = io.imread(rgbname)
        if not os.path.exists(os.path.join(base_path, f1shots_path)):
            os.makedirs(os.path.join(base_path, f1shots_path))
        execute_with_img(lab, rgb, content, base_path, io_path, imfile_path, f1shots_path, scene, bbox, rotate)

    def execute_with_img(self, lab, rgb, content, base_path, io_path, imfile_path, f1shots_path, scene, bbox, rotate=False):
        outfile_path = imfile_path
        if not imfile_path.endswith('.png'):
            imfile_path += '.png'
        rgbname = os.path.join(base_path, io_path, imfile_path)
        (img_height, img_width) = lab.shape
        labids = np.unique(lab)
        print(labids)
        # rgb = cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB)

        bbx2 = bbox[0] + bbox[2] - 1
        bby2 = bbox[1] + bbox[3] - 1

        bg_container_id = all_items.index(scene)

        # crop to defined bbox
        rgb = rgb[bbox[1]:bby2, bbox[0]:bbx2, :]
        lab = lab[bbox[1]:bby2, bbox[0]:bbx2]

        # Create binary mask over tote area
        mask_bg_container = np.ones_like(lab)
        mask_bg_container[bbox[1]:bby2, bbox[0]:bbx2] = 0

        # Set areas outside tote area to id 1
        lab[np.where(mask_bg_container == 1)] = labids[1]

        lab = lab.astype(bool)
        lab = np.invert(lab)

        all_labels = measure.label(lab, background=0)

        ccs, counts = np.unique(all_labels, return_counts = True)

        srt_idx = counts.argsort()[::-1]

        n_items = len(content)
        assert n_items in (2,4), 'Number of items should be 2 or 4, but is %d' % (n_items)

        # Expected item positions in the image
        if n_items == 2:
            anchors = np.array([[img_height / 2, img_width    ], [img_height / 2, 0            ]])
        elif n_items == 4:
            anchors = np.array([[         0,         0],
                                [         0, img_width],
                                [img_height, img_width],
                                [img_height,         0]])
        else:
            raise RuntimeError('Number of items should be 2 or 4')

        mask = bg_container_id*np.ones_like(lab, dtype=np.uint8) # fill with known background
        mask[np.where(mask_bg_container==0)] = all_items.index(scene)

        for c in range(1, n_items + 1):
            mask[np.where(all_labels == c)] = c

        segs = np.unique(mask)
        segs = np.setdiff1d(segs, np.array([0, bg_container_id]))
        n_segs = len(segs)
        assert n_segs == n_items, 'Segmentation has failed with %d segmentations for %d items' % (n_segs, n_items)

        # Find centroids of all segments
        coms = []
        for idx, c in enumerate(segs):
            binmask = np.zeros_like(mask)
            binmask[np.where(mask == c)] = 1

            com = ndimage.measurements.center_of_mass(binmask)
            coms.append( (com[1], c) )

        coms.sort(reverse=True)

        newmask = mask.copy()
        for i in range(len(coms)):
            c = coms[i][1]
            newmask[np.where(mask == c)] = all_items.index(content[i])

        newmask = self.validator.execute(rgb, newmask, content, all_items)

        mask = newmask.copy()

        # Display stuff
        mask_ = mask.copy()

        bnd_img = mark_boundaries(rgb, mask_, mode='thick')
        bnd_img = Image.fromarray((bnd_img * 255).astype('uint8'))

        cmap = np.load(os.path.expanduser('~/ros_ws/src/acrv_apc_2017_perception/src/refinenet/SUNcmap.npy'))
        mask_cmap = Image.fromarray(np.uint8(mask))
        mask_raw = mask_cmap.copy()

        mask_cmap.putpalette(cmap)
        # mask_cmap.convert('RGB')

        out_img = mask_cmap.convert('RGB')
        rgb_img = Image.fromarray(cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB))
        out_img = Image.blend(bnd_img, out_img, 0.5)

        for c in np.unique(mask):
            if c < 1 or c > 56:
                continue
            binmask = np.zeros_like(mask)
            binmask[np.where(mask == c)] = 1
            inds = np.nonzero(binmask)
            bounds = np.array([np.min(inds[0]), np.max(inds[0]), np.min(inds[1]), np.max(inds[1])])
            rgb_np = np.array(rgb_img)

            try:
                self.img_count[all_items[c]] += 1
            except:
                self.img_count[all_items[c]] = 1

            seg_msk = binmask[bounds[0]:bounds[1], bounds[2]:bounds[3]]
            seg_rgb = rgb_np[bounds[0]:bounds[1], bounds[2]:bounds[3]]
            seg_msk_img = Image.fromarray(seg_msk)
            seg_rgb_img = Image.fromarray(seg_rgb)
            img_count_str = '%05d' % self.img_count[all_items[c]]
            if not os.path.exists(os.path.join(base_path, 'auto_segments', all_items[c], img_count_str)):
                os.makedirs(os.path.join(base_path, 'auto_segments', all_items[c], img_count_str))
            seg_msk_img.save(os.path.join(base_path, 'auto_segments', all_items[c], img_count_str, 'mask.png'))
            seg_rgb_img.save(os.path.join(base_path, 'auto_segments', all_items[c], img_count_str, 'color.png'))

            com = ndimage.measurements.center_of_mass(binmask)

            fs = int(round(out_img.size[1]/30))
            font = ImageFont.truetype('/usr/share/fonts/dejavu/DejaVuSans.ttf', fs)
            draw = ImageDraw.Draw(out_img)
            draw.text((com[1], com[0]), 'O', (0,0,255), font=font)
            draw.text((com[1], com[0]), all_items[c], (255,255,0), font=font)

            draw = ImageDraw.Draw(out_img)



        try:
            self.imfile_count[imfile_path] += 1
        except:
            self.imfile_count[imfile_path] = 1

        img_count_all = '%05d' % self.imfile_count[imfile_path]

        while os.path.exists(os.path.join(f1shots_path, outfile_path, imfile_path[:-4] + '_' + img_count_all + '.png')):
            self.imfile_count[imfile_path] += 1
            img_count_all = '%05d' % self.imfile_count[imfile_path]

        if not os.path.exists(os.path.join(base_path, f1shots_path, outfile_path)):
            os.makedirs(os.path.join(base_path, f1shots_path, outfile_path))

        mask_f1shots = os.path.join(f1shots_path, outfile_path, 'mask_' + imfile_path[:-4] + '_' + img_count_all + '.png')
        autoseg_f1shots = os.path.join(f1shots_path, outfile_path, 'autoseg_' + imfile_path[:-4] + '_' + img_count_all + '.png')
        rgb_f1shots = os.path.join(f1shots_path, outfile_path, imfile_path[:-4] + '_' + img_count_all + '.png')

        mask_filename = os.path.join(base_path, mask_f1shots)
        autoseg_filename = os.path.join(base_path, autoseg_f1shots)
        rgb_filename = os.path.join(base_path, rgb_f1shots)
        rospy.loginfo('Mask filename: %s' % mask_filename)
        rospy.loginfo('Autosegmented image filename: %s' % autoseg_filename)
        rospy.loginfo('RGB image filename: %s' % rgb_filename)

        mask_raw.save(mask_filename)
        out_img.save(autoseg_filename)
        rgb_img.save(rgb_filename)

        fl_path = os.path.join(base_path, 'f1shots_list.txt')
        with open(fl_path, 'a') as fl:
            fl.write('%s\t%s\n' % (rgb_f1shots, mask_f1shots))

            if rotate:
                randdegs = np.random.rand(10) * 360
                for d, deg in enumerate(randdegs):
                    out_img_rot = out_img.rotate(deg)
                    outfile = imfile_path + 'rot' + str(d) + '.png'

                    mask_rot = mask_raw.rotate(deg)
                    mask_rot.save(os.path.join(base_path, f1shots_path, 'mask_' + outfile))

                    rgb_rot = rgb_img.rotate(deg)
                    rgb_rot.save(os.path.join(base_path, f1shots_path, outfile))

                    out_img_rot.save(os.path.join(base_path, f1shots_path, 'autoseg_' + outfile))
                    fl.write('%s\t%s\n' % (os.path.join(f1shots_path, outfile), os.path.join(f1shots_path, 'mask_' + outfile)))

        # All done

if __name__ == "__main__":
    rospy.init_node('autosegmenter')
    autoseg = AutoSegmenter()
    autoseg.start()

    # p = os.path.expanduser('~/cloudstor/acrv_apc_2017_data/results_refinenet/20170701_tote_bin')
    # flist = read_file_list(os.path.join(p, 'flist.txt'))
    # for f in flist:
    #     execute(p, '125', f, 'f1shots', 'tote', (21, 0, 602, 350))
