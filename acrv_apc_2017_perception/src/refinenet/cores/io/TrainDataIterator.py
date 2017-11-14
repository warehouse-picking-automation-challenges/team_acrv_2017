import numpy as np
from multiprocessing import Process, Queue
from mxnet.io import DataIter, DataBatch
import mxnet as mx

import numpy as np
from mxnet.io import DataIter
from PIL import Image
import os
import preprocessing
import logging

import sys

#rgb_mean=(140.5192, 59.6655, 63.8419), #mean on tote trainval

class TrainDataIterator(DataIter):
    def __init__(self,
                 root_dir,
                 flist_path,
                 rgb_mean=(128,128,128),
                 random_flip=True,
                 random_scale=False,
                 random_rotate=True,
                 scale_range=(0.8, 1.2),
                 crop_size=400,
                 random_crop=True,
                 epoch_size=True,
                 label_shrink_scale=1.0,
                 shuffle=True,
                 data_queue_size=100,
                 batch_size=1,
                 data_worker_num=1):

        self.rgb_mean = np.array(rgb_mean, dtype=np.uint8).reshape((1,1,3))
        self.random_flip = random_flip
        self.random_scale = random_scale
        self.random_rotate = random_rotate
        self.scale_range = scale_range
        assert scale_range[1]>=scale_range[0]>0
        self.crop_size = crop_size
        self.label_shrink_scale = label_shrink_scale
        self.random_crop = random_crop
        self.epoch_size = epoch_size
        self.data_count = 0
        self.shuffle = shuffle
        self.batch_size = batch_size
        self.flist = None
        self.root_dir = root_dir
        self._load_flist(flist_path)
        self.data_num = self.get_data_num()
        self.avail_data_num = self.data_num
        self.cursor = 0
        self.reset_list()

        self.flist_item_queue = Queue(maxsize=1000)
        self.list_producer = Process(target=self._produce_flist_item)
        self.list_producer.daemon = True
        self.list_producer.start()

        self.data_queue = Queue(maxsize=data_queue_size)
        for i in range(data_worker_num):
            producer = Process(target=self._produce_data)
            producer.daemon = True
            producer.start()

    def _produce_flist_item(self):
        while True:
            if self.cursor + 1 <= self.data_num:
                file = self.flist[self.cursor]
                self.flist_item_queue.put(file)
                self.cursor += 1
            else:
                self.reset_list()

    def _produce_data(self):
        while True:
            flist_item = self.flist_item_queue.get()
            value = self._process_data(flist_item)
            if value is not None:
                self.data_queue.put(value)

    def get_data(self):
        images = []
        labels = []
        for i in range(self.batch_size):
            data = self.data_queue.get()
            images.append(data[0])
            labels.append(data[1])
        images = np.concatenate(images)
        labels = np.concatenate(labels)
        return (mx.nd.array(images), mx.nd.array(labels))

    def get_data_num(self):
        return len(self.flist)

    def _load_flist(self,
                   flist_path):
        with open(flist_path) as f:
            lines = f.readlines()
            self.flist = []
            for line in lines:
                if len(line.rstrip()) == 0:
                    continue
                item = self._parse_flist_item(line.rstrip())
                self.flist.append(item)
            self.data_num = len(self.flist)

    def reset_list(self):
        self.cursor = 0
        if self.shuffle:
            np.random.shuffle(self.flist)
    
    def _process_data(self, item):
        try:
            im = Image.open(os.path.join(self.root_dir, item[0]))
            im = im.convert("RGB")
            l = Image.open(os.path.join(self.root_dir, item[1]))
        except Exception as e:
            logging.info(e)
            return None
          
        if self.random_rotate:
          deg = np.random.rand(1) * 360
          im=im.rotate(deg, resample=Image.BICUBIC, expand=True)
          l=l.rotate(deg, resample=Image.NEAREST, expand=True)
        
        im_arr = np.array(im)
        l_arr = np.array(l)
        r_start, c_start, new_crop_size = preprocessing.calc_crop_params(im_arr, self.scale_range, self.crop_size)


        #random flip
        if self.random_flip:
            im_arr, l_arr = preprocessing.random_flip(im_arr, l_arr)
        im_arr, l_arr = preprocessing.pad_image(im_arr, l_arr, new_crop_size, self.rgb_mean)

        #do crop
        if self.random_crop:
          im_arr = im_arr[r_start:r_start+new_crop_size, c_start:c_start+new_crop_size, :]
          l_arr = l_arr[r_start:r_start+new_crop_size, c_start:c_start+new_crop_size]

        #do resize
      
        im_arr = Image.fromarray(im_arr).resize((self.crop_size, self.crop_size), Image.BICUBIC)
        im_arr = np.array(im_arr, dtype=np.float32)
        im_arr -= self.rgb_mean


        l_dim = int(self.crop_size*self.label_shrink_scale)
        l_arr = Image.fromarray(l_arr).resize((l_dim, l_dim), Image.NEAREST)
        l_arr = np.array(l_arr, dtype=np.uint8)


        im_arr = np.expand_dims(im_arr, 0)
        im_arr = np.transpose(im_arr, [0, 3, 1, 2])
        l_arr = l_arr.reshape(1, -1)

        return (im_arr, l_arr)

    def _parse_flist_item(self, line):
        items = line.split("\t")
        assert len(items) == 2
        im = items[0]
        l = items[1]
        return (im, l)

    @property
    def provide_data(self):
        return [("data", (self.batch_size, 3, self.crop_size, self.crop_size))]

    @property
    def provide_label(self):
        label_dim = int(self.crop_size*self.label_shrink_scale)
        return [("softmax_label", (self.batch_size, label_dim*label_dim))]
    
    def reset(self):
        self.data_count = 0
        pass

    def iter_next(self):
        self.data_count += self.batch_size
        return self.data_count <= self.epoch_size*self.batch_size

    def next(self):
        if self.iter_next():
            data = self.get_data()
            return DataBatch(data=[data[0]], label=[data[1]], pad=None, index=None)
        else:
            raise StopIteration
