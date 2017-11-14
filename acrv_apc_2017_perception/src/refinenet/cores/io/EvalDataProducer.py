import numpy as np
from multiprocessing import Process, Queue
import preprocessing
from PIL import Image
import os
import mxnet as mx

class EvalDataProducer(object):
    def __init__(self,
                 root_dir,
                 flist_path,
                 rgb_mean=(128, 128, 128),
                 data_queue_size=100,
                 data_worker_num=1):
        self.flist = None
        self.root_dir = root_dir
        self._load_flist(flist_path)
        self.data_num = self.get_data_num()
        self.avail_data_num = self.data_num
        self.cursor = 0
        self.rgb_mean = np.array(rgb_mean).reshape((1,1,3))

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
                return

    def _produce_data(self):
        while True:
            flist_item = self.flist_item_queue.get()
            value = self._process_data(flist_item)
            if value is not None:
                self.data_queue.put(value)
            else:
                raise AssertionError("file error: %s"%flist_item[0])

    def _process_data(self, item):
        rgb_name = item[0]
        label_name = item[1]
        im = Image.open(os.path.join(self.root_dir, item[0]))
        im = im.convert("RGB")        
        w, h = im.size
        
        im_arr = np.array(im, dtype=np.float32)
        im_arr -= self.rgb_mean
        
        label_file_name = os.path.join(self.root_dir, item[1])
        if os.path.exists(label_file_name):
          l = Image.open(label_file_name)
          l_arr = np.array(l, dtype=np.uint8)
        else:
          print("WARNING: GT File for %s not found" % label_file_name)
          l_arr = np.zeros((h,w),dtype=np.uint8)
          

        im_arr = np.expand_dims(im_arr, 0)
        im_arr = np.transpose(im_arr, [0, 3, 1, 2])

        return (im_arr, l_arr, label_name[label_name.rfind('/')+1:], rgb_name[rgb_name.rfind('/')+1:], im, label_name)

    def get_data(self):
        if self.avail_data_num>0:
            self.avail_data_num -= 1
            data = self.data_queue.get()
            return data
        else:
            return None

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



    def _parse_flist_item(self, line):
        items = line.split("\t")
        assert len(items)>=1 and len(items) <= 2
        
        im = items[0]

        if len(items)==1:
          l = 'UNAVAILABLE'
          label_name = 'UNAVAILABLE'
          print("WARNING: GT for %s not available!" % im)
        else:          
          l = items[1]
          label_name = items[1]
          
        return (im, l, label_name[label_name.rfind('/')+1:])




