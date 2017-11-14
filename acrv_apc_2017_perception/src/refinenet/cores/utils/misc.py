import mxnet.metric
import numpy as np
import mxnet.callback
import time
import logging
import os
import mxnet as mx
from skimage import measure
from scipy import ndimage

def create_executor(net, arg_dict, aux_dict, grad_req, shape_dict, ctx):
    e = net.simple_bind(ctx=ctx,
                        grad_req=grad_req,
                        **shape_dict)
    for arg in e.arg_dict:
        if "data" in arg or "label" in arg:
                continue
        if "upsampling" in arg:
            shape = e.arg_dict[arg].shape
            weight = np.zeros(np.prod(shape), dtype='float32')
            f = np.ceil(shape[3] / 2.)
            c = (2 * f - 1 - f % 2) / (2. * f)
            for i in range(np.prod(shape)):
                x = i % shape[3]
                y = (i / shape[3]) % shape[2]
                weight[i] = (1 - abs(x / f - c)) * (1 - abs(y / f - c))
            e.arg_dict[arg][:] = weight.reshape(shape)
            continue
        if arg not in arg_dict.keys():
            raise AssertionError("Argument %s is not initialized"%arg)
        else:
            e.arg_dict[arg][:] = arg_dict[arg]
    for aux in e.aux_dict:
        if aux not in aux_dict.keys():
            raise AssertionError("Argument %s is not initialized"%aux)
        else:
            e.aux_dict[aux][:] = aux_dict[aux]
    return e

def module_checkpoint(prefix):
    def _callback(epoch_num, sym=None, arg=None, aux=None):
    #   if not epoch_num % 5 or epoch_num == 1:
        save_checkpoint(prefix, epoch_num, symbol=sym, arg_params=arg, aux_params=aux)
    return _callback

def load_checkpoint(prefix, epoch, load_symbol=True):
    symbol=None
    if load_symbol:
        symbol = mx.sym.load('%s-symbol.json' % prefix)
    save_dict = mx.nd.load('%s-%d.params' % (prefix, epoch))
    arg_params = {}
    aux_params = {}
    for k, v in save_dict.items():
        tp, name = k.split(':', 1)
        if tp == 'arg':
            arg_params[name] = v
        if tp == 'aux':
            aux_params[name] = v
    return (arg_params, aux_params, symbol)


def save_checkpoint(prefix, epoch, symbol, arg_params, aux_params):
    if symbol is not None:
        symbol.save('%s-symbol.json' % prefix)

    save_dict = {('arg:%s' % k) : v.as_in_context(mx.cpu()) for k, v in arg_params.items()}
    save_dict.update({('aux:%s' % k) : v.as_in_context(mx.cpu()) for k, v in aux_params.items()})
    param_name = '%s-%d.params' % (prefix, epoch)
    mx.nd.save(param_name, save_dict)
    logging.info('Saved checkpoint to \"%s\"', param_name)




class IOU(object):
    def __init__(self, class_num, ignored_label=255):
        self.ignored_label = ignored_label
        self.class_num = class_num
        self.conf_mat = None
        self.reset()

    def reset(self):
        self.conf_mat = np.zeros((self.class_num, self.class_num), dtype=np.ulonglong)

    def update(self, label, pred_label):
        self.__eval_pair(pred_label, label)

    def __eval_pair(self, pred_label, label):
        valid_index = label.flat != self.ignored_label
        gt = np.extract(valid_index, label.flat)
        p = np.extract(valid_index, pred_label.flat)
        temp = np.ravel_multi_index(np.array([gt, p]), (self.conf_mat.shape))
        temp_mat = np.bincount(temp, minlength=np.prod(self.conf_mat.shape)).reshape(self.conf_mat.shape)
        self.conf_mat[:]=self.conf_mat+temp_mat

    def get(self):
        return "iou", np.mean(self.get_class_scores())

    def get_class_scores(self):
        scores = []
        for i in range(self.class_num):
            tp = np.longlong(self.conf_mat[i, i])
            gti = np.longlong(self.conf_mat[i, :].sum())
            resi = np.longlong(self.conf_mat[:, i].sum())
            denom = gti+resi-tp
            try:
                res = float(tp)/denom
            except ZeroDivisionError:
                res = 0
            scores.append(res)
        return scores

class Speedometer(object):
    """Calculate and log training speed periodically.

    Parameters
    ----------
    batch_size: int
        batch_size of data
    frequent: int
        How many batches between calculations.
        Defaults to calculating & logging every 50 batches.
    """
    def __init__(self, batch_size, frequent=50):
        self.batch_size = batch_size
        self.frequent = frequent
        self.init = False
        self.tic = 0
        self.last_count = 0

    def __call__(self, param):
        """Callback to Show speed."""
        count = param.nbatch
        if self.last_count > count:
            self.init = False
        self.last_count = count

        if self.init:
            if count % self.frequent == 0:
                speed = self.frequent * self.batch_size / (time.time() - self.tic)
                if param.eval_metric is not None:
                    name_value = param.eval_metric.get_name_value()
                    param.eval_metric.reset()
                    res_str = ""
                    cur_time = time.strftime("%d/%m/%Y--%H:%M:%S")
                    for name, value in name_value:
                        res_str += "\t%s=%f" % (name, value)

                    logging.info('Epoch[%d] Batch[%d]\tSpeed: %.2f samples/sec\t%s\ttime=%s', param.epoch, count, speed, res_str, cur_time)
                else:
                    logging.info("Iter[%d] Batch[%d]\tSpeed: %.2f samples/sec",
                                 param.epoch, count, speed)
                self.tic = time.time()
        else:
            self.init = True
            self.tic = time.time()


def get_file_list(root_dir, flist_name):
    with open(os.path.join(root_dir,flist_name)) as f:
        names = f.read().splitlines()
        flist = []
        for name in names:
            im, l = name.split('\t')
            flist.append(
                (im, l))
        return flist


def find_arguments(list, keyword):
    result = []
    for name in list:
        if name.find(keyword) != -1:
            result.append(name)
    return result


def upsample_filt(size):
    factor = (size + 1) // 2
    if size % 2 == 1:
        center = factor - 1
    else:
        center = factor - 0.5
    og = np.ogrid[:size, :size]
    return (1 - abs(og[0] - center) / factor) * \
           (1 - abs(og[1] - center) / factor)


def get_up_kernel(size, channel):
    kernel = np.zeros((channel, channel, size, size))
    kernel[range(channel), range(channel), :, :] = upsample_filt(size)
    return kernel


def pad_image(img_array, downsample_scale):
    r = img_array.shape[-2]
    c = img_array.shape[-1]
    orig_size = (r, c)
    scale = int(downsample_scale)
    r_pad = 0
    c_pad = 0
    if r % scale > 0:
        r_pad = (r/scale+1)*scale - r
    if c % scale > 0:
        c_pad = (c/scale+1)*scale - c
    if r_pad>0 or c_pad>0:
        img_array = np.lib.pad(img_array, ((0,0),(0,0),(0,r_pad), (0, c_pad)), 'constant', constant_values=0)
    return img_array, orig_size


def crop_pred(pred, orig_size):
    pred = pred[:,0:orig_size[0],0:orig_size[1]]
    return pred


def im_to_array(im):
    im_array = np.array(im, dtype=np.float32)
    im_array = np.transpose(im_array, [2, 0, 1])
    return im_array


def clean_dict(arg_dict, keyword_list):
    new_dict = {}
    for key in arg_dict:
        flag = False
        for k in keyword_list:
            if k in key:
                flag = True
                break
        if not flag:
            new_dict[key] = arg_dict[key].asnumpy()
    return new_dict

def segment_confidence(conf_map, pred_label, num_classes):
  mean_conf_map = np.zeros_like(pred_label).astype('float')
  all_segs_img = np.zeros_like(pred_label)

  all_confs = [] # array of segment confidences
  all_confs_classes = [] # array of segment classes
  seginfo = {}

  seg_cnt=0


  for c in np.unique(pred_label):
    if c<1 or c>num_classes:
      continue # this is unknown / tote
    # binarize segmentation
    binmask = np.zeros_like(pred_label)
    binmask[np.where(pred_label==c)] = 1
    # find individual connected components
    ccs = measure.label(binmask.astype(bool), background=0)
    ccsind, counts = np.unique(ccs, return_counts=True)
    srtidx = counts.argsort()[::-1] # sort segments largest to smallest

    for cc in ccsind:
      if cc==0:
        continue # this is background
      binmask_oneseg = np.zeros_like(binmask)
      binmask_oneseg[np.where(ccs==cc)] = 1

      seg_cnt += 1
      all_segs_img[np.where(ccs==cc)] = seg_cnt

      # mean conf
      mean_conf = conf_map[np.where(ccs==cc)].mean()
      mean_conf_map[np.where(ccs==cc)] = mean_conf
      #np.append(all_confs, mean_conf)
      #all_confs[seg_cnt] = mean_conf
      seginfo[seg_cnt] = (c, cc, mean_conf, (0,0)) # class ID, segment count, mean confidence, centroid

      # put text only on largest segment
      if cc==srtidx[1]:

        com = ndimage.measurements.center_of_mass(binmask_oneseg)
        seginfo[seg_cnt] = (c, cc, mean_conf, com) # class ID, segment count, mean confidence, centroid


  return mean_conf_map, all_segs_img, seginfo

def confidence_maps(pred):

  # compute differences in confidences
  srtdclasses = pred.argsort(0)[::-1] # sorted classes for each pixel
  # indices for each pixel for best and second best
  bestclasses = srtdclasses[0].flatten()
  secondbestclasses = srtdclasses[1].flatten()

  # create indices
  img_height = pred.shape[1]
  img_width = pred.shape[2]
  rar, car = np.meshgrid(np.arange(img_height),np.arange(img_width))
  rar = rar.flatten('F')
  car = car.flatten('F')
  best_scores_map = pred[bestclasses,rar,car].reshape(img_height, img_width)
  second_best_scores_map = pred[secondbestclasses,rar,car].reshape(img_height, img_width)
  diff_conf_map = best_scores_map-second_best_scores_map

  return best_scores_map, second_best_scores_map, diff_conf_map
