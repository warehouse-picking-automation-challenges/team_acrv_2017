import mxnet.metric
import numpy as np
import mxnet.callback
import time
import logging
import os
from mxnet.metric import check_label_shapes
import mxnet.ndarray as ndarray

class Loss(mxnet.metric.EvalMetric):
    """Calculate loss"""

    def __init__(self):
        super(Loss, self).__init__('loss')

    def update(self, labels, preds):
        check_label_shapes(labels, preds)

        for label, pred in zip(labels, preds):
            pred = pred.asnumpy().reshape(pred.shape[0],pred.shape[1], -1)
            label = label.asnumpy().astype(np.int32)
            valid_index = label != 255
            prob = np.swapaxes(pred, 0, 1)
            prob = prob[:, valid_index]
            label = label[valid_index]

            loss = np.sum(-np.log(prob[label, np.arange(len(label))]))
            self.sum_metric += loss
            self.num_inst += valid_index.sum()


class Accuracy(mxnet.metric.EvalMetric):
    """Calculate accuracy"""

    def __init__(self):
        super(Accuracy, self).__init__('accuracy')

    def update(self, labels, preds):
        check_label_shapes(labels, preds)

        for label, pred_label in zip(labels, preds):
            if pred_label.shape != label.shape:
                pred_label = ndarray.argmax_channel(pred_label)
            pred_label = pred_label.asnumpy().astype(np.int32).reshape(pred_label.shape[0], -1)
            label = label.asnumpy().astype(np.int32)

            check_label_shapes(label, pred_label)


            valid_index = label != 255
            self.sum_metric += (label[valid_index] == pred_label[valid_index]).sum()
            self.num_inst += valid_index.sum()


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
