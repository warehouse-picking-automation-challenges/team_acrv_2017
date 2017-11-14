import mxnet as mx
import numpy as np


def _attr_scope_lr(lr_type, lr_owner):
    assert lr_type in ('alex', 'alex10', 'torch', 'psp')
    # weight (lr_mult, wd_mult); bias;
    # 1, 1; 2, 0;
    if lr_type == 'alex':
        if lr_owner == 'weight':
            return mx.AttrScope()
        elif lr_owner == 'bias':
            return mx.AttrScope(lr_mult='2.', wd_mult='0.')
        else:
            assert False
    # 10, 1; 20, 0;
    if lr_type == 'alex10':
        if lr_owner == 'weight':
            return mx.AttrScope(lr_mult='10.', wd_mult='1.')
        elif lr_owner == 'bias':
            return mx.AttrScope(lr_mult='20.', wd_mult='0.')
        else:
            assert False
    # 0, 0; 0, 0;
    # so apply this to both
    if lr_type == 'fixed':
        assert lr_owner in ('weight', 'bias')
        return mx.AttrScope(lr_mult='0.', wd_mult='0.')
    # 1, 0; 1, 0;
    if lr_type == 'psp':
        assert lr_owner in ('weight', 'bias')
        return mx.AttrScope(wd_mult='0.')
    # 1, 1; 1, 1;
    # so do nothing
    return mx.AttrScope()

def relu(data):
    return mx.sym.Activation(data, act_type='relu')

def prelu(data, name):
    return mx.sym.LeakyReLU(data, act_type='prelu', name=name)

def dropout(data, p=0.5):
    return mx.sym.Dropout(data, p=p)

def conv(data, name, num_filter, pad=0, kernel=1, stride=1, dilate=1, no_bias=False, workspace=512, lr_type="alex"):
  
    with _attr_scope_lr(lr_type, 'weight'):
        weight = mx.sym.Variable('{}_weight'.format(name))
    if no_bias:
        return mx.sym.Convolution(data=data, weight=weight, name=name,
                                  kernel=(kernel, kernel),
                                  stride=(stride, stride),
                                  dilate=(dilate, dilate),
                                  pad=(pad, pad),
                                  num_filter=num_filter,
                                  workspace=workspace,
                                  no_bias=True)
    else:
        with _attr_scope_lr(lr_type, 'bias'):
            bias = mx.sym.Variable('{}_bias'.format(name))
        return mx.sym.Convolution(data=data, weight=weight, bias=bias, name=name,
                                  kernel=(kernel, kernel),
                                  stride=(stride, stride),
                                  dilate=(dilate, dilate),
                                  pad=(pad, pad),
                                  num_filter=num_filter,
                                  workspace=workspace,
                                  no_bias=False)
def aconv(data, name, num_filter, vertical, pad=0, kernel=1, no_bias=False, workspace=512, lr_type="alex"):
    with _attr_scope_lr(lr_type, 'weight'):
        weight = mx.sym.Variable('{}_weight'.format(name))
    if vertical:
        k = (kernel, 1)
        p = (pad, 0)
    else:
        k = (1, kernel)
        p = (0, pad)

    if no_bias:
        return mx.sym.Convolution(data=data, weight=weight, name=name,
                                  kernel=k,
                                  pad=p,
                                  num_filter=num_filter,
                                  workspace=workspace,
                                  no_bias=True)
    else:
        with _attr_scope_lr(lr_type, 'bias'):
            bias = mx.sym.Variable('{}_bias'.format(name))
        return mx.sym.Convolution(data=data, weight=weight, bias=bias, name=name,
                                  kernel=k,
                                  pad=p,
                                  num_filter=num_filter,
                                  workspace=workspace,
                                  no_bias=False)

def bn(data, name, eps=1.001e-5, fix_gamma=False, use_global_stats=True, lr_type="alex"):
    with _attr_scope_lr(lr_type, 'bias'):
        beta = mx.sym.Variable('{}_beta'.format(name))
    if fix_gamma:
        return mx.sym.BatchNorm(data=data, beta=beta, name=name,
                        eps=eps,
                        fix_gamma=True,
                        use_global_stats=use_global_stats)
    else:
        with _attr_scope_lr(lr_type, 'weight'):
            gamma = mx.sym.Variable('{}_gamma'.format(name))
        return mx.sym.BatchNorm(data=data, beta=beta, gamma=gamma, name=name,
                        eps=eps,
                        fix_gamma=False,
                        use_global_stats=use_global_stats)
    




def maxpool(data, kernel=1, pad=0, stride=1):
    return mx.sym.Pooling(data, kernel=(kernel, kernel),
                        pad=(pad,pad),
                        stride=(stride, stride),
                        pool_type="max")

def avgpool(data, kernel=1, pad=0, stride=1):
    return mx.sym.Pooling(data, kernel=(kernel, kernel),
                        pad=(pad,pad),
                        stride=(stride, stride),
                        pool_type="avg")                        

def upsample(data, name, scale, num_filter, workspace=512):
    p = int(np.ceil((scale-1)/2.0))
    weight = mx.sym.Variable('upsampling_{}'.format(name), init=mx.init.Bilinear(), lr_mult=0)
    return mx.symbol.Deconvolution(data=data, kernel=(scale*2-scale%2,scale*2-scale%2), stride=(scale, scale), num_filter=num_filter, pad=(p, p), 
        workspace=workspace, num_group=num_filter, no_bias=True, weight=weight)




def crop(data, ref):
    return mx.symbol.Crop(*[data, ref])





















