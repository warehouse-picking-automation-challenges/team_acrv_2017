import net_symbols as syms
import mxnet as mx

def create_CRP(data, num_filter, name, workspace=512, lr_type="alex10"):
    relu1 = syms.relu(data)
    c1 = syms.conv(relu1, kernel=3, pad=1, num_filter=num_filter, name=name+"_conv1", no_bias=True, workspace=workspace, lr_type=lr_type)
    p1 = syms.maxpool(c1, kernel=5, pad=2)

    c2 = syms.conv(p1, kernel=3, pad=1, num_filter=num_filter, name=name+"_conv2", no_bias=True, workspace=workspace, lr_type=lr_type)
    p2 = syms.maxpool(c2, kernel=5, pad=2)
    
    c3 = syms.conv(p2, kernel=3, pad=1, num_filter=num_filter, name=name+"_conv3", no_bias=True, workspace=workspace, lr_type=lr_type)
    p3 = syms.maxpool(c3, kernel=5, pad=2)
    
    c4 = syms.conv(p3, kernel=3, pad=1, num_filter=num_filter, name=name+"_conv4", no_bias=True, workspace=workspace, lr_type=lr_type)
    p4 = syms.maxpool(c4, kernel=5, pad=2)
    
    return relu1+p1+p2+p3+p4

def create_RCU(data, num_filter, name, workspace=512, lr_type="alex10"):
    relu1 = syms.relu(data)
    res1 = syms.conv(relu1, num_filter=num_filter, name=name+"_b1", kernel=3, pad=1, workspace=workspace, lr_type=lr_type)
    relu2 = syms.relu(res1)
    res2 = syms.conv(relu2, num_filter=num_filter, name=name+"_b2", kernel=3, pad=1, workspace=workspace, no_bias=True, lr_type=lr_type)
    return data+res2

def create_big_RCU(data, num_filter, name, workspace=512, lr_type="alex10"):
    rcu1 = create_RCU(data, num_filter=num_filter, name=name+"_part1", workspace=workspace, lr_type=lr_type)
    rcu2 = create_RCU(rcu1, num_filter=num_filter, name=name+"_part2", workspace=workspace, lr_type=lr_type)
    return rcu2

def create_refine_block(input1, name, num_filter1, num_filter2=None, input2=None, workspace=512, lr_type="alex10"):
    rcu1 = create_big_RCU(input1, num_filter=num_filter1, name=name+"_rcu1", workspace=workspace, lr_type=lr_type)
    if input2 is not None:
        assert num_filter2 is not None
        rcu2 = create_big_RCU(input2, num_filter=num_filter2, name=name+"_rcu2", workspace=workspace, lr_type=lr_type)
        adapt1 = syms.conv(data=rcu1, kernel=3, pad=1, num_filter=num_filter2, name=name+"_adapt1", workspace=workspace, no_bias=True, lr_type=lr_type)
        adapt2 = syms.conv(data=rcu2, kernel=3, pad=1, num_filter=num_filter2, name=name+"_adapt2", workspace=workspace, no_bias=True, lr_type=lr_type)

        up1 = syms.upsample(adapt1, name=name+"_up1", scale=2, num_filter=num_filter2)
        res = up1+adapt2
        crp = create_CRP(res, name=name+"_crp", num_filter=num_filter2, workspace=workspace, lr_type=lr_type)
        output_rcu = create_RCU(crp, num_filter=num_filter2, name=name+"_outputrcu", workspace=workspace, lr_type=lr_type)
        return output_rcu
    else:
        crp = create_CRP(rcu1, name=name+"_crp", num_filter=num_filter1, workspace=workspace, lr_type=lr_type)
        rcu2 = create_RCU(crp, num_filter=num_filter1, name=name+"_outputrcu", workspace=workspace, lr_type=lr_type)
        return rcu2


def create_block(data, name, num_filter, kernel, pad=0, stride=1, dilate=1, workspace=512, use_global_stats=True):
    res = syms.conv(data=data, name="res" + name, num_filter=num_filter, pad=pad, kernel=kernel, stride=stride, dilate=dilate, no_bias=True, workspace=workspace)
    bn = syms.bn(res, name="bn" + name, use_global_stats=use_global_stats)
    return bn


def create_big_block(data, name, num_filter1, num_filter2, stride=1, dilate=1, pad=1, identity_map=True, workspace=512, use_global_stats=True):
    blocka = create_block(data, name=name+"_branch2a", num_filter=num_filter1, kernel=1, stride=stride, workspace=workspace, use_global_stats=use_global_stats)
    relu1 = syms.relu(blocka)
    blockb = create_block(relu1, name=name + "_branch2b", num_filter=num_filter1, kernel=3, dilate=dilate, pad=pad, workspace=workspace, use_global_stats=use_global_stats)
    relu2 = syms.relu(blockb)
    blockc = create_block(relu2, name=name+"_branch2c", num_filter=num_filter2, kernel=1, workspace=workspace, use_global_stats=use_global_stats)
    if identity_map:
        return syms.relu(data+blockc)
    else:
        branch1 = create_block(data, name=name+"_branch1", num_filter=num_filter2, kernel=1, stride=stride, workspace=workspace, use_global_stats=use_global_stats)
        return syms.relu(branch1+blockc)


def create_res101_main(data, workspace=512, use_global_stats=True):
    conv1 = syms.conv(data, name="conv1", num_filter=64, pad=3, kernel=7, stride=2, workspace=workspace, no_bias=True)
    bn = syms.bn(conv1, name="bn_conv1", use_global_stats=use_global_stats)
    relu = syms.relu(bn)
    pool1 = syms.maxpool(relu, kernel=3, stride=2, pad=1)

    res2a = create_big_block(pool1, "2a", 64, 256, identity_map=False, workspace=workspace, use_global_stats=use_global_stats)
    res2b = create_big_block(res2a, "2b", 64, 256, workspace=workspace, use_global_stats=use_global_stats)
    res2c = create_big_block(res2b, "2c", 64, 256, workspace=workspace, use_global_stats=use_global_stats)
    res3a = create_big_block(res2c, "3a", 128, 512, stride=2, identity_map=False, workspace=workspace, use_global_stats=use_global_stats)
    res3b1 = create_big_block(res3a, "3b1", 128, 512, workspace=workspace, use_global_stats=use_global_stats)
    res3b2 = create_big_block(res3b1, "3b2", 128, 512, workspace=workspace, use_global_stats=use_global_stats)
    res3b3 = create_big_block(res3b2, "3b3", 128, 512, workspace=workspace, use_global_stats=use_global_stats)

    res4a = create_big_block(res3b3, "4a", 256, 1024, stride=2, identity_map=False, workspace=workspace, use_global_stats=use_global_stats)
    res4b1 = create_big_block(res4a, "4b1", 256, 1024, workspace=workspace, use_global_stats=use_global_stats)
    res4b2 = create_big_block(res4b1, "4b2", 256, 1024, workspace=workspace, use_global_stats=use_global_stats)
    res4b3 = create_big_block(res4b2, "4b3", 256, 1024, workspace=workspace, use_global_stats=use_global_stats)
    res4b4 = create_big_block(res4b3, "4b4", 256, 1024, workspace=workspace, use_global_stats=use_global_stats)
    res4b5 = create_big_block(res4b4, "4b5", 256, 1024, workspace=workspace, use_global_stats=use_global_stats)
    res4b6 = create_big_block(res4b5, "4b6", 256, 1024, workspace=workspace, use_global_stats=use_global_stats)
    res4b7 = create_big_block(res4b6, "4b7", 256, 1024, workspace=workspace, use_global_stats=use_global_stats)
    res4b8 = create_big_block(res4b7, "4b8", 256, 1024, workspace=workspace, use_global_stats=use_global_stats)
    res4b9 = create_big_block(res4b8, "4b9", 256, 1024, workspace=workspace, use_global_stats=use_global_stats)
    res4b10 = create_big_block(res4b9, "4b10", 256, 1024, workspace=workspace, use_global_stats=use_global_stats)
    res4b11 = create_big_block(res4b10, "4b11", 256, 1024, workspace=workspace, use_global_stats=use_global_stats)
    res4b12 = create_big_block(res4b11, "4b12", 256, 1024, workspace=workspace, use_global_stats=use_global_stats)
    res4b13 = create_big_block(res4b12, "4b13", 256, 1024, workspace=workspace, use_global_stats=use_global_stats)
    res4b14 = create_big_block(res4b13, "4b14", 256, 1024, workspace=workspace, use_global_stats=use_global_stats)
    res4b15 = create_big_block(res4b14, "4b15", 256, 1024, workspace=workspace, use_global_stats=use_global_stats)
    res4b16 = create_big_block(res4b15, "4b16", 256, 1024, workspace=workspace, use_global_stats=use_global_stats)
    res4b17 = create_big_block(res4b16, "4b17", 256, 1024, workspace=workspace, use_global_stats=use_global_stats)
    res4b18 = create_big_block(res4b17, "4b18", 256, 1024, workspace=workspace, use_global_stats=use_global_stats)
    res4b19 = create_big_block(res4b18, "4b19", 256, 1024, workspace=workspace, use_global_stats=use_global_stats)
    res4b20 = create_big_block(res4b19, "4b20", 256, 1024, workspace=workspace, use_global_stats=use_global_stats)
    res4b21 = create_big_block(res4b20, "4b21", 256, 1024, workspace=workspace, use_global_stats=use_global_stats)
    res4b22 = create_big_block(res4b21, "4b22", 256, 1024, workspace=workspace, use_global_stats=use_global_stats)

    res5a = create_big_block(res4b22, "5a", 512, 2048, stride=2, identity_map=False, workspace=workspace, use_global_stats=use_global_stats)
    res5b = create_big_block(res5a, "5b", 512, 2048, workspace=workspace, use_global_stats=use_global_stats)
    res5c = create_big_block(res5b, "5c", 512, 2048, workspace=workspace, use_global_stats=use_global_stats)

    return [res2c, res3b3, res4b22, res5c]


def create_main(data, class_num, workspace=512, lr_type="alex10"):
    outputs = create_res101_main(data, workspace=workspace)
    
    outputs = outputs[::-1]

    refine1_adapt1 = syms.conv(data=outputs[0], kernel=3, pad=1, num_filter=512, name="refine1_input_adapt1", no_bias=True, workspace=workspace, lr_type=lr_type)
    refine_block1 = create_refine_block(refine1_adapt1, name="refine1", num_filter1=512, workspace=workspace, lr_type=lr_type)

    refine2_adapt2 = syms.conv(data=outputs[1], kernel=3, pad=1, num_filter=256, name="refine2_input_adapt2", no_bias=True, workspace=workspace, lr_type=lr_type)
    refine_block2 = create_refine_block(input1=refine_block1, input2=refine2_adapt2, num_filter1=512, num_filter2=256, name="refine2", workspace=workspace, lr_type=lr_type)

    refine3_adapt2 = syms.conv(data=outputs[2], kernel=3, pad=1, num_filter=256, name="refine3_input_adapt2", no_bias=True, workspace=workspace, lr_type=lr_type)
    refine_block3 = create_refine_block(input1=refine_block2, input2=refine3_adapt2, num_filter1=256, num_filter2=256, name="refine3", workspace=workspace, lr_type=lr_type)

    refine4_adapt2 = syms.conv(data=outputs[3], kernel=3, pad=1, num_filter=256, name="refine4_input_adapt2", no_bias=True, workspace=workspace, lr_type=lr_type)
    refine_block4 = create_refine_block(input1=refine_block3, input2=refine4_adapt2, num_filter1=256, num_filter2=256, name="refine4", workspace=workspace, lr_type=lr_type)

    final_rcu = create_big_RCU(refine_block4, num_filter=256, name="final_rcu1" , workspace=workspace, lr_type=lr_type)
    dp = syms.dropout(data=final_rcu, p=0.5)
    classifier = syms.conv(data=dp, kernel=3, num_filter=class_num, name="score", workspace=workspace, pad=1, lr_type=lr_type)
    return classifier

def create_training(class_num, workspace=512):
    data = mx.symbol.Variable("data")
    net = create_main(data, class_num, workspace=workspace)
    softmax = mx.symbol.SoftmaxOutput(data=net, multi_output=True, use_ignore=True, ignore_label=255, name="softmax", normalization="valid")
    return softmax

def create_infer(class_num, workspace=512):
    data = mx.symbol.Variable("data")
    net = create_main(data, class_num, workspace=workspace)
    net = mx.symbol.SoftmaxActivation(net, mode="channel")
    up4 = syms.upsample(net, name="up4", scale=2, num_filter=class_num)
    up5 = syms.upsample(up4, name="up5", scale=2, num_filter=class_num)
    return up5
