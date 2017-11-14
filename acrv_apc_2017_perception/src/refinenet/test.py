import argparse
import logging
import mxnet as mx
import os
from cores.io.EvalDataProducer import EvalDataProducer
import cores.symbols.refineNet101 as net
from cores.utils import misc
from cores.utils import metrics
import numpy as np
import time

from PIL import ImageFont
from PIL import Image
from PIL import ImageDraw

import scipy.io as sio
from skimage import measure
from skimage.segmentation import find_boundaries, mark_boundaries
from scipy import ndimage
#import cv2

import sys
import inspect

currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0, parentdir)
from eval_semseg import items

known_items = [
    [],
    ['Pie_Plates', 'Composition_Book'],
    ['Pie_Plates', 'Composition_Book', 'Epsom_Salts'],
    ['Pie_Plates', 'Composition_Book', 'Epsom_Salts', 'Windex'],
    ['Pie_Plates', 'Composition_Book', 'Epsom_Salts', 'Windex', 'Toilet_Brush'],
    ['Pie_Plates', 'Composition_Book', 'Epsom_Salts', 'Windex', 'Toilet_Brush', 'Robots_Everywhere'],
    ['Pie_Plates', 'Composition_Book', 'Epsom_Salts', 'Windex', 'Toilet_Brush', 'Robots_Everywhere', 'Reynolds_Wrap', 'Ticonderoga_Pencils', 'White_Facecloth', 'Mesh_Cup', 'Colgate_Toothbrush_4PK', 'Robots_DVD'],
    ['Tissue_Box', 'Fiskars_Scissors', 'Bath_Sponge', 'Plastic_Wine_Glass', 'Windex', 'Hanes_Socks', 'Black_Fashion_Gloves'],
    ['Pie_Plates', 'Toilet_Brush','Windex','Robots_Everywhere','Epsom_Salts','Composition_Book','Mesh_Cup','Reynolds_Wrap','Colgate_Toothbrush_4PK','Robots_DVD','Ticonderoga_Pencils','tote'], # 8 - val_00036
    ['Mesh_Cup', 'Laugh_Out_Loud_Jokes', 'Pie_Plates','Epsom_Salts','Table_Cloth','Hand_Weight', 'tote'], # 9 - synth 000014
    [],
    ['Pie_Plates', 'Composition_Book', 'Epsom_Salts', 'Windex', 'Toilet_Brush', 'Robots_Everywhere', 'Reynolds_Wrap', 'Ticonderoga_Pencils', 'White_Facecloth', 'Mesh_Cup', 'Colgate_Toothbrush_4PK', 'Robots_DVD'],
    ['Mesh_Cup', 'Mouse_Traps', 'Colgate_Toothbrush_4PK', 'Black_Fashion_Gloves', 'Scotch_Sponges', 'Glue_Sticks', 'Expo_Eraser', 'Plastic_Wine_Glass', 'Balloons', 'Bath_Sponge', 'Marbles', 'Speed_Stick', 'Measuring_Spoons'],
    ['Mesh_Cup', 'Shower_Curtain', 'Scotch_Sponges', 'Harry_Potter_DVD', 'Windex', 'Utility_Brush', 'Band_Aid_Tape', 'Black_Fashion_Gloves'],
    ['Flashlight','Hand_Weight','tote']
]
ki = 14

OUTPUT_FOLDER = "outputs"
SNAPSHOT_FOLDER = "snapshots"

def main():
    logging.info(args)
    tr_items = items.getTrainingItems()
    all_items = items.getAllItems()
    all_items_lower = [x.lower() for x in all_items]

    
    if not args.autotune:
        os.environ["MXNET_CUDNN_AUTOTUNE_DEFAULT"] = "0"
    if not os.path.exists(OUTPUT_FOLDER):
        os.mkdir(OUTPUT_FOLDER)
    epoch_str = str(args.epoch)
    #if not os.path.exists(os.path.join(OUTPUT_FOLDER, epoch_str)):
        #os.mkdir(os.path.join(OUTPUT_FOLDER, epoch_str))

    full_output_folder = os.path.join(OUTPUT_FOLDER, args.model_name, epoch_str)
    if not os.path.exists(full_output_folder):
        os.makedirs(full_output_folder)

    ctx = mx.gpu(int(args.gpu))
    cmap = np.load("SUNcmap.npy")


    seg_net = net.create_infer(args.class_num, args.workspace)
    seg_net_prefix = os.path.join(SNAPSHOT_FOLDER, args.model_name)
    arg_dict, aux_dict, _ = misc.load_checkpoint(seg_net_prefix, args.epoch, load_symbol=False)

    mod = mx.module.Module(seg_net, data_names=('data',), label_names=(), context=ctx)
    mod.bind(data_shapes=[("data", (1, 3, args.max_dim, args.max_dim))], for_training=False, grad_req='null')
    mod.init_params(arg_params=arg_dict, aux_params=aux_dict, allow_missing=True)

    data_producer = EvalDataProducer(
            root_dir=args.data_dir,
            flist_path=os.path.join(args.data_dir, args.flist),
            data_worker_num = args.worker_num)

    nbatch = 0
    start = time.time()
    iou_metric = metrics.IOU(args.class_num)
    print "Start evalutation for epoch [%d]" % args.epoch

    filelist = os.path.join(full_output_folder, 'test.txt')

    fl = open(filelist, 'w') # open file to write out file list
    while True:
        data = data_producer.get_data()
        if data is None:
            break
        #print(data[0].dtype)
        im = data[0]
        im_disp = im[0]
        im_disp = np.swapaxes(im_disp, 2, 1)
        im_disp = np.swapaxes(im_disp, 2, 0)
        # print(im_disp.shape)
        # cv2.imshow('weird', im_disp)
        # cv2.waitKey(0)
        im, orig_size = misc.pad_image(im, 32)
        label = data[1].squeeze()
        label_name = data[2]
        #print(data)
        rgb_name = data[3]
        full_label_name = data[5]
        #print(np.unique(label))

        if not args.fixed_size:
            mod.reshape([("data", im.shape)])
        mod.forward(mx.io.DataBatch(data=[mx.nd.array(im)], label=None, pad=None, index=None))
        pred = mod.get_outputs()[0].asnumpy().squeeze()

        pred = misc.crop_pred(pred, orig_size)

        # for i in range(pred.shape[0]):
        #     pred_disp = pred[i]
        #     pred_disp = np.swapaxes(pred_disp, 0, 1)
        #     cv2.imshow('hd_image', pred_disp)
        #     cv2.waitKey(500)

        if args.binary:
          all_items=['object','tote']
        else:              
          # Anton's hack to determine if we test on Dropbox classification_output
          if args.live:
            fname = os.path.join(args.data_dir, rgb_name)
            fname = fname.replace('_rgb.png','_items.txt')
            
            print("Reading %s" % fname)
            with open(fname) as f:
              known_items = f.readlines()              
            known_items = [x.strip() for x in known_items] 
            if 'tote' not in known_items:
              known_items.append('tote')
            print(known_items)
            #print(all_items_lower)
            #sys.exit()
            for i in all_items_lower[1:]:
                j = all_items_lower.index(i)
                #print(i,j)
                if i in known_items:
                    #print('Not zeroing pred[%d]' % (j))
                    continue
                pred[j] = np.zeros_like(pred[j])
          else:
            # normal test with GT mask
            for i in range(args.class_num):
              if i in np.unique(label):
                  #print('Not zeroing pred[%d]' % (i))
                  continue
              pred[i] = np.zeros_like(pred[i])
              

            # for i in tr_items[1:]:
            #     j = tr_items.index(i)
            for i in all_items[1:]:
                j = all_items.index(i)
                #print(np.unique(pred[j]))
                #if i in known_items[ki]:
                    #print('Not zeroing pred[%d]' % (j))
                    #continue
                #pred[j] = np.zeros_like(pred[j])
                ##print('%d (%s) disabled' % (j, i))
                ##print(np.unique(pred[j]))

        # Do argmax on softmax
        pred_label = pred.argmax(0)

        
              
        #####
        if args.second_check:
        
          # generate a random 20 items list including the test item
          fname = os.path.join(args.data_dir,rgb_name) # file name
          fname = fname.replace('_rgb.png','_map.txt')
          
          with open(fname) as f:
            content = f.readlines()
            # you may also want to remove whitespace characters like `\n` at the end of each line
            
          content = [x.strip() for x in content] 
          print(content)
          presentID = all_items_lower.index(content[0])
          print(presentID)
          
          # select a random present items list
          rand_item_list = np.random.permutation(57)
          rand_item_list = rand_item_list[0:5]
          # if 0 or object missing, insert
          if not 0 in rand_item_list:
            rand_item_list=np.insert(rand_item_list,0,0)
          if not presentID in rand_item_list:          
            rand_item_list=np.insert(rand_item_list,0,presentID)
            
          #rand_item_list = rand_item_list[0:20]
          #rand_item_list = [0,presentID,57]
          print(rand_item_list)
          
          print(np.unique(pred_label))
          for j in range(59):
            if j in rand_item_list:
              continue
            pred[j] = np.zeros_like(pred[j])

          pred_label = pred.argmax(0)
          print(np.unique(pred_label))
          for i in np.unique(pred_label):
            print(i,all_items_lower[i])
          
          
          # set anything outside a predefined bbox to unknown
          bbox = (150, 30, 400, 400) # left, top, w, h
          bbx2 = bbox[0]+bbox[2]-1 # right
          bby2 = bbox[1]+bbox[3]-1 # bottom
          roi = np.zeros_like(pred_label)
          roi[bbox[1]:bby2, bbox[0]:bbx2] = 1 # set region of interest to ones
          pred_label[np.where(roi==0)]=0 # set anything outside to unknown
          

        #########
        
  
        #print(np.unique(pred_label))

        iou_metric.update(label.reshape(1, -1), pred_label.reshape(1, -1))
        print(np.mean(iou_metric.get_class_scores()))


        if args.output:
            # raw output
            out_lab_raw = Image.fromarray(np.uint8(pred_label))
            
            pred_label_ = pred_label.copy()
            pred_label_[np.where(pred_label_>56)]=0 # don't show tote, storage
            out_lab = np.uint8(pred_label_)

            out_lab = Image.fromarray(out_lab)
            
            

            out_lab.putpalette(cmap)
            out_lab = out_lab.convert('RGB')
            rgb_img = data[4]

            rgb_img_arr = np.array(rgb_img, dtype=np.float32) / 255

            bnd_img = mark_boundaries(rgb_img_arr, pred_label_, mode='thick')

            bnd_img = Image.fromarray((bnd_img*255).astype('uint8'));
            out_img = Image.blend(bnd_img, out_lab, 0.6)
            


            
            
            # segment sizes
            segsizes = np.zeros(59)
            for c in np.unique(pred_label):
              if c<1 or c>56:
                continue
              segsizes[c] = np.sum(np.where(pred_label==c))
            
            sortedsegs = np.argsort(segsizes)[::-1]
            #print(segsizes)
            #print(sortedsegs)


            fs = 12
            font = ImageFont.truetype("/usr/share/fonts/dejavu/DejaVuSans.ttf", fs)
  
            # draw one label for each segment
            if args.seg_conf:
              
              # get confidences
              best_scores_map, second_best_scores_map, diff_conf_map = misc.confidence_maps(pred)
              mean_conf_map, all_segs_img, seginfo = misc.segment_confidence(diff_conf_map, pred_label, 56)
              
              # put text on segments
              for k,v in seginfo.iteritems():
                if v[3][0] == 0: # dummy, ignore
                  continue
              
                draw_label = "%s (%.1f%%)" % (all_items[v[0]], v[2]*100)
        
        
                com = v[3]
      
                        
                draw = ImageDraw.Draw(out_img)
                draw.text((com[1], com[0]), 'O', (0,0,255), font=font)
                draw.text((com[1]-len(all_items[c])/2*fs/2, com[0]), draw_label, (255,255,0), font=font)

                draw = ImageDraw.Draw(out_img)  
                      

              conf_img = Image.fromarray((best_scores_map*255).astype('uint8'))
              conf_img.save(os.path.join(full_output_folder, "conf1_"+rgb_name))
              
              conf_img2 = Image.fromarray((second_best_scores_map*255).astype('uint8'))
              conf_img2.save(os.path.join(full_output_folder, "conf2_"+rgb_name))

              diff_conf_img = Image.fromarray(((diff_conf_map)*255).astype('uint8'))
              diff_conf_img.save(os.path.join(full_output_folder, "diffconf_"+rgb_name))
            
              mean_conf_img = Image.fromarray((mean_conf_map*255).astype('uint8'))            
              mean_conf_img.save(os.path.join(full_output_folder, "segconf_"+rgb_name))
            else:
          
              #draw one label for each item
              for c in np.unique(pred_label):
                if c<1 or c>56:
                  continue # unknown or background
                binmask = np.zeros_like(pred_label)
                binmask[np.where(pred_label==c)] = 1

                com = ndimage.measurements.center_of_mass(binmask)

                fs = 14
                if c==sortedsegs[0]:
                  fs = 28
                #print(c,fs,sortedsegs[0])
                
                draw = ImageDraw.Draw(out_img)
                draw.text((com[1], com[0]), 'O', (0,0,255), font=font)
                draw.text((com[1]-len(all_items[c])/2*fs/2, com[0]), all_items[c], (255,255,0), font=font)

                draw = ImageDraw.Draw(out_img)                

            #print(c,ccs,counts)
            #ccs = (binseg*255).astype('uint8')
            
            #out_img = Image.fromarray(ccs)

            rgb_img.save(os.path.join(full_output_folder, rgb_name))
            out_img.save(os.path.join(full_output_folder, "overlay_"+rgb_name))
            out_lab_raw.save(os.path.join(full_output_folder, "raw_"+rgb_name))
            
            if args.debug:
              for c in range(59):
                if pred[c].sum():
                  conf_mask = pred[c]
                  conf_img = Image.fromarray((conf_mask*(255/conf_mask.max())).astype('uint8'))
                  conf_img.save(os.path.join(full_output_folder, "conf_"+all_items_lower[c]+"_"+rgb_name));

            
            #rgb_img .save(os.path.join("outputs", str(args.epoch), label_name))

            # add to file list
            fl.write("%s\t%s\n" % (os.path.join(full_output_folder, "raw_"+rgb_name), os.path.join(args.data_dir, full_label_name)))
        nbatch += 1
        if nbatch % 100 == 0:
            print "processed %d images" % nbatch


        #break
    fl.close()

    logging.info("Average time(s) per batch: %.2f",(time.time()-start)/nbatch)
    class_scores = iou_metric.get_class_scores()
    value = np.mean(class_scores)
    logging.info("Evaluation summary-Epoch[%d] iou=%.4f", args.epoch, value)
    score_str = "\n"
    i = 0
    for s in class_scores:
      if s or 1:
        score_str += "%30s (%3d) %.4f\n" % (all_items[i], i, s)
      i += 1

    logging.info("Class scores-Epoch[%d]: %s", args.epoch, score_str)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Inference")
    parser.add_argument("--epoch", default=0, type=int,
                        help="Epoch number.")
    parser.add_argument("--gpu", default=0,
                        help="Device index.")
    parser.add_argument("--output", default=True, type=bool,
                        help="Produce the output.")
    parser.add_argument("--class-num", default=41, type=int,
                        help="Class number.")
    parser.add_argument("--model-name", default="refineNet101",
                        help="Model name.")
    parser.add_argument("--workspace", default=1000, type=int,
                        help="Workspace size.")
    parser.add_argument("--data-dir", default="VOC_DATA",
                        help="Directory for dataset.")
    parser.add_argument("--flist", default="val.txt",
                        help="File list for dataset.")
    parser.add_argument("--max-dim", default=640, type=int,
                        help="Image width.")
    parser.add_argument("--worker-num", default=1, type=int,
                        help="Number of data workers.")
    parser.add_argument("--fixed-size", default=False, type=bool,
                        help="If input is fixed size.")
    parser.add_argument("--autotune", default=False, type=bool,
                        help="Use auto tuning.")
    parser.add_argument("--second-check", default=False, type=bool,
                        help="Run on second check.")
    parser.add_argument("--binary", default=False, type=bool,
                        help="Run binary segmentation.")
    parser.add_argument("--seg-conf", default=False, type=bool,
                        help="Return segment confidence")
    parser.add_argument("--live", default=False, type=bool,
                        help="Run on live captured images ")
    parser.add_argument("--debug", default=False, type=bool,
                        help="Debug output. All confidences")

    args = parser.parse_args()

    LOG_FILE_NAME = "evallog"
    LOG_FILE_NAME = LOG_FILE_NAME + "_" + args.model_name + ".log"
    
    logging.basicConfig(filename=LOG_FILE_NAME, level=logging.INFO)
    console = logging.StreamHandler()
    logging.getLogger().addHandler(console)
    main()
