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

# for item list
sys.path.append('../eval_semseg')
import items
tr_items = items.getTrainingItems()
all_items = items.getAllItems()



all_items = [x.lower() for x in all_items]
INPUT_PATH = '/home/amilan/research/projects/ARChallenge2017/ARCDataset/0701_data_capture/seen/'
OUTPUT_PATH = 'outputs/20170701_tote_bin/125/'
rotaugm=False # flag for using rotation augmentation

# file list to go through
flist = INPUT_PATH + 'imglist.txt'
scene = 'tote' # scene type

with open(flist) as fl:
  imfiles = fl.readlines()
imfiles = [x.strip() for x in imfiles]   # get rid of \n escape chars

F1SHOTS_PATH='f1shots/' # where will the new data go?

fl = open('f1shots_list.txt', 'w') # open file to write out file list
for imfile in imfiles:
  # get foreground-background segmentation image
  raw_output_file = OUTPUT_PATH+'/raw_'+imfile
  print(raw_output_file)
  lab = io.imread(raw_output_file)
  
  # TODO
  # run 2-class RefineNet to separate foreground and background
  #mod.forward(mx.io.DataBatch(data=[mx.nd.array(im)], label=None, pad=None, index=None))
  #pred = mod.get_outputs()[0].asnumpy().squeeze()
  

  # manually cropped bounding box of container (tote)
  #bbox = (540,0,870,1080) # left, top, w, h of storage/tote (inclusive)
  bbox = (21, 0, 602, 350)
  bbx2 = bbox[0]+bbox[2]-1 # right
  bby2 = bbox[1]+bbox[3]-1 # bottom
  

  # get captured RGB image  
  rgb = io.imread(INPUT_PATH+imfile)
  
  # crop to defined bbox
  rgb = rgb[bbox[1]:bby2, bbox[0]:bbx2, :]
  lab = lab[bbox[1]:bby2, bbox[0]:bbx2]

  # image dimensions and label IDs (those should be 0,1
  (img_height, img_width) = lab.shape
  labids = np.unique(lab)




  bg_container_id = all_items.index(scene)

  mask_bg_container = np.ones_like(lab) # all ones img
  #mask_bg_container[bbox[1]:bby2, bbox[0]:bbx2] = 0 # set bbox to zeros

  # now set everything outside bbox to ones
  #lab[np.where(mask_bg_container==1)] = labids[1]
  #lab[:,0:bbox[0]] = labids[1]
  #lab[:,1410:] = labids[1]
  lab = lab.astype(bool)
  lab = np.invert(lab)

  print(lab.dtype)
  print(np.unique(lab))

  all_labels = measure.label(lab, background=0)

  print(np.unique(all_labels))

  ccs, counts = np.unique(all_labels, return_counts=True)
  #dict(zip(ccs, counts))

  print(ccs)
  print(counts)
  srtidx = counts.argsort()[::-1]
  print(srtidx)

  fname = INPUT_PATH+imfile
  fname = fname.replace('_rgb.png','_map.txt')
  
  print("Reading %s" % fname)
  with open(fname) as f:
    content = f.readlines()
    # you may also want to remove whitespace characters like `\n` at the end of each line
    
  content = [x.strip() for x in content] 
  print(content)

  nItems = len(content)
  assert nItems in (2,4), 'Number of items should be 2 or 4'

  # define anchors (np style, x=vertical, y=horizontal)
  if nItems == 2:
    anchors = np.array([[0,img_width/2], [img_height, img_width/2]]) # top | bottom
    anchors = np.array([[img_height/2, img_width], [img_height/2, 0]]) # right | left
  else:
    anchors = np.array([[0, 0],
                        [0, img_width],                      
                        [img_height, img_width],
                        [img_height, 0]
                        ])

  print("Anchors:")
  print(anchors)

  # create mask
  mask = np.zeros_like(lab, dtype=np.uint8) # all zeros for unknown
  mask = all_items.index(scene)*np.ones_like(lab, dtype=np.uint8) # fill with known background
  mask[np.where(mask_bg_container==0)] = all_items.index(scene) # mark bounding box of container manually
  

  # label n larges segments with 1...4
  for c in range(nItems):
    mask[np.where(all_labels==srtidx[c+1])] = c+1 # +1 because zeros are unknown
    
  # by now we should have extracted the right number of segments
  segs = np.unique(mask)
  segs = np.setdiff1d(segs, np.array([0, bg_container_id])) # remove unknown and bg_countainer class
  nSegs = len(segs)
  print("Expected number of items:\t%d" % nItems)
  print("Recovered number of segments:\t%d" % nSegs)
  assert nSegs == nItems, "Segmentation must have failed"

  # sort top->bottom, or clockwise from left-top
  # 1. Find centroids 
  coms=np.zeros((nItems,2))
  for idx,c in enumerate(segs):
    binmask = np.zeros_like(mask)
    binmask[np.where(mask==c)] = 1
    
    com = ndimage.measurements.center_of_mass(binmask)
    coms[idx] = com 


  # do assignment
  distmat = pairwise_distances(anchors, coms) # pairwise distances (i,j) = dist(anchor[i], seg[j])
  ass = distmat.argmin(1) # which anchors to assign to
  print(anchors)
  print(coms)
  print(distmat)
  print(ass)
  print(segs)
  print(distmat.argmin(0))
  print(distmat.argmin(1))

  # reassign
  newmask = mask.copy()
  for idx,c in enumerate(segs):
    # idx = anchor ID    
    seg_to_replace = ass[idx] + 1  # + 1 shift to accommodate unknown as 0    
    newmask[np.where(mask==seg_to_replace)] = idx+1
  mask = newmask.copy()

  coms=np.zeros((nItems,2))
  for idx,c in enumerate(segs):
    binmask = np.zeros_like(mask)
    binmask[np.where(mask==c)] = 1
    
    com = ndimage.measurements.center_of_mass(binmask)
    coms[idx] = com 
  print(coms)


  # move mask to full label space [0...58]
  newmask = mask.copy()
  for idx,c in enumerate(segs):
    classID = all_items.index(content[idx])
    # TODO sanity checks
    # What if item not found?
    newmask[np.where(mask==c)] = classID
    
  mask = newmask.copy()
  
  # find boundaries
  mask_ = mask.copy()
  #mask_[np.where(mask>56)]=0 # don't show tote, storage
  bnd_img = mark_boundaries(rgb, mask_, mode='thick')
  bnd_img = Image.fromarray((bnd_img*255).astype('uint8'));

  ## write out results
  cmap = np.load("SUNcmap.npy")
  mask_cmap = Image.fromarray(np.uint8(mask))
  mask_raw = mask_cmap.copy()
  mask_raw.save(OUTPUT_PATH+'/mask_'+imfile)

  mask_cmap.putpalette(cmap)
  mask_cmap.convert('RGB')


  out_img = mask_cmap.convert('RGB')
  rgb_img = Image.fromarray(rgb)
  out_img = Image.blend(bnd_img, out_img, 0.5)

  print("Available IDs in mask")
  print(np.unique(mask))
  for c in np.unique(mask):
    #if c<1 or c>56:
      #continue
    binmask = np.zeros_like(mask)
    binmask[np.where(mask==c)] = 1
    
    com = ndimage.measurements.center_of_mass(binmask)
    #print(c)
    #print(com)
    fs = int(round(out_img.size[1]/30))
    font = ImageFont.truetype("/usr/share/fonts/dejavu/DejaVuSans.ttf", fs)
    draw = ImageDraw.Draw(out_img)
    draw.text((com[1], com[0]), 'O', (0,0,255), font=font)
    draw.text((com[1], com[0]), all_items[c], (255,255,0), font=font)
    #draw.text((com[1], com[0]+30), "%.0f,%.0f" % (com[1],com[0]), (255,255,0), font=font)
    
    draw = ImageDraw.Draw(out_img)
                
  out_img.save(OUTPUT_PATH+'/autoseg_'+imfile)  
  rgb_img.save(OUTPUT_PATH+imfile)  
  # add to file list
  fl.write("%s\t%s\n" % ((F1SHOTS_PATH+'/'+imfile), (F1SHOTS_PATH+'/mask_'+imfile)))
  
  if rotaugm:
    randdegs = np.random.rand(10) * 360
    for d,deg in enumerate(randdegs):
      out_img_rot = out_img.rotate(deg)
      outfile = imfile + 'rot'+str(d)+'.png'
      
      mask_rot = mask_raw.rotate(deg)
      mask_rot.save(OUTPUT_PATH+'/mask_'+outfile)
      
      rgb_rot = rgb_img.rotate(deg)
      rgb_rot.save(OUTPUT_PATH+outfile)
      
      out_img_rot.save(OUTPUT_PATH+'/autoseg_'+outfile)
      fl.write("%s\t%s\n" % ((F1SHOTS_PATH+'/'+outfile), (F1SHOTS_PATH+'/mask_'+outfile)))
  #break
  
fl.close()