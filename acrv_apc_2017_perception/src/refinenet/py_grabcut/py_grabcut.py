import numpy as np
import cv2
from matplotlib import pyplot as plt
import glob 		# for iterating through directories
import sys
import os
     
path_training_items = '/home/amilan/ownCloud/ARChallenge2017/acrv_apc_2017_data/data/items/Training_items/'
#path_training_items = '/home/amilan/ownCloud/ARChallenge2017/acrv_apc_2017_data/data/items/Unseen_items/'

#img = cv2.imread('ballons.jpg')
#img = cv2.imread('/home/amilan/ownCloud/ARChallenge2017/acrv_apc_2017_data/data/items/Training_items/Balloons/Balloons_Bottom-Side_01.png');
#img = cv2.imread('/home/amilan/ownCloud/ARChallenge2017/acrv_apc_2017_data/data/items/Training_items/Bath_Sponge/Bath_Sponge_Bottom-Side_01.png');
#img = cv2.imread('/home/amilan/ownCloud/ARChallenge2017/acrv_apc_2017_data/data/items/Training_items/Flashlight/Flashlight_Bottom-Side_01.png');
#img = cv2.imread('/home/amilan/ownCloud/ARChallenge2017/acrv_apc_2017_data/data/items/Unseen_items/Bunny_Book/Bunny_Book_Bottom-Side_01.png');



for dirname in glob.iglob(os.path.join(path_training_items,'*')):
	#for filename in glob.iglob(os.path.join(dirname
    item_name = os.path.split(dirname)[-1]
    #print os.path.join(dirname,item_name +'*.png')
    for filename in glob.iglob(os.path.join(dirname,item_name +'*.png')):
		print("Reading %s" % filename)
		basefilename = os.path.basename(filename)

		img = cv2.imread(filename);
		height, width = img.shape[:2]
		
		# downsize 10x for faster processing
		img = cv2.resize(img, (np.int(0.1*width), np.int(0.1*height)), interpolation = cv2.INTER_CUBIC)

		mask = np.zeros(img.shape[:2],np.uint8)
		mask[:] = 255
		bgdModel = np.zeros((1,65),np.float64)
		fgdModel = np.zeros((1,65),np.float64)


		height_small, width_small = img.shape[:2]
		brd = 5 # border width for background
		rect = (brd,brd,width_small-2*brd,height_small-2*brd) # leave 10 px border
		print "Running GrabCut..."
		# mask is filled with 
		# 0 - an obvious background pixels
		# 1 - an obvious foreground (object) pixel
		# 2 - a possible background pixel
		# 3 - a possible foreground pixel
		cv2.grabCut(img,mask,rect,bgdModel,fgdModel,5,cv2.GC_INIT_WITH_RECT)
		print "Done!"
		mask2 = np.where((mask==2)|(mask==0),0,1).astype('uint8')
		img = img*mask2[:,:,np.newaxis]
		# img = np.where((mask==2)|(mask==0),128,255)
		
		mask_object = np.where((mask==2)|(mask==0),0,255).astype('uint8')
		mask_object = cv2.resize(mask_object, (width, height), interpolation = cv2.INTER_NEAREST )
		writefile = os.path.join(dirname, 'mask_' + basefilename)
		cv2.imwrite(writefile,mask_object)
		print("Writing %s" % writefile)

		#plt.imshow(img),plt.colorbar(),plt.show()
		