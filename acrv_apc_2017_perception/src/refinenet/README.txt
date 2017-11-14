STEP1: set up the data (PASCAL VOC 12)

To train in VOC12, it usually involves three parts, COCO, SBD and original VOC 12. So gather all training images and put them into "VOC_DATA/train_images" and put all training masks into "VOC_DATA/train_labels". Similarly, there are 1449 images in VOC12 validation set. Put these into "VOC_DATA/val_images" and "VOC_DATA/val_labels" respectively.
In "VOC_DATA", there are four lists:
train++.txt         include images from COCO, SBD and VOC12
train+.txt          include images from SBD and VOC12
tran.txt            include images from VOC12
val_list.txt        include images in VOC12 validation set
===============================================================================================
STEP2: training

To train a model, simply run:

python train.py --gpus 0 

To evaluate a trained model, since training and validation are separate parts in this code, which means validation is another independent script. We evaluate the saved snapshots (e.g. epoch 1) by running:

python test.py --gpu 0 --epoch 1

or there is a useful script "start_evalutation.sh" to help us find unevaluated snapshots and do it automatically. just:

./start_evalutation.sh --gpu 0


The trainin for VOC12 is divided into three parts, where the three lists are used respectively. 
phase 1:
python train.py --gpus 0 --flist train++.txt --lr 1e-4 --batch-size 1
Then evaluate the snapshots at the same time to find out the good checkpoint.

phase 2:
for example, we found epoch 50 is good enough, then we continue training by:
python train.py --gpus 0 --flist train+.txt --lr 1e-5 --batch-size 1 --epoch 50
Evaluate the snapshots

phase 3:
Suppose we start from epoch 80, then:
python train.py --gpus 0 --flist train.txt --lr 1e-6 --batch-size 1 --epoch 80

===============================================================================================
EXTRA INFO
All the evaluated images are stored in "outputs/xx", where "xx" is the epoch number. Please notice that "start_evaluation.sh" decides whether to do evalutation based on this corresponding folder. It means if the folder exists, it indicates the evaluation was already done, so skipped. Therefore, if you want to do evaluation again, please delete the corresponding folder.

Also the training parameters might not be optimal in terms of learning rate, batch size etc. One can try multi-gpu training, for example:

python train.py --gpus 0,1,2,3 --batch-size 8, --lr 1e-4 --flist train++.txt

