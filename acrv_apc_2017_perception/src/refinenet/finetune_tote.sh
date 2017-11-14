#!/bin/bash

# Set parameters here

# Which GPU(s) to use
# GPUS=0
GPUS=0,1,2,3

# Where is all the data
DATA_DIR="${HOME}/cloudstor/acrv_apc_2017_data/results_refinenet/20170701_tote_bin"
# DATA_DIR="${HOME}/cloudstor/acrv_apc_2017_data/data_train"

# File list with *all* training files
# FLIST='train_tote.txt'
FLIST='f1shots_list.txt'
# VLIST='val_list.txt'

# New model name
MODEL_NAME=`date +%y%m%d_%H%M`_tote

# Base model name
INIT_MODEL='170713_tote_offline'
EPOCH=199

# EPOCH_SIZE=1
# EPOCH_SIZE=100
EPOCH_SIZE=50
# BATCH_SIZE=16
BATCH_SIZE=32
WORKER_NUM=4

LEARNING_RATE='5e-4'

CLASS_NUM=59

# python2 train.py --gpus $GPUS --data-dir $DATA_DIR --flist $FLIST --vlist $VLIST --model-name $MODEL_NAME --class-num $CLASS_NUM --epoch $EPOCH --init-model-name $INIT_MODEL --batch-size $BATCH_SIZE --epoch-size $EPOCH_SIZE --max-epoch 400 --batch-size $BATCH_SIZE --lr $LEARNING_RATE --worker-num $WORKER_NUM
python2 train.py --gpus $GPUS --data-dir $DATA_DIR --flist $FLIST --model-name $MODEL_NAME --class-num $CLASS_NUM --epoch $EPOCH --init-model-name $INIT_MODEL --batch-size $BATCH_SIZE --epoch-size $EPOCH_SIZE --max-epoch 400 --batch-size $BATCH_SIZE --lr $LEARNING_RATE --worker-num $WORKER_NUM
# python2 -m cProfile train.py --gpus $GPUS --data-dir $DATA_DIR --flist $FLIST --model-name $MODEL_NAME --class-num 59 --epoch $EPOCH --init-model-name $INIT_MODEL --epoch-size $EPOCH_SIZE --max-epoch 400 --batch-size $BATCH_SIZE --lr '5e-4' --rotate 'False' 2>
