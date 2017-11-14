import logging
import argparse
import mxnet as mx
from cores.io.TrainDataIterator import TrainDataIterator
from cores.io.EvalDataProducer import EvalDataProducer
import cores.symbols.refineNet101 as net
import os
from cores.utils import misc
from cores.utils import metrics


LOG_FILE_NAME = "traininglog.log"
EVAL_LOG_FILE_NAME = "evallog.log"
SNAPSHOT_FOLDER = "snapshots"

def main():
    logging.info("Training parameters are:")
    logging.info(args)
    ctx = [mx.gpu(int(i)) for i in args.gpus.split(',')]

    if args.mirror:
        os.environ["MXNET_BACKWARD_DO_MIRROR"] = "1"
    if not args.autotune:
        os.environ["MXNET_CUDNN_AUTOTUNE_DEFAULT"]="0"

    data_producer = TrainDataIterator(
        root_dir=args.data_dir,
        crop_size=args.crop_size,
        flist_path=os.path.join(args.data_dir, args.flist),
        batch_size=args.batch_size,
        shuffle=True,
        epoch_size=args.epoch_size,
        random_flip=False,
        random_crop=False,
        random_scale=False,
        random_rotate=False,
        data_worker_num=args.worker_num,
        label_shrink_scale=1.0/4
    )

    network_prefix = os.path.join(SNAPSHOT_FOLDER, args.model_name)
    snapshot_file = '%s-%d.params' % (network_prefix, args.epoch)
    init_model_prefix = os.path.join(SNAPSHOT_FOLDER, args.init_model_name)
    init_model_file = '%s-%d.params' % (init_model_prefix, args.epoch)
    print("Init Model:", init_model_prefix)
    arg_dict = {}
    aux_dict = {}
    if args.epoch == 0:
        symbol = net.create_training(args.class_num, args.workspace)
        if not os.path.exists(init_model_file):
            logging.warn("No model file found. Start from scratch!")
        else:
            arg_dict, aux_dict, _ = misc.load_checkpoint(init_model_prefix, args.epoch, load_symbol=False)
    else:
        arg_dict, aux_dict, symbol = misc.load_checkpoint(init_model_prefix, args.epoch)

    initializer = mx.initializer.Normal()

    mod = mx.mod.Module(symbol, context=ctx)
    mod.bind(data_shapes=data_producer.provide_data,
            label_shapes=data_producer.provide_label)
    mod.init_params(initializer=initializer, arg_params=arg_dict, aux_params=aux_dict, allow_missing=(args.epoch == 0))

    opt_params = {"learning_rate":args.lr,
                "wd": args.wd,
                'momentum': args.momentum,
                'rescale_grad': 1.0/len(args.gpus.split(','))}
    mod.fit(data_producer,
            optimizer=args.optimizer,
            optimizer_params=opt_params,
            num_epoch=args.max_epoch,
            epoch_end_callback=misc.module_checkpoint(network_prefix),
            batch_end_callback=misc.Speedometer(args.batch_size, frequent=10),
            eval_metric=[metrics.Accuracy(), metrics.Loss()],
            begin_epoch=args.epoch+1,
            kvstore='device')

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Training parameters")
    parser.add_argument("--epoch", default=0, type=int,
                        help="Starting epoch.")
    parser.add_argument("--max-epoch", default=200, type=int,
                        help="Max epoch.")
    parser.add_argument("--gpus", default="0",
                        help="Device indices for training.")
    parser.add_argument("--vgpus", default="0",
                        help="Device indices for validation.")
    parser.add_argument("--data-dir", default="VOC_DATA",
                        help="Directory for dataset.")
    parser.add_argument("--flist", default="train.txt",
                        help="File list for dataset.")
    parser.add_argument("--vlist", default="validation.txt",
                        help="File list for validation.")

    parser.add_argument("--batch-size", default=1, type=int,
                        help="Batch size.")
    parser.add_argument("--model-name", default="refineNet101",
                        help="Model name.")
    parser.add_argument("--init-model-name", default="refineNet101",
                        help="Initial Model name.")
    parser.add_argument("--class-num", default=41, type=int,
                        help="Class number.")
    parser.add_argument("--crop-size", default=448, type=int,
                        help="Image inputsize.")
    parser.add_argument("--epoch-size", default=200, type=int,
                        help="Epoch size.")


    parser.add_argument("--workspace", default=1000, type=int,
                        help="Workspace size.")
    parser.add_argument("--worker-num", default=2, type=int,
                        help="Number of data workers.")


    parser.add_argument("--optimizer", default="sgd",
                        help="Optimizer.")
    parser.add_argument("--lr", default=1e-4, type=float,
                        help="Learning rate.")
    parser.add_argument("--wd", default=5e-4, type=float,
                        help="Weight decay.")
    parser.add_argument("--momentum", default=0.9, type=float,
                        help="Momentum.")
    parser.add_argument("--mirror", default=False, type=bool,
                        help="Use mirror.")
    parser.add_argument("--rotate", default=True, type=bool,
                        help="Use random rotation.")
    parser.add_argument("--autotune", default=True, type=bool,
                        help="Use auto tuning.")


    args = parser.parse_args()
    LOG_FILE_NAME = "traininglog_" + args.model_name + ".log"
    EVAL_LOG_FILE_NAME = "evallog_" + args.model_name + ".log"

    if args.epoch==0:
        if os.path.exists(LOG_FILE_NAME):
            os.remove(LOG_FILE_NAME)
        if os.path.exists(EVAL_LOG_FILE_NAME):
            os.remove(EVAL_LOG_FILE_NAME)
    if not os.path.exists(SNAPSHOT_FOLDER):
        os.mkdir(SNAPSHOT_FOLDER)
    logging.basicConfig(filename=LOG_FILE_NAME, level=logging.INFO)
    console = logging.StreamHandler()
    logging.getLogger().addHandler(console)
    main()
