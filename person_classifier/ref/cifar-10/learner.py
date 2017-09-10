#python library
import argparse
import time
import numpy as np

#chainer libraty
import chainer
from chainer.datasets import tuple_dataset
import chainer.functions as F
import chainer.links as L
from chainer import training
from chainer.training import extensions
from chainer import serializers

from chainer import cuda, Function, gradient_check, Variable, optimizers, utils
from chainer.functions.loss.mean_squared_error import mean_squared_error
from chainer.functions.loss.sigmoid_cross_entropy import sigmoid_cross_entropy

#python scripts
import load_dataset as ld
import network_structure as nn


#main
if __name__ == '__main__':

    # Load CNN model
    # model = nn.CNN_classification_0() # first model (only CPU)
    # model = nn.CNN_classification_1() # good_result_1 (GPU)
    # model = nn.CNN_classification_2() # good_result_2 (GPU)
    model = nn.CNN_classification()   # good_result_3 (GPU)

    # Setup optimizer
    optimizer = chainer.optimizers.Adam()
    optimizer.setup(model)

    train_N = 40000
    validation_N = 4000

    # parse args
    parser = argparse.ArgumentParser(description='CIFAR10 CLASSIFER')
    parser.add_argument('--batchsize', '-b', type=int, default=100,
                        help='Number of images in each mini batch')
    parser.add_argument('--epoch', '-e', type=int, default=100,
                        help='Number of sweeps over the dataset to train')
    parser.add_argument('--gpu', '-g', type=int, default=-1,
                        help='GPU ID (negative value indicates CPU)')
    args = parser.parse_args()

    # load dataset
    print "\n"+"loading the dataset.."+"\n"
    Xt,Yt,Xv,Yv = ld.load_dataset(train_N,validation_N)

    # train = zip(Xt,Yt)
    # validation = zip(Xv,Yv)

    train = tuple_dataset.TupleDataset(Xt, Yt)
    validation = tuple_dataset.TupleDataset(Xv, Yv)

    print "\n"+"the dataset is loaded!!"+"\n"
    print "train_N: " + str(len(Xt))
    print "validation_N: " + str(len(Xv)) + "\n"
    print 'minibatch-size: ' + format(args.batchsize)
    print 'epoch: ' + format(args.epoch) + "\n"

    if args.gpu >= 0:
        chainer.cuda.get_device(args.gpu).use()
        model.to_gpu()
        print 'using GPU \n'

    start_time = time.time() #start time measurement
    train_iter = chainer.iterators.SerialIterator(train, args.batchsize)
    validation_iter = chainer.iterators.SerialIterator(validation, args.batchsize,repeat=False, shuffle=False)

    updater = training.StandardUpdater(train_iter, optimizer, device=args.gpu)
    trainer = training.Trainer(updater, (args.epoch, 'epoch'))

    trainer.extend(extensions.Evaluator(validation_iter, model, device=args.gpu))
    trainer.extend(extensions.snapshot())
    trainer.extend(extensions.LogReport())
    trainer.extend(extensions.PrintReport(['epoch', 'main/loss', 'validation/main/loss','main/accuracy', 'validation/main/accuracy']))
    trainer.extend(extensions.ProgressBar())
    trainer.run()

    execution_time = time.time() - start_time

    print "execution time : " + str(execution_time)

    if args.gpu >= 0:
        model.to_cpu()

    serializers.save_npz('cnn_gpu.model', model)
    serializers.save_npz('cnn_gpu.state', optimizer)
