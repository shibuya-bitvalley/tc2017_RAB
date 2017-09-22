#python library
import matplotlib.pyplot as plt
import numpy as np
from numpy.random import *
import time

#openCV
import cv2

#chainer library
import chainer
from chainer import cuda
import chainer.functions as F
import chainer.links as L
from chainer import training
from chainer import serializers

#python script
import network_structure as nn
import load_dataset as ld


# load_person classifier
class PersonClassifier:
    def __init__(self):

        model_path = 'thibault_model5/'
        model_name = 'cnn_gpu.model'
        model = nn.CNN_thibault2()
        self.size = 50

        print ("loading classifier model...")
        serializers.load_npz(model_path+model_name, model)
        optimizer = chainer.optimizers.Adam()
        optimizer.setup(model)

        #chainer.cuda.get_device(0).use()
        #model.to_gpu()

        self.model = model

    def __call__(self, img):

        img = cv2.resize(img, (self.size,self.size), interpolation = cv2.INTER_LINEAR)
        img = img.reshape((1,3,self.size,self.size)).astype(np.float32) / 255.
        #validation_output = self.model.forward(chainer.Variable(cuda.cupy.array(img))).data
        validation_output = self.model.forward(chainer.Variable(img)).data
        prob = F.softmax(validation_output).data[0][1]
        label =  np.argmax(validation_output)

        img = None
        validation_output = None

        return label, prob
