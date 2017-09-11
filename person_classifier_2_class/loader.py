#python library
import matplotlib.pyplot as plt
import numpy as np
from numpy.random import *
import time

#openCV
import cv2

#chainer library
import chainer
import chainer.functions as F
import chainer.links as L
from chainer import training
from chainer import serializers

#python script
import network_structure as nn
import load_dataset as ld
import visualizer as v


#validation using learned CNN
def validation(size,model,x,y):

    validation_label = []
    success_counter = 0
    mistake_counter = 0
    end_flag = False

    for i in range(len(x)):

        input_img = cv2.cvtColor(cv2.imread(x[i]),cv2.COLOR_BGR2RGB)
        img = cv2.resize(input_img, (size,size), interpolation = cv2.INTER_LINEAR)
        img = img.reshape((1,3,size,size)).astype(np.float32) / 255.

        start = time.time()
        validation_output = model.forward(chainer.Variable(img))
        print
        print validation_output.data
        label =  np.argmax(validation_output.data)
        elapsed_time = time.time() - start

        validation_label.append(label)

        print str(i+1) + " actual: " + str(y[i]) \
              + " estimated: " + str(label) \
              + " time:{0:.5f}".format(elapsed_time) + " [sec]"

        # visualize result
        #end_flag = v.result_visualizer(input_img, label)

        if label == y[i]:
            success_counter += 1.0

        if label == 1 and y[i] == 0:
            mistake_counter += 1.0

        if end_flag == True:
            break

    cv2.destroyAllWindows()

    success_rate = (success_counter/i)*100
    mistake_rate = (mistake_counter/i)*100

    print "test accuracy: " + str(round(success_rate,2))
    print "mistake rate: " + str(round(mistake_rate,2))

    return validation_label


#main
if __name__ == '__main__':

    test_N = 1000

    # model_path = 'thibault_model/'
    # model_name = 'cnn_gpu.model'
    # size = 50
    # model = nn.CNN_thibault()

    model_path = 'thibault_model2/'
    model_name = 'cnn_gpu.model'
    size = 50
    model = nn.CNN_thibault()

    serializers.load_npz(model_path+model_name, model)
    optimizer = chainer.optimizers.Adam()
    optimizer.setup(model)

    Xts, Yts = ld.load_test_dataset(test_N)
    Ye = validation(size,model,Xts,Yts)

    # Xtr, Ytr, Xv, Yv = ld.load_dataset(10, 100)
    # Ye = validation(size,model,Xv,Yv)

    #v.loss_visualizer(model_path)
    plt.show()
