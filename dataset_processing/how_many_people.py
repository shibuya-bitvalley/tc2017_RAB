# -*- coding: utf-8 -*-

# Python libraries
import os
import numpy as np
from numpy.random import *
from matplotlib import pylab as plt
import random

# OpenCV
import cv2


# main
if __name__ == '__main__':

    dataset_path = '../../dataset/images/overlaped/2017_08_18/for_training/humans/'

    total_image = 0
    total_orange = 0
    total_blue = 0

    for d in range(10):

        orange_cnt = 0
        blue_cnt = 0

        label_data = []

        image_path = dataset_path + str(d+1) + '/'

        label_file = open(image_path+'labels.txt','r')

        for label_f in label_file:
            label_str = label_f
            label_data.append(int(label_str))

        data_N = len(label_data)

        for i in range(data_N):

            img = cv2.imread(image_path+str(i)+'.jpg')

            height = img.shape[0]
            width = img.shape[1]

            if label_data[i] == 1:
                orange_cnt += 1

            elif label_data[i] == 2:
                blue_cnt += 1

        total_image += data_N
        total_orange += orange_cnt
        total_blue += blue_cnt

        print
        print 'directory: '+str(d+1)
        print ' image: '+str(data_N)
        print ' orange: '+str(orange_cnt)
        print ' blue: '+str(blue_cnt)

    print
    print 'total image: '+str(total_image)
    print 'total orange: '+str(total_orange)
    print 'total blue: '+str(total_blue)
