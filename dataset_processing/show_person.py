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
    end_flag = False
    fontType = cv2.FONT_HERSHEY_SIMPLEX

    while(True):

        d = random.randint(1,24)

        label_data = []

        print
        print 'directory: '+str(d)

        image_path = dataset_path + str(d) + '/'

        label_file = open(image_path+'labels.txt','r')

        for label_f in label_file:
            label_str = label_f
            label_data.append(int(label_str))

        data_N = len(label_data)

        image_index = random.randint(0,data_N-1)
        print str(image_index)+'.jpg'

        img = cv2.imread(image_path+str(image_index)+'.jpg')

        height = img.shape[0]
        width = img.shape[1]

        if label_data[image_index] == 1:
            print "ORANGE"
            cv2.putText(img, "ORANGE", (5, height-20), fontType, 0.8, (0,165,255), 2, cv2.CV_AA)
        elif label_data[image_index] == 2:
            print "BLUE"
            cv2.putText(img, "BLUE", (5, height-20), fontType, 0.8, (255,125,86), 2, cv2.CV_AA)

        cv2.imshow("image", img)

        while (True):

            key = cv2.waitKey(1) & 0xFF

            # load next image
            if key & 0xFF == ord("z"):
                break

            # finish this program
            if key & 0xFF == 27:
                end_flag = True
                break

        if end_flag == True:
            break

    cv2.destroyAllWindows()
